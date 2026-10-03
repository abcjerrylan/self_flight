# P2B：BMI088 原始采样

本轮实现范围是原创驱动、真实 SPI/DRDY 接入、两路测量/可用时间、丢样/延迟统计和原始日志回放。标定补齐轮已实现静止陀螺偏置、加计六面标定与质量判定；实机修正验收状态见 [标定说明](imu-calibration.md)。软件滤波与姿态估计后续在P3实现。

## 配置与来源

引脚按[微空 AIO-35A 手册](https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)核对；协议按 [Bosch BMI088 数据手册 1.9](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)实现，没有复制 PNX 或 Bosch 驱动源。

| 项目 | 当前配置 |
| --- | --- |
| SPI2 | PD3 SCK、PC2_C MISO、PC3_C MOSI；8-bit/MSB/mode 3，软件片选 |
| 时钟 | HSE8 / PLL3 M4 N192；P4 为 SPI 提供 96MHz，分频16为6MHz；Q8 保持 USB 48MHz |
| 数字通路 | PC2_C/PC3_C 到 PC2/PC3 的开关闭合；原样生成 MSP 的 USER CODE 钩子保存此设置 |
| 片选 | PD4 accel、PD5 gyro，高电平空闲；PA15 的 BMI270 CS 也保持高，SPI3未启用 |
| DRDY | PC14 accel、PC15 gyro，上升沿，EXTI15_10 优先级5 |
| 加计 | ID 0x1e；±12g，800Hz，正常滤波；INT1/2 均配置 DRDY 输出 |
| 陀螺 | ID 0x0f；±2000deg/s，1000Hz，116Hz带宽；INT3/4 均配置 DRDY 输出 |
| 总线界限 | 每次一个完整片选周期，HAL SPI 超时2ms；传输结束必定拉高 CS |

手册只标注 DRDY 网名，未指出它接到哪一个 BMI088 INT 引脚，因此同一传感器的两路 INT 均映射 DRDY。写配置后保留2ms间隔，软复位/启动等待40ms，读回全部配置；陀螺带宽寄存器的只读 bit7 按掩码忽略。失败打印错误/ID/寄存器，采样不启用；不使用固定零值假装有效。

未使用 DMA、Cache、温控、FIFO、硬件同步模式或其他板的偏置。HAL的2ms是毫秒tick超时参数，不是测量证明的严格2ms执行上界；中断和调度会影响实际耗时，日志记录它。

## 所有权与时间

采样线程优先级5、静态栈4096；USB线程优先级15、静态栈4096，USBX初始化线程保持原配置。唯一采样线程拥有 SPI2。

ISR 只记录各自 TIM2 时间/序号并设置 ThreadX event flag。每路只保存最新事件，多个未处理事件合并；线程优先处理陀螺，再处理加计，分别记录开始读取和数据完整可用时间。`measured_us` 是 DRDY ISR 中读到的 MCU 时间，包含中断响应和传感器内部滤波延迟，不是精确的芯片内部采样时刻。

时间扩展、事件取出及共享快照使用短 PRIMASK 临界区，恢复进入前状态，支持 ISR/线程调用；临界区不覆盖 SPI/USB等待。TIM2 仍需至少每71.6分钟读取一次，本轮服务持续读取。没有声称已实测回绕精度。

读取期间又到达新 DRDY，或可用时已超过该传感器一周期的数据，标记 invalid 并计数。SPI失败不会发布有效样本；快照超过5ms也失效。加计与陀螺独立保存时间，不把它们伪装为同步数据。

`missed` 计数是已观察到的 DRDY 事件中被最新值覆盖的数量；不保证发现中断屏蔽期间完全未观察到的边沿。`overlap` 为读取中又到新事件，`stale` 为超周期，`spierr` 为传输错误。`latmax` 是可用时间减DRDY时间，`readmax` 是可用时间减开始读取时间，单位us；所有成对统计均为 accel/gyro。

## 原始值与坐标

驱动保留芯片轴的 int16 原始计数。SI比例为 accel `12*9.80665/32768` m/s²/LSB，gyro `2000*pi/(180*32768)` rad/s/LSB；24位 sensor_time 只属于加计，不当作 MCU 微秒。

板级预期变换 `(x_b,y_b,z_b)=(-y_s,-x_s,-z_s)` 来自微空手册链接的 [ArduPilot MicoAir743v2 硬件定义](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/MicoAir743v2/hwdef.dat)中 `ROLL_180_YAW_270`，为右手正交变换。本轮用户协助逐轴摆放/转动后，主轴与正负方向已经确认，记录见validation；这不是精密安装角校准。回放同时保留sensor SI与body FRD。改变板卡安装方向时需要重新确认。标定状态不由本轮方向检查决定，见标定说明。

## USB CSV 与回放

256条固定容量 ThreadX队列保存原始记录，采样线程无等待发送；满时丢弃最新日志并计数。USB线程把多条CSV合并成小于512字节的批次。未连接/DTR关闭/发送失败也累计 logdrop，不影响采样；USB发送错误后仍按原策略需要重插恢复日志。因此刚打开时 logdrop 非零正常，检查采集窗口内的增量和序号连续性。

逐样本格式（CSV v1）：

```text
IMU,A或G,sequence,measured_us,started_us,available_us,raw_x,raw_y,raw_z,sensor_time,valid
```

时间为十进制微秒。格式化使用秒/余数拼接，避免依赖 nano printf 的 long long；秒字段32位，持续运行136年后不再满足此输出格式。每秒另有 `# BMI088` 初始化诊断及 `# STATS`；回放忽略注释。

在仓库根目录，先关闭占用COM端口的串口工具，再执行：

```powershell
./tools/capture-imu.ps1 -Port COM51 -Seconds 20 -Output build/p2b/imu.csv
python tools/replay-imu.py build/p2b/imu.csv --output build/p2b/imu-si.csv
python tests/host/replay_tests.py
```

采集脚本自动启用DTR，用大接收缓冲连续读原始字节，结束后关闭端口。回放忽略采集末尾未写完的最后一行，但完整格式错误行仍报错。不要用缓慢的逐行终端刷新代替完整速率记录。回放输出两路事件速率、序号缺口、无效数、dt范围、延迟/读取上界、轴均值/标准差和比力模长。这些是当前记录的统计，不自动认定静止、合格标定或可飞行。

## 实机验收

1. 烧入当前HEX、正常启动，`init=1 err=0 aid=1e gid=0f`；否则根据src/reg/expect/got定位。
2. 静止采集20秒，检查约800Hz/1000Hz、两路序号与时间、有效样本、窗口内错误/丢样/日志缺口，保存原始CSV和回放报告。
3. 元件面朝上、箭头按预期机头方向放置，静止比力应约 `(0,0,-9.81)`；分别让三个机体轴正方向朝上/朝下，检查比力符号和主轴。手动正向绕三个轴旋转，检查对应陀螺符号；这是实物方向验证，不能由主机矩阵测试替代。
4. 测试关闭/重开串口、物理USB拔插、复位和传感器失效。连续时间回绕及电机脚电平仍属于尚待完成的P2A项目。

实际验证状态和结果见 [validation.md](validation.md)、[progress.md](progress.md)。电机引脚仍为低电平GPIO，没有 PWM/DShot。
