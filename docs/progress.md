# 进度

日期：2026-10-03。

## P0：已完成

- 检查目标目录原先不存在，检查目标祖先工作规则，读取参考工程分层、启动与 BSP/IMU 相关资料。
- 建立独立 Git 仓库；P0 当时未提交/推送。当前在用户已有的 `main` / `origin/main` 上继续工作。
- 完成 README、项目 AGENTS.md、独立 CMake/presets、忽略规则、来源与依赖说明。
- 完成参考审查，记录已有用户修改和文档/源码差异；结束状态与 12 个抽查文件摘要一致。
- 没有复制 PNX 模块、子模块、旧 build 或依赖，没有在参考工程构建。

## P1：已完成

- 实现向量加减/标量/dot/cross、Hamilton 四元数乘法/共轭、稳健归一化、轴角构造、向量旋转与有限值检查。
- 实现 64 位微秒时间契约、显式间隔错误与样本新鲜度检查；时间完全由调用者传入。
- 定义 IMU、标定、姿态、飞手指令、控制目标、四路输出和飞行状态数据对象；默认 invalid/禁止输出。
- 明确 FRD/NED、WXYZ、body→navigation、右侧旋转先作用、SI 单位、两路采样时间与失败不修改输出。
- host Debug 和 Release configure/build/CTest 均实际通过，分别 14/14；正常构建没有 ARM/HAL/ThreadX/PNX 依赖。
- MCU 未完成与 host 工具链拒绝门控验证通过；两份核心实现通过 Cortex-M7 对象编译检查。
- H743 板级资源与真实生成流程已有文档；基础 MCU 依赖将在 P2 验证后锁定，没有虚构版本或生成工程。

完整命令、工具版本、结果和限制见 [validation.md](validation.md)。

## P2A：软件完成，实机待验证

- CubeMX 6.15.0 实际生成 STM32H743VIH6 / TFBGA100 工程；保存真实 IOC、H743 启动/链接文件、HAL/CMSIS 与 ThreadX。
- 锁定 STM32CubeH7 V1.12.1、X-CUBE-AZRTOS-H7 3.4.0 / ThreadX、USBX 6.4.0；保留许可和包 SHA-256。406 个引入的厂商文件与官方包逐字节匹配。
- 实现 CPU 400 MHz、TIM2 1 MHz 微秒时间、HAL TIM6 / ThreadX SysTick 各自时基与一个静态启动线程。
- 根据 USART1 已被占用的反馈，日志改为 USB OTG FS / CDC ACM（PA11/PA12）；USART1/PA9/PA10 未初始化。保存实际重跑的 CubeMX 配置、USBX 静态内存和 USER CODE 接入，Debug/Release 再次编译通过。
- 四路电机脚配置为低电平 GPIO；没有 PWM/DShot。Flash 0x08000000 为明确的独立启动构建配置，bootloader 布局尚待确认。
- MCU Debug/Release 完整编译链接通过，产出 ELF/HEX/BIN/map；脚本重新生成后再构建成功。
- 向量表、MSP、ThreadX 中断入口与硬浮点属性核对通过。host Debug/Release 回归各 14/14 通过。
- 在原有 `main` 分支工作，未提交或推送本轮修改。参考工程 12 个文件摘要仍一致。
- 最初只验证软件；本轮用户自行烧录后，USB 枚举和连续日志已实测通过（见末尾）。计时误差和回绕尚未实测。

## 下一步

连接板卡后，确认版号与 bootloader 布局，通过 USB DFU 下载，再正常启动并读取 USB 虚拟串口，完成 [P2A 实机检查](../boards/micoair743v2_aio35/README.md)。然后进入 P2B，接入 BMI088 的 SPI、两路 CS/DRDY、器件 ID/配置与原始数据时间戳。

尚未实现或验证：IMU 驱动/标定计算、Mahony、PID、混控、DShot、RC、解锁状态机、持久化、烧录和飞行。现有数据类型不等于这些功能已存在。

## USB 首次接板检查

用户已自行烧录；COM51 已枚举，设备序列号 SELF_FLIGHT_H743，Windows 驱动状态 OK。用户释放串口后，DTR=true 的 12 秒监听收到 11 行日志；DTR=false 的 3 秒为 0 字符，重新开启后的 4 秒收到 4 行。应用启动、每秒日志、串口重开和 DTR 控制已验证；COM51 已释放。没有更改固件、重新烧录或输出电机信号。计时精度、回绕、物理拔插与电机脚等项目仍待验证。
