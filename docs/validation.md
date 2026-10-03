# 验证记录

2026-10-03。保留 P0/P1 和最初 USART1 版 P2A 记录，末尾为当前 USB 日志版结果。工作目录为最终独立仓库根目录。

## 原生构建

工具：CMake 3.28.1、Ninja 1.11.1、MinGW GCC/G++ 8.1.0、Git 2.51.0.windows.2。

```text
cmake --preset host-debug -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build --preset host-debug
ctest --preset host-debug
cmake --preset host-release -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build --preset host-release
ctest --preset host-release
```

| 检查 | 实际结果 |
| --- | --- |
| Debug configure / build | 通过，生成静态核心库与原生测试程序，警告作为错误 |
| Debug CTest | 14/14 通过，0 失败，约 0.26 秒 |
| Release configure / build | 通过，使用优化/NDEBUG，测试 CHECK 仍执行 |
| Release CTest | 14/14 通过，0 失败，约 0.21 秒 |

测试覆盖向量与右手坐标、身份和已知轴旋转、非交换组合顺序、逆与 q/-q 等价、缩放归一化/大数、零/近零/NaN/Inf、失败不修改输出、时间边界/倒退/重复/超期、样本有效性和安全默认值。另有 128 组确定性旋转检查，验证长度、内积、逆旋转和组合；不是硬件模拟。

本机最初在临时工作目录内的沙箱 configure 停留在编译器 ABI 检测，未产生完成结果，已停止该次进程。未把这次尝试记作通过，也没有把旧 build 复制到新仓库。最终目标目录以正常本机工具运行上面的全新构建并通过；环境停滞的具体原因未进一步确定。

## 独立性与边界

1. 检查 Debug `compile_commands.json`：3 个编译项全部来自本仓库，只包含本仓库核心 include，原生 G++ 编译；没有 PNX、HAL、ThreadX 或 ARM 工具链 include/link 输入。共享 CMake/presets 没有本机绝对路径，也没有子模块。
2. `cmake -S . -B build/mcu-gate -DSELF_FLIGHT_TARGET=mcu`：按预期失败，明确要求 P2 真实 H743 生成；诊断保存在 `build/validation/mcu-gate.txt`。
3. `cmake -S . -B build/host-toolchain-gate -DCMAKE_TOOLCHAIN_FILE=unused-do-not-load.cmake`：按预期失败，host 不加载交叉工具链；诊断保存在 `build/validation/host-toolchain-gate.txt`。
4. 独立 Git 当前使用 `main`，没有提交、推送或添加远程；构建目录被忽略。P0 目标原先不存在，未覆盖用户项目。
5. 参考工程的结束 `git --no-optional-locks status --short` 与初次结果一致；[12 个抽查文件的 SHA-256](reference-fingerprints.json) 前后逐项一致。未在参考目录构建或生成文件。

## 同一核心的 ARM 编译检查

发现本机 GNU Tools for STM32 13.3.rel1，G++ 13.3.1。对 `math.cpp` 和 `time.cpp` 分别执行以下命令形式，两个目标文件均编译成功：

```text
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror -Iflight/core/include -c flight/core/src/math.cpp -o build/validation/math-arm.o
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror -Iflight/core/include -c flight/core/src/time.cpp -o build/validation/time-arm.o
```

这仅证明当前纯核心源可以编译为 ARM 对象；未链接启动/HAL/RTOS，未在目标执行，也不冻结未来完整固件的全部编译参数。

## P0/P1 当时的硬件状态

CubeMX 生成：未执行。MCU 固件编译链接：未执行。烧录/IMU/电机/飞行：未执行。

## P2A 真实生成与构建

工具：CubeMX 6.15.0、STM32CubeH7 V1.12.1、X-CUBE-AZRTOS-H7 3.4.0 / ThreadX 6.4.0、STM32CubeCLT 1.19.0 / ARM GCC 13.3.1。CMake/Ninja 与原生编译器同前。

使用 CubeMX CLI 实际生成 STM32H743VIHx（STM32H743VIH6 / TFBGA100），未把 H723 启动源改名。初次 CMake 模板的 Windows 路径错误没有记作通过；改用 STM32CubeIDE 模板生成真实文件，整理文件位置并使用独立 CMake。最终 `tools/generate-board.ps1` 在已保存 IOC 上实际重跑成功，未修改非 USER CODE 初始化代码。

```text
cmake --preset mcu-debug
cmake --build --preset mcu-debug -j 8
cmake --preset mcu-release
cmake --build --preset mcu-release -j 8
```

| 检查 | 实际结果 |
| --- | --- |
| IOC 重新生成 | 通过；静态 ThreadX 内存、TIM2/USART1/TIM6、400 MHz 时钟保持 |
| MCU Debug 编译/链接 | 通过，Flash 33652 B，RAM_D1 12344 B（含保留 heap/MSP 空间） |
| MCU Release 编译/链接 | 通过，Flash 23364 B，RAM_D1 12336 B（含保留 heap/MSP 空间） |
| 固件产物 | 两套均生成 ELF、HEX、BIN、map |
| 最终编译日志 | 无 warning/error；自写 C++ 使用警告作为错误 |
| host Debug 回归 | 14/14，通过，0.21 秒 |
| host Release 回归 | 14/14，通过，0.16 秒 |
| 厂商来源 | 268 个文件逐项 SHA-256 与安装的锁定官方包匹配；见 third_party/vendor-fingerprints.json |
| 参考工程 | 12 个抽查文件与 P0/P1 摘要逐项一致；未写入或构建参考工程 |

日志位于 `build/p2a/`：`generate-board.log`、`mcu-debug-build.log`、`mcu-release-build.log`、`host-debug-tests.log`、`host-release-tests.log`。构建目录没有纳入源码；可以从当前源码重新构建。

## P2A 链接检查

Release 的 `nm`、`objdump` 和 `readelf` 核对结果：

- `g_pfnVectors` / `__Vectors` 同为 **0x08000000**，初始 MSP **0x24080000**，向量表中的复位入口为 **0x08000901**（Thumb）。
- `SysTick_Handler` 与 `PendSV_Handler` 是 ThreadX 的强符号，向量表分别指向 **0x08000339** / **0x0800038d**（Thumb）；HAL 生成的 IRQ 文件未定义重复 SysTick/PendSV。
- `TIM6_DAC_IRQHandler` 为强符号，调用 HAL TIM6 时基；SVC 沿用启动文件的弱默认入口，当前 ThreadX GNU 端口不使用该入口。
- ELF 为 Cortex-M7 / FPv5-D16，参数使用 VFP 寄存器；整个 MCU 编译使用一致的 hard-float 配置。
- 共享 CMake、preset、IOC 没有本机固件库或 PNX 绝对路径。MCU 只从本仓库编译，host 仍只编译原生核心和测试。

链接和配置检查不证明真实频率、计时精度或硬件启动。

## P2A 尚待实机验证

用户确认硬件尚未连接。CubeProgrammer 枚举没有发现调试器/DFU/串口；未烧录任何固件。

待验证：实物版号、bootloader 布局、实际启动、UART 连续日志、配置和实际时钟、TIM2 计时误差、至少一次 32 位计数回绕、复位与电机脚电平。接板步骤见 boards/micoair743v2_aio35/README.md。P2A 不能标记为完整实机验收通过。

## USB 日志版：当前源码与产物

USART1 被用户其他用途占用，因此当前日志改为 USB OTG FS / CDC ACM。实际 CubeMX 重新生成并归一化成功；PA11/PA12、PLL3 48 MHz、PCD/IRQ 与 USBX 为真实生成配置，main/MSP 没有 USART1/PA9/PA10 初始化。自定义 USBX 启动、CDC 连接钩子仍只在 USER CODE 块中。

使用同一锁定的 X-CUBE-AZRTOS-H7 3.4.0 包，新增 USBX 6.4.0；没有引入 PNX 源。USBX 和 ThreadX 均为 1000 Hz。同步发送只在 CDC 激活并设置 DTR 后执行，100 ticks 为 100 ms；发送错误后的中止只针对当前有效实例，避免 USB reset 删除端点后再次访问。

| 检查 | 当前结果 |
| --- | --- |
| MCU Debug 编译/链接 | 通过，Flash 65100 B，RAM_D1 47848 B（含保留 heap/MSP 空间） |
| MCU Release 编译/链接 | 通过，Flash 42576 B，RAM_D1 47800 B（含保留 heap/MSP 空间） |
| 固件产物 | 两套 ELF、HEX、BIN、map 已更新 |
| host Debug / Release 回归 | 各 14/14，通过；0.25 / 0.20 秒 |
| 厂商来源 | 当前 406 个文件全部与锁定官方安装包 SHA-256 相同 |
| USB HAL 回调 | SetupStage/DataInStage/DataOutStage/ResetCallback 均为强 T 符号，USBX OBJECT 库接入生效 |
| 中断 | OTG_FS_IRQHandler、ThreadX SysTick/PendSV 均为强 T 符号 |
| 向量/ABI | 向量表 0x08000000，MSP 0x24080000；Release Reset 0x0800280d，SysTick 0x08000339，PendSV 0x0800039d（Thumb）；hard-float VFP 参数 ABI |
| 实机 | 本轮只读 CubeProgrammer 枚举无 DFU/调试器/串口；没有烧录或实际 USB 输出结果 |

构建和回归日志在 `build/usb/`，CubeMX 日志仍为 `build/p2a/generate-board.log`。旧 P2A 表中的资源数量和入口地址属于 USART1 版，不是当前 USB 版产物。

仍需实机验证正常 USB 枚举、串口 DTR、持续输出、关闭/重开、拔插与暂停读取时的超时行为，以及计时精度、回绕、复位和电机脚。步骤见 [板级 README](../boards/micoair743v2_aio35/README.md)。

## USB 实机连接：已枚举，读取暂受端口占用限制

用户已自行烧录并连接板卡。本轮只读 CubeProgrammer 和 Windows 串口查询均识别 COM51，名称 STMicroelectronics Virtual COM Port，PNPDeviceID 为 USB\VID_0483&PID_5740\SELF_FLIGHT_H743，驱动状态 OK。这确认当前应用的 USB 枚举已成功，不等于日志和计时实测通过。

尝试以 115200/8N1、DTR=true 打开 COM51；普通环境和排除沙箱限制后的本机环境均返回 Access denied，未能取得串口句柄，因此实际接收测试尚未开始。检测到 SerialDebug 程序运行，疑似占用端口；已请用户先断开其 COM51。未强制结束程序、未重新烧录、未更改固件或发送应用命令。

端口释放后待执行：打开串口、启用 DTR，读取至少 12 秒；根据实际日志判断发送是否正常。当前无可用接收日志，不能把此步骤记为通过。

## USB 实机接收：通过

用户断开串口工具后，COM51 成功打开，以 115200/8N1、DTR=true 连续监听 12 秒，收到 869 个字符 / 11 行完整日志，seq=406～416，t=406.999679～416.999679s，tick=407001～417001，所有行 dt_us=1000000。cpu=400000000 是固件读取 RCC 配置的结果，不是独立频率测量。

关闭串口后再次打开做 DTR 对比：DTR=false 的 3 秒收到 0 字符；改为 DTR=true 的 4 秒收到 316 字符 / 4 行，seq=477～480，时间连续增加。确认当前固件按设计要求 DTR 开启后才输出；串口关闭/重新打开没有阻止输出。原串口工具是否曾启用 DTR 未直接读取，其无消息很可能与该设置有关。

原始记录：build/usb/com51-hardware-probe.log、build/usb/com51-dtr-check.json。测试完成后均在 finally 中关闭并释放 COM51，用户可重新打开。没有更改固件、重新烧录、向设备发送应用命令或输出电机信号。

已验证正常枚举、应用启动、每秒日志、串口关闭/重开及 DTR 控制。仍未验证物理 USB 拔插、发送超时恢复、独立计时精度、TIM2 71.6 分钟回绕、复位和电机脚电平，P2A 全部实机项目仍未完成。


## P2B 软件与实机连续采样（2026-10-03）

真实CubeMX生成新增SPI2和EXTI；PLL3P4使SPI核时钟96MHz，prescaler16，mode3，GPIO HIGH speed。PC2_C/PC3_C数字开关在MSP USER CODE块闭合。USART1和电机定时器未启用。

| 检查 | 结果 |
| --- | --- |
| host Debug / Release | 各18/18通过（原核心14，BMI088驱动/错误/转换/安装矩阵4） |
| Python回放 | 3/3通过，覆盖32位序号回绕、64位时间、SI、缺口、错误字段和截断末行 |
| MCU Debug | Flash79,184B，RAM_D1 64,848B；ELF/HEX/BIN/map |
| MCU Release | Flash51,232B，RAM_D1 64,800B；ELF/HEX/BIN/map |
| 中断链接 | EXTI15_10、HAL_GPIO_EXTI_Callback、OTG_FS、HAL PCD回调、ThreadX SysTick/PendSV 均为强符号 |
| 厂商来源 | 411个厂商文件与锁定官方包逐字节一致；没有PNX include/link依赖 |
| 实机初始化 | init=1 err=0；accel ID=0x1e，gyro ID=0x0f，配置读回成功 |

用户通过USB DFU自行烧入新版并正常启动。COM51 / DTR=true，以大缓冲连续采集20秒，保存2,154,212字节。采集脚本结束后已释放COM51。原始日志build/p2b/imu-bench.csv，回放imu-bench-si.csv和imu-bench-report.json。

| 20秒采集 | 加计 A | 陀螺 G |
| --- | --- | --- |
| 有效记录 | 16,080 / 16,080 | 19,990 / 19,990 |
| 事件速率 | 803.4888Hz（配置800） | 998.8717Hz（配置1000） |
| 相邻测量时间范围 | 1238–1251us | 949–1053us |
| 最大测量到可用延迟 | 73us | 66us |
| 最大SPI读取耗时 | 39us | 42us |
| 序号缺口 / 重复 / 倒退 | 0 / 0 / 0 | 0 / 0 / 0 |
| 无效 / missed / spierr / overlap / stale | 全部0 | 全部0 |

速率依据MCU DRDY时间计算，不是独立时钟校准。约0.44%和-0.11%的标称速率偏差保留原样；中断响应也贡献单次时间抖动。未证明不可观测的DRDY边沿绝对没有丢失。全运行累计latmax=77/78us、readmax=42/49us；当前窗口统计更小。

窗口首尾logdrop均520929，增量0。该累计值来自开始采集前串口/DTR未打开时主动舍弃的日志，不能算成这20秒的采样丢失。初始化后到末条状态约309秒，missed/spierr/overlap/stale仍0。

静止body加速度均值(-0.2214,-0.0213,-9.7923)m/s²，比力模长均值9.79605m/s²；陀螺均值(-0.001523,+0.001203,-0.002793)rad/s。没有应用偏置或加计标定；手持/摆放误差和噪声不能由此区分。

逐轴物理检查结果如下。传感器失效注入、物理拔插、独立计时精度、TIM2回绕和电机脚电平尚未测量。


## P2B 实物三轴方向检查

以用户板上机头箭头为X前，右为Y，元件面朝上时Z下；用户确认每次摆好/转完后采集。预期变换(-y_s,-x_s,-z_s)没有因本轮检查改动。静态三姿态各记录4秒，元件面朝上用最初20秒基线。

| 摆放 | body加速度均值 (X,Y,Z)，m/s² | 方向判定 |
| --- | --- | --- |
| 元件面朝上 | (-0.221,-0.021,-9.792) | Z负 |
| 机头箭头朝上 | (+9.709,+0.061,+0.761) | X正 |
| 右侧朝上 | (-0.574,+9.815,-0.145) | Y正 |
| 元件面朝下 | (-1.674,+0.076,+9.647) | Z正 |

摆放有倾斜，尤其翻面时X分量约-1.67；只能验证轴与符号，不能将其解释为加计误差或六面标定。上述三个4秒窗口两路均0无效/序号缺口。

陀螺记录先打开USB，再提示用户从水平姿态分别右侧下压、抬机头、从上方看顺时针转机头并保持。X窗口30秒，Y/Z各60秒。对各轴abs(w)>0.15rad/s的样本按DRDY时间积分，仅用于观测运动主轴与符号：

| 动作 | 正向积分 (X,Y,Z)，deg | 负向积分 (X,Y,Z)，deg | 判断 |
| --- | --- | --- | --- |
| 右侧下压 | (+44.98,+0.23,+0.92) | (-0.96,-0.14,-2.37) | X正为主 |
| 抬机头 | (+50.03,+114.45,+29.05) | (-53.60,-42.98,-44.08) | Y净正约71.47，其他轴手持调整 |
| 顺时针转机头 | (+6.91,+5.61,+96.36) | (-7.60,-5.91,-5.39) | Z净正约90.97 |

X动作记录没有完整90度，Y有明显手持调整；这些数值不是受控转台校准结果。三次动作的主轴和正向符合FRD，不能据此认定比例/偏置已标定。全部动作窗口两路序号缺口/重复/倒退/无效均0。

原始数据axis-x-up.csv、axis-y-up.csv、axis-z-up.csv、gyro-x-positive.csv、gyro-y-positive.csv、gyro-z-positive.csv均保存在build/p2b；axis-check-report.json为回放和运动观测报告。motion阈值只应用于报告，不应用于固件或后续控制。测试结束COM51已释放。P2B原始采样、时间/延迟统计、回放与基本方向验收通过；传感器失效注入未做，软件错误路径仅为主机模拟测试通过。


## P2B 标定补齐（2026-10-03）

本轮补齐原方案P2B中尚未实现的标定，没有把前轮采样/方向检查算成标定。原始CSV v1保持不变；新增共享C++静止窗口、六面拟合、参数验证与SI修正，以及每秒CAL/CORR诊断。未新增CubeMX外设或厂商依赖。

| 软件检查 | 实际结果 |
| --- | --- |
| host Debug | 22/22，0.44s；新4组为静止/拒绝/六面拟合/应用 |
| host Release | 22/22，0.36s；测试仍实际执行 |
| Python回放 | 4/4；新增传感器轴修正后再安装旋转，拒绝Unknown |
| 温度协议 | 原BMI088 data测试新增正温/负温/无效码/传输失败不改输出 |
| MCU Debug | Flash87,824B / RAM_D1 64,944B；ELF/HEX/BIN/map |
| MCU Release | Flash56,932B / RAM_D1 64,896B；ELF/HEX/BIN/map |

核心拒绝覆盖：运动、单路缺失、漏序号、重复时间、非法值、错误六面/倾斜、非法参数、Unknown校准；测试包含uint32序号回绕和超过uint32的时间。真实运动日志用同一C++静止核心回放，拒绝该窗口；没有另一套Python拟合公式。

六面每面实际8秒，共38,600条加计和47,987条陀螺，全部有效，序号/时间无缺口或倒退。参数和六个原始CSV的SHA-256保存在 [本板参数证据](calibration-micoair743v2-20261003.json)。拟合后六面模长均值9.80790–9.81803m/s²；这是训练数据，不作为独立验收。

用户通过USB DFU自行烧入新版，再静止采集独立20秒、2,173,134字节。CAL=gyro1/accel1/quality2/err0，启动3秒合格窗口a=2416/g=2998，restart=0。实际sensor-axis gyro bias=(-0.000701,+0.001319,+0.000889)rad/s，最大轴方差约2.355e-5(rad/s)²，记录温度30.375°C且valid=1。加计偏置/比例与本板生成参数一致。

| 独立20秒记录 | 加计 | 陀螺 |
| --- | --- | --- |
| 有效记录 | 16,082 / 16,082 | 19,990 / 19,990 |
| 事件速率 | 803.5834Hz | 998.8621Hz |
| 最大原始读取到可用延迟 | 80us | 88us |
| 最大SPI读取耗时 | 56us | 41us |
| 无效/序号缺口/重复/倒退 | 全部0 | 全部0 |
| 窗口missed/spierr/overlap/stale | 全部0 | 全部0 |

原始比力模长均值9.791287m/s²，修正后9.804991m/s²；修正body加计均值(-0.18403,+0.03465,-9.80312)m/s²，非零横向分量包含摆放倾斜。修正body陀螺均值(-0.0004914,+0.0003998,-0.0000578)rad/s，模长约0.000636rad/s。不能把短期均值接近零当作长期温漂补偿完成。

20条加计和20条陀螺CORR用相同序号逐条与原始CSV核对；与离线修正最大差分别1.018微m/s²、0.736微rad/s，在目标float及整数日志量化误差内。报告build/calibration/onboard-corrected-report.json；输入的陀螺bias使用实际板上CAL而非旧离线参考。

窗口logdrop首尾均222591，无新增；未开串口期间原始采样持续。此次正常启动用户重插USB后COM51枚举/日志正常。计时精度、71.6分钟回绕和电机脚电平仍未独立测量。

运动验证分开记录：第一次重启记录在36秒才开始，CAL已经Accepted，因此它只能证明接受后转动不会重新学习零偏；不将其误记为启动运动拒绝。随后用户在持续转动中重插USB，8秒onboard-motion-reject.csv所有CAL均gyro=0、quality=Unknown，Motion/Collecting并增加候选清空计数。放稳后的onboard-recovery.csv从a=1375/g=1709、Collecting，进展到a=2413/g=2999、Accepted；最终零偏(-0.000840,+0.001375,+0.000799)rad/s，温度32.125°C。证明运动不通过且静止后可自动恢复。restart计数是每次候选被清空的次数，不能当作完整3秒窗口重试数。

实机静止、运动拒绝和恢复记录结束均释放COM51。标定参数属于这一块实物；对角六面模型与启动静止零偏均通过当前基本验收，不声称精密转台、交叉轴/温漂模型或飞行验收。四路电机仍为低电平GPIO，未启用PWM/DShot。
