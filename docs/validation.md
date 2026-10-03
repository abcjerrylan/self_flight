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
