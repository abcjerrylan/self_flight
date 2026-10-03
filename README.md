# self_flight：独立四旋翼飞控

起步板：MicoAir743v2-AIO-35A；后续迁移到自研 STM32 飞控板。

已完成 **P0/P1 和 P2A 软件部分**：纯 C++ 核心、真实 H743 CubeMX 工程、ThreadX 启动、微秒时间、USB 虚拟串口日志和完整固件构建。USART1 未初始化，PA9/PA10 留给其他用途。用户已自行烧录，应用启动、USB 枚举和每秒日志已实测通过；计时精度及其他实机项目待验证。IMU、姿态解算、控制、DShot、RC 与解锁状态机属于后续阶段。

统一使用 `main` 分支，不按版本新建分支，不使用 `codex/` 前缀。代码优先简洁直接。

## 电脑端构建

依赖 CMake ≥ 3.22、Ninja、支持 C++17 的原生编译器，不需要 HAL、ThreadX 或 ARM 工具链。

```text
cmake --preset host-debug
cmake --build --preset host-debug
ctest --preset host-debug
```

Release 使用 `host-release` preset。MinGW 可在首次 configure 添加 `-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++`。本机路径放到私有 `CMakeUserPresets.json`，不写入共享文件。

## H743 固件构建

依赖 PATH 中的 `arm-none-eabi-gcc/g++`、CMake 和 Ninja。厂商源已包含在仓库，正常构建不需要 CubeMX，也不读取本机固件包或参考工程。

```text
cmake --preset mcu-debug
cmake --build --preset mcu-debug
cmake --preset mcu-release
cmake --build --preset mcu-release
```

输出在 `build/mcu-debug` 或 `build/mcu-release`：`self_flight.elf`、`.hex`、`.bin`、`.map`。当前是 **Flash 0x08000000 的独立启动配置**，不是已确认的 bootloader 应用布局；连接实物后先确认是否保留 bootloader，再确定烧录地址。

## 结构

| 位置 | 内容 |
| --- | --- |
| `flight/core/` | 原创数学、时间检查、数据契约，无 HAL/RTOS 依赖 |
| `app/app.cpp` | 一个启动线程，每秒打印系统时间 |
| `platform/stm32h7/` | TIM2 微秒时间、USB CDC ACM 输出、时钟查询 |
| `boards/micoair743v2_aio35/` | IOC、真实生成代码、HAL/CMSIS/ThreadX/USBX、板级构建 |
| `cmake/arm-gcc.cmake` | Cortex-M7 硬浮点工具链 |
| `tools/` | CubeMX 重新生成与输出路径整理 |
| `tests/host/` | 14 组电脑端测试 |
| `third_party/` | 依赖版本、许可、官方包与源码摘要 |
| `docs/` | 架构、契约、参考审查、实际验证和进度 |

核心目标 `self_flight::core` 在 host 与 MCU 中使用相同源文件。PNX 仅供只读参考，没有模块、子模块或 include/link 依赖。

阅读：[板级配置和接板检查](boards/micoair743v2_aio35/README.md)、[实际验证](docs/validation.md)、[进度](docs/progress.md)、[架构](docs/architecture.md)、[数据契约](docs/data-contracts.md)、[依赖](third_party/README.md)。
