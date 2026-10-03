# self_flight：独立四旋翼飞控

起步板：MicoAir743v2-AIO-35A；后续迁移到自研 STM32 飞控板。

已完成 **P0/P1、P2A 软件、P2B（含标定）和 P3**：纯 C++ 核心、真实 H743 CubeMX 工程、ThreadX、微秒时间、USB 日志，以及原创 BMI088 驱动、DRDY 采样和原始 CSV 回放。USART1 未初始化，PA9/PA10 留给其他用途。P2B的芯片ID、配置、20秒连续采样和三轴方向已实测通过；P2A的计时精度/回绕等待验项目见进度。IMU标定已完成实际六面采集、板上启动零偏与修正验证；P3实现原创低通、Mahony、残余零偏和USB姿态诊断，主机与固件编译通过，桌面静止与三轴方向验收已通过。控制、DShot、RC与解锁状态机尚未实现。

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
| `app/app.cpp` | 低优先级 USB 日志线程、逐样本CSV和每秒诊断 |
| `drivers/` | 原创、可原生测试的 BMI088 协议与原始值转换 |
| `flight/services/` | 单一 SPI 采样所有者、DRDY 时间、快照和固定日志队列 |
| `platform/stm32h7/` | TIM2 微秒时间、USB CDC ACM 输出、时钟查询 |
| `boards/micoair743v2_aio35/` | IOC、真实生成代码、HAL/CMSIS/ThreadX/USBX、板级构建 |
| `cmake/arm-gcc.cmake` | Cortex-M7 硬浮点工具链 |
| `tools/` | CubeMX 重新生成、USB原始采集和SI回放 |
| `tests/host/` | 37 组 C++ 测试和原始IMU/姿态Python回放测试 |
| `third_party/` | 依赖版本、许可、官方包与源码摘要 |
| `docs/` | 架构、契约、参考审查、实际验证和进度 |

核心目标 `self_flight::core` 在 host 与 MCU 中使用相同源文件。PNX 仅供只读参考，没有模块、子模块或 include/link 依赖。

阅读：[板级配置和接板检查](boards/micoair743v2_aio35/README.md)、[实际验证](docs/validation.md)、[进度](docs/progress.md)、[架构](docs/architecture.md)、[数据契约](docs/data-contracts.md)、[依赖](third_party/README.md)。

BMI088配置、CSV格式和采集回放见 [P2B说明](docs/p2b-imu.md)；标定流程、参数和质量门限见 [标定说明](docs/imu-calibration.md)。

P3参数、偏航边界、姿态日志与回放见 [姿态估计](docs/p3-attitude.md)。

USB 实时姿态页面：[attitude-viewer.html](tools/attitude-viewer.html)，用桌面 Chrome / Edge 打开后选择飞控串口；支持四元数板卡模型、机体轴累计转角曲线和本地日志回放；累计角需要烧录输出 total_kind=body 的新版固件，累计值会漂移，不用于姿态控制。使用说明见 [实时姿态页面](docs/attitude-viewer.md)。
