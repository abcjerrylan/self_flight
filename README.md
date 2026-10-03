# self_flight：独立四旋翼飞控

起步板：MicoAir743v2-AIO-35A；后续迁移到自研 STM32 飞控板。

当前阶段是 **P0/P1：独立工程和可在电脑上验证的核心**。已实现基础向量/四元数、时间检查和数据契约。尚未实现 IMU 驱动、Mahony、PID、DShot、RC 或解锁状态机，也没有可烧录的 H743 固件。

开发统一在 `main` 分支推进，不按版本或里程碑新建分支，分支名称不使用 `codex/` 前缀。

## 主机构建与测试

依赖：CMake ≥ 3.22、Ninja、支持 C++17 的原生编译器（GCC/Clang/MSVC）。主机核心没有第三方库依赖，无需 HAL、ThreadX、CubeMX 或 ARM 编译器。

在本仓库根目录、能找到工具的终端执行：

```text
cmake --preset host-debug
cmake --build --preset host-debug
ctest --preset host-debug
```

优化构建同样保留检查，不依赖可能被禁用的 `assert`：

```text
cmake --preset host-release
cmake --build --preset host-release
ctest --preset host-release
```

可在首次 configure 时使用 `-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++` 明确选择 MinGW。若编译器不在 PATH，可用本机私有 `CMakeUserPresets.json` 配置；不要把机器路径写入共享 presets。

## 当前结构

| 位置 | 内容 |
| --- | --- |
| `flight/core/include/self_flight/core/` | 数学、时间和数据接口 |
| `flight/core/src/` | 原创实现，无 HAL/RTOS/硬件依赖 |
| `tests/host/` | 14 组 CTest，含坐标/组合、异常输入、时间与数据默认状态 |
| `boards/micoair743v2_aio35/` | 真实板资源规划与 P2 生成流程，当前只有设计文档 |
| `third_party/` | 依赖来源、工具验证版本和后续锁定策略；当前没有引入的库 |
| `docs/` | 架构、数据契约、参考审查、验证和进度 |

核心目标为 `self_flight::core`，未来 MCU 目标使用同一源文件。原生头文件根目录只有本仓库 `flight/core/include`，不加载参考工程。

## MCU 构建状态

MCU 实现属于 P2，参见 [板级生成步骤](boards/micoair743v2_aio35/README.md)。当前 `SELF_FLIGHT_TARGET=mcu` 明确报错，防止把主机程序当作固件。没有虚构 `.ioc`、启动文件、链接布局或伪造的 MCU preset。

## 阅读顺序

1. [架构边界](docs/architecture.md)
2. [坐标与数据契约](docs/data-contracts.md)
3. [参考工程审查](docs/reference-review.md)
4. [依赖策略](third_party/README.md)
5. [实际验证结果](docs/validation.md)
6. [进度与下一步](docs/progress.md)

本仓库与 PNX 独立。未复制其模块、子模块或厂商依赖；不能在参考仓库中构建。当前项目未对外发布，整体开源许可尚未选择；新增第三方内容必须保留自身许可并记录来源。
