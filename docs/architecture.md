# 架构与构建边界

## 当前有效实现

`flight/core` 是纯 C++17 静态库：数学和时间检查已有实现，飞控数据对象已有定义。P2A 已加入真实 H743 启动、ThreadX、TIM2 时间和 USB CDC ACM 日志；控制器、状态机与 IMU 服务尚未实现。

依赖方向为 `app → services → core/drivers → platform → board/HAL`。core 独立，不包含 HAL、ThreadX、板型或引脚，不获取硬件时间、不动态分配内存。业务算法未来在 `flight/core` 下增量扩展。

`board` 持有固定硬件绑定、安装变换、IOC、启动/链接和内存事实；platform 持有 HAL、RTOS、DMA、缓存与硬件时钟；driver 持有器件协议；service 管理采样所有权、时间、发布和任务协调。首版不需要运行时工厂、虚函数 BSP 或配置生成器。

## 构建

顶层先选择 host/mcu，再进入 CMake `project`。host 不设置工具链、不查找板目录、不获取 MCU 依赖。源文件显式列出，没有递归 glob 或外部路径注入。

使用同一 `flight_core` 源码创建静态库，分别链接原生测试或 MCU 固件。host 的 `BUILD_TESTING=OFF` 可以只构建库。未知目标、host 传入工具链或 ARM 编译器用于 host 应明确失败。

MCU 板级集成加载真实 CubeMX 代码和锁定依赖。首次 `project` 前选定 ARM 工具链，使用独立 `build/mcu-debug` 与 `build/mcu-release`，不在原生 build 目录切换工具链。

## 运行契约

TIM2 扩展为 64 位微秒，P2A 只有启动线程读取，每秒检查一次回绕；不能并发调用或漏过完整回绕周期。P2B 在接入 ISR 采样时再实现其时间戳所有权。core 只检查已经扩展的时间。控制器未来显式使用 `dt`，不把 RTOS tick 或固定 sleep 当作真实采样间隔。

USB 日志由当前低频启动线程同步发送，打开串口并设置 DTR 后才输出，每次发送最多等待 100 ms。没有等待电脑连接的启动循环，也没有额外日志队列。异常发送中止后需重插 USB；发送不用于 ISR 或未来快速控制线程。USBX 与 ThreadX 时基均为 1000 Hz。

ISR 仅记录时间/状态与通知；后续快速线程拥有 IMU 采样与控制链路。样本流、最新状态和日志分开设计；`data.hpp` 的对象不是线程安全消息容器，需要 service 层提供一致发布与读取。

默认无效/禁止输出只用于降低误用风险。真正的解锁条件、故障锁存、边沿解锁和硬件输出门控将在 P4 实现与验证。
