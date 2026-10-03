# 架构与构建边界

## 当前有效实现

`flight/core` 是纯 C++17 静态库：数学和时间检查已有实现，飞控数据对象已有定义。P2A 已加入真实 H743 启动、ThreadX、TIM2 时间和 USB CDC ACM 日志；P2B 加入原创 BMI088 驱动、单一采样服务与 CSV 回放。P2B标定补齐轮增加便携静止窗口、六面拟合和传感器SI修正，平台服务负责启动标定和参数接入。控制器、姿态估计和状态机尚未实现。

依赖关系为 `app → services/platform`、`services → core/drivers/platform`、`platform → board/HAL`，便携driver只依赖core；HAL回调通过服务入口递送DRDY事件。core 独立，不包含 HAL、ThreadX、板型或引脚，不获取硬件时间、不动态分配内存。业务算法未来在 `flight/core` 下增量扩展。

`board` 持有固定硬件绑定、安装变换、IOC、启动/链接和内存事实；platform 持有 HAL、RTOS、DMA、缓存与硬件时钟；driver 持有器件协议；service 管理采样所有权、时间、发布和任务协调。首版不需要运行时工厂、虚函数 BSP 或配置生成器。

## 构建

顶层先选择 host/mcu，再进入 CMake `project`。host 不设置ARM工具链、不加载板级CMake、不获取MCU依赖；安装变换测试只包含无HAL的板级profile头文件。源文件显式列出，没有递归 glob 或外部路径注入。

使用同一 `flight_core` 源码创建静态库，分别链接原生测试或 MCU 固件。host 的 `BUILD_TESTING=OFF` 可以只构建库。未知目标、host 传入工具链或 ARM 编译器用于 host 应明确失败。

MCU 板级集成加载真实 CubeMX 代码和锁定依赖。首次 `project` 前选定 ARM 工具链，使用独立 `build/mcu-debug` 与 `build/mcu-release`，不在原生 build 目录切换工具链。

## 运行契约

TIM2 扩展为 64 位微秒，P2B 用短 PRIMASK 临界区串行化 ISR/线程读取及回绕扩展；不能漏过完整回绕周期。core 只检查已经扩展的时间。控制器未来显式使用 `dt`，不把 RTOS tick 或固定 sleep 当作真实采样间隔。

USB 日志由低优先级线程同步发送，打开串口并设置 DTR 后输出，每个小于512字节批次的传输超时为100ms。P2B 使用256条固定容量队列；满或电脑不读取时丢日志并统计，采样不等待。异常发送中止后需重插 USB；发送不用于 ISR 或快速控制线程。USBX 与 ThreadX 时基均为1000Hz。

ISR 仅记录时间/序号与通知；当前唯一高优先级采样线程拥有SPI2，分别读取陀螺/加计，拒绝重叠或过旧的样本。最新状态以短临界区复制，原始日志流以ThreadX队列传递；`data.hpp` 的对象本身不是线程安全消息容器。详见 [P2B](p2b-imu.md)。

默认无效/禁止输出只用于降低误用风险。真正的解锁条件、故障锁存、边沿解锁和硬件输出门控将在 P4 实现与验证。
