# MicoAir743v2-AIO-35A 板级规划

状态：P1 设计文档。当前没有 `.ioc`、CubeMX 生成源、链接脚本、启动代码或可烧录固件。

## 资源

资源已在设计方案阶段对照官方附录，本轮尚未拿实物确认版号。

| 功能 | 资源 |
| --- | --- |
| MCU / HSE | STM32H743VIH6 / 8 MHz |
| BMI088 SPI2 | SCK PD3、MISO PC2、MOSI PC3 |
| 片选 | gyro PD5、accel PD4 |
| DRDY | gyro PC15、accel PC14 |
| M1 / M2 / M3 / M4 | PE14/TIM1_CH4、PE13/CH3、PE11/CH2、PE9/CH1 |
| RC USART6 | PC6 TX、PC7 RX |
| 起步日志 USART1 | PA9 TX、PA10 RX |
| SWD | PA13、PA14 |
| 后期 SPL06 | I²C2 PB10/PB11 |

来源：[微空 AIO-35A 手册](https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)。M1–M4 只是电气号，安装变换、机臂位置与旋向暂未配置，不能用于解锁。

## P2 真实生成步骤

1. 确认板型号/版号、芯片修订、接收机、调试器、现有 bootloader 和恢复方式；确定应用链接起址与向量表，不假定可覆盖所有 Flash。
2. 锁定 CubeMX 与 STM32CubeH7 包版本，以 H743VIH6 新建 IOC；开启 SWD、HSE 与合法电源/时钟组合。查 H743 数据手册、RM0433 和勘误，不套 H723。
3. 先配 UART 和独立硬件时间源；计时器选择待资源复核，不能与电机 TIM1 冲突。明确 HAL/ThreadX 时基、SysTick 和异常入口所有权。
4. 配 SPI2、两路独立 CS/DRDY、EXTI 和 NVIC；验证实际 ODR、中断极性与引脚封装名称。先保留有限超时阻塞读取用于基线。
5. 使用 CubeMX 实际生成代码，保存 IOC、工具版本、选项及生成记录。业务代码独立，不能手工伪造生成文件。
6. 锁定 HAL/CMSIS/ThreadX 来源和端口，建立 ARM toolchain 与真实 MCU CMake 目标；核心源仍来自 `flight/core`。
7. 定义 H743 内存和 DMA 专用段/MPU，核对可达性及 32 字节 Cache 行边界。DMAMUX request、DMA stream、NVIC 优先级在 IOC 验证后确定。
8. 先编译链接并检查 map；再在授权的硬件阶段依次验证启动、时钟、时间、UART、IMU ID/配置/方向/测量时间，最后才考虑电机输出。

不启用虚构 IMU heater；初期 BMI088 单 IMU、异步两路时间。BMI270、SPL06、SD 卡和 USB 功能后置。DShot300 和 RC 资源只做规划，首轮不产生电机输出。

依据：[ST H743 文档](https://www.st.com/en/microcontrollers-microprocessors/stm32h743-753/documentation.html)、[AN4839](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf)、[BMI088](https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi088)。
