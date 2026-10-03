# MicoAir743v2-AIO-35A：P2A 板级启动

P2A 的软件生成与 Debug/Release 编译链接已完成。日志已改为板上 USB 虚拟串口，USART1 留给其他用途；COM51 枚举、每秒日志和 DTR 控制已实测通过，其余实机项目待验证。

## 当前配置

| 项目 | 配置 |
| --- | --- |
| MCU | STM32H743VIH6，TFBGA100；CubeMX 型号 STM32H743VIHx |
| 时钟 | HSE 8 MHz；PLL M=4/N=400/P=2；CPU 400 MHz、HCLK 200 MHz、APB 100 MHz |
| 电源 | LDO / VOS1，FLASH_LATENCY_2 |
| 日志 | USB OTG FS / CDC ACM，PA11 DM / PA12 DP，USBX 6.4.0，无 DMA；USART1/PA9/PA10 未初始化 |
| USB 时钟 | PLL3 M=4/N=192/Q=8，48 MHz；IRQ 优先级 6，关闭 VBUS sensing |
| 微秒时间 | TIM2 32 位、200 MHz / (199+1) = 1 MHz、ARR=0xffffffff |
| HAL 时基 | TIM6，1 ms；TIM6_DAC_IRQHandler 调用 HAL_TIM_IRQHandler |
| ThreadX 时基 | SysTick，1000 Hz；ThreadX Cortex-M7 GNU 端口拥有 SysTick/PendSV |
| SWD | PA13 / PA14 |
| 电机脚 | PE14 / PE13 / PE11 / PE9：普通 GPIO、低电平、下拉；未启用 TIM1 |
| Flash / 主 RAM | 0x08000000 / 2 MB；AXI SRAM 0x24000000 / 512 KB；初始 MSP 0x24080000 |

保留 CubeMX 生成的 MPU 基线，当前不启用 I/D Cache，也不使用 DMA。DMA 可达性、缓冲区和 Cache 配置随 P2B 实际采样方案接入。

引脚依据：[官方 AIO-35A 手册](https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)。实物版号尚未确认。

后续资源保留：BMI088 SPI2 PD3/PC2/PC3，两路 CS PD5/PD4、DRDY PC15/PC14；RC USART6 PC6/PC7；SPL06 I²C2 PB10/PB11。这些外围尚未生成或实现。

## 启动代码

CubeMX `main.c` 完成 MPU、HAL、时钟、GPIO、TIM2 和 USB PCD 初始化，然后进入 ThreadX。`App_ThreadX_Init` 的 USER CODE 块调用 `app_start()`，创建一个 4096 字节静态栈的线程，优先级 10。

线程启动 TIM2，每秒输出设备名、`seq`、CPU 配置频率、自 TIM2 启动以来的秒/微秒、RTOS tick 与 `dt_us`。USB 发送后再 sleep，因此相邻日志的 `dt_us` 包含发送时间，不是精确周期任务。

USBX 使用静态 32 KB 应用池，其中分配 16 KB 系统堆和启动线程栈；启动线程设置 RX/TX FIFO（128/64/64/16 words）、绑定 STM32 DCD 并启动 PCD。USBX 与 ThreadX 均配置 1000 Hz。只在 CDC 已激活且电脑设置 DTR 时同步发送，最多等待 100 ms；未连接或未打开串口时直接跳过，不等待电脑。传输异常后中止当前发送，重插 USB 恢复日志。当前只有低频启动线程发送，不在 ISR 或快速控制线程中调用。

`platform::time_us()` 比较本次与上次 TIM2 计数，回绕时给高 32 位加一。当前只有启动线程调用，每秒读取一次；不能在多个线程/ISR 中并发调用，也不能超过一次回绕周期（约 71.6 分钟）不读取。P2B 的中断采样时间戳需要调整这个所有权。

## 构建与重新生成

正常构建见仓库根 README；不会访问安装目录中的厂商源。

重新生成要求已安装 **CubeMX 6.15.0、STM32CubeH7 V1.12.1、X-CUBE-AZRTOS-H7 3.4.0**，并按厂商流程接受许可。在仓库根目录执行：

```powershell
./tools/generate-board.ps1 -CubeMXPath '<CubeMX安装目录>/STM32CubeMX.exe'
```

固件包放在自定义位置时，添加 `-FirmwareRoot '<STM32Cube_FW_H7_V1.12.1目录>'`；Java 不在 CubeMX 的 `jre/bin` 时添加 `-JavaPath`。安装/下载由开发者显式完成，构建不自动升级依赖。

脚本加载已提交的 IOC，选择 STM32CubeIDE/GCC 生成，再整理输出；已在当前项目实跑成功。生成日志保存在 `build/p2a/generate-board.log`。

CubeMX 6.15 的 CLI CMake 模板会生成绝对路径，并在 Windows 下把 sysmem/syscalls 的输出路径拼坏。STM32CubeIDE 模板也可能把 startup/sysmem/syscalls 放入重复路径。项目因此采用独立 CMake，并把这三个**原样生成的文件**整理到 Core；未改写厂商初始化代码。自定义接入只放在 USER CODE 块。ThreadX 的 `__Vectors` 通过链接别名指向生成启动文件的 `g_pfnVectors`，没有改造向量表。

整理脚本移除早期生成尝试的重复 Src/Inc/CMake 和 IDE 路径目录，并核验删除位置位于该 generated 目录中。共享 IOC 不保留本机固件安装路径，锁定 FirmwarePackage 且不选择 latest。ThreadX/USBX 构建源列表固定在 `threadx-sources.cmake` 和 `usbx-sources.cmake`，构建时不扫描源码。USBX 使用 OBJECT 库，保证其 HAL PCD 强回调进入固件。

## 接板后的 P2A 验证

当前固件是 **0x08000000 独立启动布局**。先确认实物版号及现有 bootloader 是否保留；若保留，需要先调整链接地址和向量表布局。当前布局写入 Flash 开头会覆盖那里已有的应用 bootloader，芯片 ROM DFU 不受影响。

1. 无需焊 SWD。按住 BOOT0 按钮再插入 USB 数据线，按[官方 USB 下载教程](https://micoair.cn/zh/docs/flight-controller/flight-controller-firmware-tutorial-ardupilot-px4-betaflight-inav)进入 DFU。CubeProgrammer 选择 USB 连接，下载本次生成的 `build/mcu-debug/self_flight.hex`，并校验。HEX 自带目标地址，不给它另填应用偏移。
2. 下载完成后断开 USB，松开 BOOT0，正常重插，让当前应用启动。设备管理器的“端口（COM 和 LPT）”应出现 USB Serial Device (COMx)。Windows 的 CDC ACM 使用[系统 Usbser 驱动](https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-driver-installation-based-on-compatible-ids)。DFU 下载和应用虚拟串口是同一 USB 接口的不同启动模式。
3. 关闭 CubeProgrammer，串口终端打开新 COMx，选择 115200 / 8N1，**启用 DTR**。这里的波特率只是虚拟串口参数，没有 UART 引脚传输。预期每秒一行：

   ```text
   self_flight USB seq=10 cpu=400000000 t=11.000123s tick=11000 dt_us=1000010
   ```

   示例只表达格式，不是实测值。打开较晚时 `seq` 已递增，不保证从 0 开始。如果仍显示 STM32 BOOTLOADER，则没有进入应用模式；若有 COM 但没日志，先检查 DTR，发生发送超时后重插 USB。

4. 检查正常枚举、打开/关闭串口、USB 重插和每秒输出。验证不打开串口时程序继续运行，暂停读取时发送最多等待 100 ms。检查 tick 与秒时间持续增加；比较数十秒时间增量与独立参考时间，并连续运行至少 75 分钟观察 TIM2 回绕后时间不倒退。有调试器后可核对 RCC、TIM2 PSC/ARR 和 SysTick reload。
5. 验证复位能重新启动、四路电机脚没有脉冲，USART1 对应外设不受日志占用。记录测试条件和实际误差后，才把 P2A 实机验证标记通过。

完成后进入 P2B：BMI088 的 ID、配置、DRDY 与带时间戳的原始数据。当前没有 IMU 驱动和电机协议输出。
