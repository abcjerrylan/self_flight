# MicoAir743v2-AIO-35A：P2A/P2B 板级启动与采样

P2A/P2B 软件与 Debug/Release 构建已完成。USB 虚拟串口输出 BMI088 原始样本和每秒诊断，USART1 留给其他用途；ID、两路采样速率和20秒连续记录已实测通过。加计/陀螺三轴方向也已基本实测通过，详情见验证记录。

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

保留 CubeMX 生成的 MPU 基线，当前不启用 I/D Cache，也不使用 DMA。P2B 使用短 SPI 阻塞传输，不需要 DMA/Cache 配置。

引脚依据：[官方 AIO-35A 手册](https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)。实物版号尚未确认。

BMI088 已接入：SPI2 PD3/PC2_C/PC3_C，6MHz mode3；PD4/PD5 accel/gyro CS，PC14/PC15 DRDY。PC2_C/PC3_C 数字开关显式闭合，PA15 的 BMI270 CS 保持高。配置见 [P2B说明](../../docs/p2b-imu.md)。RC USART6 PC6/PC7、SPL06 I²C2 PB10/PB11 仍只保留资源，未实现。

## 启动代码

CubeMX `main.c` 完成 MPU、HAL、公共外设时钟、GPIO、SPI2、TIM2 和 USB PCD 初始化，然后进入 ThreadX。`App_ThreadX_Init` 的 USER CODE 块调用 `app_start()`，先启动 TIM2，再创建采样线程（优先级5）和 USB 日志线程（优先级15），两者静态栈均4096字节。

DRDY ISR 只记录事件时间和序号；采样线程独占SPI2，读取原始值和可用时间；USB线程从256条固定队列取日志、合并发送，每秒追加诊断。采样线程不等待USB发送。

USBX 使用静态 32 KB 应用池，其中分配 16 KB 系统堆和启动线程栈；启动线程设置 RX/TX FIFO（128/64/64/16 words）、绑定 STM32 DCD 并启动 PCD。USBX 与 ThreadX 均配置 1000 Hz。只在 CDC 已激活且电脑设置 DTR 时同步发送，最多等待 100 ms；未连接或未打开串口时直接跳过，不等待电脑。传输异常后中止当前发送，重插 USB 恢复日志。发送只在低优先级USB线程中进行，不在 ISR 或采样线程中调用；未发送的日志计入logdrop。

`platform::time_us()` 比较本次与上次 TIM2 计数，回绕时给高 32 位加一。P2B 使用短 PRIMASK 临界区保护高低位扩展，恢复进入前的屏蔽状态，可由线程和ISR调用；不能超过一次回绕周期（约71.6分钟）不读取。实机回绕仍待验证。

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
3. 关闭 CubeProgrammer，串口终端打开新 COMx，选择 115200 / 8N1，**启用 DTR**。这里的波特率只是虚拟串口参数，没有 UART 引脚传输。P2B 预期逐样本CSV，并每秒追加两行诊断：

   ```text
   IMU,G,10,1000123,1000133,1000153,0,1,-2,0,1
   # BMI088 init=1 err=0 aid=1e gid=0f src=0 reg=02 expect=00 got=00 cpu=400000000 tick=1000
   # STATS a=800 g=1000 missed=0/0 spierr=0/0 overlap=0/0 stale=0/0 latmax=50/50 readmax=30/30 logdrop=0 wait=0 valid=1/1
   ```

   示例只表达格式，不是实测值。打开较晚时 `seq` 已递增，不保证从 0 开始。如果仍显示 STM32 BOOTLOADER，则没有进入应用模式；若有 COM 但没日志，先检查 DTR，发生发送超时后重插 USB。

4. 检查正常枚举、打开/关闭串口、USB 重插和每秒输出。验证不打开串口时程序继续运行，暂停读取时发送最多等待 100 ms。检查 tick 与秒时间持续增加；比较数十秒时间增量与独立参考时间，并连续运行至少 75 分钟观察 TIM2 回绕后时间不倒退。有调试器后可核对 RCC、TIM2 PSC/ARR 和 SysTick reload。
5. 验证复位能重新启动、四路电机脚没有脉冲，USART1 对应外设不受日志占用。记录测试条件和实际误差后，才把 P2A 实机验证标记通过。

P2B 已实现并实测 BMI088 的 ID、配置、DRDY 与原始数据时间戳；详细采集、回放和方向验收见 [P2B说明](../../docs/p2b-imu.md)。当前没有电机协议输出。
