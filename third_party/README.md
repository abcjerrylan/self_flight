# 依赖来源与锁定

P1 核心没有第三方库。P2A 从本机已安装的官方包通过 CubeMX 生成并复制 HAL/CMSIS/ThreadX/USBX，未复制 PNX 的依赖，也没有自动下载或升级。

锁定 **CubeMX 6.15.0、STM32CubeH7 V1.12.1、X-CUBE-AZRTOS-H7 3.4.0 / ThreadX、USBX 6.4.0、ARM GCC 13.3.1**。版本、官方来源与两份官方归档 SHA-256 见 [dependencies.json](dependencies.json)。HAL 为 1.11.5，CMSIS Device 为 1.10.6；Core 头文件的版本宏为 5.3。

厂商源码位于 `boards/micoair743v2_aio35/generated/micoair743v2_aio35/Drivers` 和 `Middlewares`，共 417 个文件，与所选官方包逐项字节一致，摘要见 [vendor-fingerprints.json](vendor-fingerprints.json)。没有修改厂商源码。独立检出可直接构建 MCU，正常构建不需要固件安装目录或网络。

需要重新生成时，在 CubeMX 中安装指定版本的 STM32CubeH7 和 X-CUBE-AZRTOS-H7 包，依其安装流程接受许可，核对归档摘要，然后使用 [板级生成脚本](../boards/micoair743v2_aio35/README.md)。不使用浮动 latest/master；没有需要构建时自动获取的源码。

保留各组件实际 LICENSE、ThreadX/USBX LICENSED-HARDWARE，以及 `licenses/` 中两份完整包的条款。不同组件许可不同；此旧 Azure RTOS 包中的 ThreadX/USBX 使用 Microsoft Azure RTOS 条款，不能按当前其他发行版统一写成 MIT。源码原有版权声明均保留。

P2B 新增的 5 个 SPI HAL 文件同样逐字节匹配官方包。BMI088 驱动根据 Bosch 数据手册原创实现，没有引入或复制其驱动库。仅新增原创驱动/平台/启动/构建接入。整体项目原创部分的开源许可尚未选择，本轮没有提交、推送或发布这些修改。

P4A接入UART6 SBUS，CubeMX复制的6个UART HAL/LL文件逐字节匹配锁定H7 V1.12.1归档；原417个文件摘要检查通过，更新后共417个。SBUS和DShot协议代码原创实现，核对外部协议资料未引入PX4/Betaflight源码或依赖。
