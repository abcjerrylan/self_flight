# 进度

当前更新：2026-10-07。以下保留各阶段历史结果。

## P0：已完成

- 检查目标目录原先不存在，检查目标祖先工作规则，读取参考工程分层、启动与 BSP/IMU 相关资料。
- 建立独立 Git 仓库；P0 当时未提交/推送。当前在用户已有的 `main` / `origin/main` 上继续工作。
- 完成 README、项目 AGENTS.md、独立 CMake/presets、忽略规则、来源与依赖说明。
- 完成参考审查，记录已有用户修改和文档/源码差异；结束状态与 12 个抽查文件摘要一致。
- 没有复制 PNX 模块、子模块、旧 build 或依赖，没有在参考工程构建。

## P1：已完成

- 实现向量加减/标量/dot/cross、Hamilton 四元数乘法/共轭、稳健归一化、轴角构造、向量旋转与有限值检查。
- 实现 64 位微秒时间契约、显式间隔错误与样本新鲜度检查；时间完全由调用者传入。
- 定义 IMU、标定、姿态、飞手指令、控制目标、四路输出和飞行状态数据对象；默认 invalid/禁止输出。
- 明确 FRD/NED、WXYZ、body→navigation、右侧旋转先作用、SI 单位、两路采样时间与失败不修改输出。
- host Debug 和 Release configure/build/CTest 均实际通过，分别 14/14；正常构建没有 ARM/HAL/ThreadX/PNX 依赖。
- MCU 未完成与 host 工具链拒绝门控验证通过；两份核心实现通过 Cortex-M7 对象编译检查。
- H743 板级资源与真实生成流程已有文档；基础 MCU 依赖将在 P2 验证后锁定，没有虚构版本或生成工程。

完整命令、工具版本、结果和限制见 [validation.md](validation.md)。

## P2A：软件完成，实机待验证

- CubeMX 6.15.0 实际生成 STM32H743VIH6 / TFBGA100 工程；保存真实 IOC、H743 启动/链接文件、HAL/CMSIS 与 ThreadX。
- 锁定 STM32CubeH7 V1.12.1、X-CUBE-AZRTOS-H7 3.4.0 / ThreadX、USBX 6.4.0；保留许可和包 SHA-256。406 个引入的厂商文件与官方包逐字节匹配。
- 实现 CPU 400 MHz、TIM2 1 MHz 微秒时间、HAL TIM6 / ThreadX SysTick 各自时基与一个静态启动线程。
- 根据 USART1 已被占用的反馈，日志改为 USB OTG FS / CDC ACM（PA11/PA12）；USART1/PA9/PA10 未初始化。保存实际重跑的 CubeMX 配置、USBX 静态内存和 USER CODE 接入，Debug/Release 再次编译通过。
- 四路电机脚配置为低电平 GPIO；没有 PWM/DShot。Flash 0x08000000 为明确的独立启动构建配置，bootloader 布局尚待确认。
- MCU Debug/Release 完整编译链接通过，产出 ELF/HEX/BIN/map；脚本重新生成后再构建成功。
- 向量表、MSP、ThreadX 中断入口与硬浮点属性核对通过。host Debug/Release 回归各 14/14 通过。
- 在原有 `main` 分支工作，未提交或推送本轮修改。参考工程 12 个文件摘要仍一致。
- 最初只验证软件；本轮用户自行烧录后，USB 枚举和连续日志已实测通过（见末尾）。计时误差和回绕尚未实测。

## P2B：已完成，含标定与基本实机验收

- 原创 BMI088 驱动完成 SPI dummy byte、芯片ID、复位、配置读回、加计错误寄存器、带符号原始值和 SI 转换。
- 实际重跑 CubeMX 接入 SPI2 / 两路 CS / DRDY，PC2_C/PC3_C 数字开关闭合。SPI6MHz mode3，USB仍48MHz，电机GPIO保持低。
- DRDY ISR记录独立测量时间和序号；采样线程独占SPI，记录读取开始/可用时间、丢样/错误/重叠/过期。固定256条日志队列隔离USB延迟。
- USB逐样本CSV、每秒诊断、Windows采集和Python回放已完成；host Debug/Release 各18/18通过，回放边界测试3/3通过。MCU两种构建通过，411个厂商文件与锁定官方包相同。
- 用户烧入新版后，在COM51静止采集20秒，收到36,070条有效样本：加计803.49Hz、陀螺998.87Hz；序号缺口、无效、SPI错误、采样丢失、重叠和过期均为0。采集窗口logdrop没有增加；最大延迟73/66us。
- 比力模长9.796m/s²，静止预期body均值约(-0.221,-0.021,-9.792)m/s²。这只是原始数据观测，没有做标定。
- 原始CSV、SI回放和统计在build/p2b；用户逐轴摆放/转动后，加计和陀螺的三轴主轴/符号均与机头箭头FRD一致；手持检查没有作为精密角度/标定验收。结果见validation。
- 始终使用main，未提交或推送；用户已有.clangd保留，PNX只读。

## 下一步

P3算法与桌面实机方向验收已完成，详见[p3-attitude.md](p3-attitude.md)。P3+与P4A的软件实施/主机验证已完成，见[p3plus-p4a.md](p3plus-p4a.md)。用户已确认微空TRS的SBUS接在UART6 RX；解析及真实UART配置已完成，用户烧录后新处理链桌面时序和16路原始通道接收已实测通过；继续确定映射和真实失联行为，再进入P4B定时器DMA/波形。

P3前补丁已随本轮固件上板：启动静止标定Accepted，TEMP有效、PIPE无新增迟到；末段运动拒绝的专项硬件复测仍待补（主机回归与旧实机回放通过）。P2A独立计时误差、71.6分钟回绕和电机脚电平等项目仍待完成。

尚未实现：PID、混控、真实定时器DMA DShot、持久化和飞行。SBUS已实现但实际通道映射与板上接收验收待完成。P4A软件状态机和DShot编码已有实现/主机测试，固件控制器及硬件驱动ready为false，不能解锁。

## USB 首次接板检查

用户已自行烧录；COM51 已枚举，设备序列号 SELF_FLIGHT_H743，Windows 驱动状态 OK。用户释放串口后，DTR=true 的 12 秒监听收到 11 行日志；DTR=false 的 3 秒为 0 字符，重新开启后的 4 秒收到 4 行。应用启动、每秒日志、串口重开和 DTR 控制已验证；COM51 已释放。没有更改固件、重新烧录或输出电机信号。计时精度、回绕、物理拔插与电机脚等项目仍待验证。


## P3：已完成，含桌面实机验收

- 同一便携C++核心实现一阶RC低通、body→navigation四元数传播/归一化、六轴Mahony、加速度可信度、残余零偏学习与幅值限制。滤波系数使用各自实际测量时间间隔。
- 采样所有权保持独立，gyro发布事件唤醒优先级6姿态线程；两路标定成功才消费输入。USB新增约50Hz ATT和每秒AHRS，解算耗时/延迟/丢样/时间错误可测；没有磁力计，absolute_yaw_valid始终false。
- host Debug/Release各34/34，Python原始回放4/4、姿态集成测试每种构建3/3。旧六面在两种原生构建回放可解释；静止旧/新日志均可通过同一C++核心回放。
- 用户烧入P3 Debug后，20秒静止1001/1001姿态帧有效，更新999.02Hz，横滚/俯仰标准差0.0102°/0.0119°，20秒相对偏航变化-0.119°；累计最大解算137us/延迟256us，采集期间原始采样/解算计数无新增错误、丢样或超期。
- 手动右侧压低：横滚约+36.05°；抬机头：俯仰约+14.76°，各400/400帧有效。手动角度/晃动不能作为精密角度验收；水平顺时针转动采集60秒，3001/3001帧有效，相对yaw增加+92.04°，方向正确；本轮累计最大解算180us、延迟282us，窗口无新增采样/解算错误、丢样或超期。文件与边界见p3-attitude.md。

## USB 实时姿态查看页面（2026-10-04）

- main 上新增单文件 HTML，Web Serial + DTR 直接读取 P3 日志；四元数驱动板卡模型、三轴角度、20 秒曲线和状态诊断，支持明确标记的演示与本地日志回放。
- Node 16 项检查通过；已有四份实机日志 4802 条 ATT 全部解析。浏览器验证演示、实际偏航日志回放、进度拖动、末尾重播和桌面布局，没有页面运行错误。
- 本次没有更新固件或烧录；浏览器 USB 设备选择/连续直连接收尚待桌面 Chrome / Edge 验证。使用见 [attitude-viewer.md](attitude-viewer.md)。

## 首次连续欧拉展开（2026-10-04，历史实现；已由下述机体轴积分替换）

- 核心新增total_roll_rad/total_pitch_rad/total_yaw_rad与累计段编号；最近等价ZYX分支支持正负多圈与纯pitch跨90°。无效帧保留旧值，解算重新对齐开始新段，不跨未知间隙累计。
- USB ATT新增total_rpy_md/total_epoch，HTML数字和曲线使用total，四元数模型保留。旧日志不伪造圈数，只显示模型和更新提示。native回放与两个Python汇总器同步。
- host Debug/Release各36/36、HTML 18/18通过，MCU两种固件编译链接通过；本轮未烧录，实机累计角和新增解算开销待验证。源码仍在main，未提交/推送。

- 补充：Python原生姿态集成每构建4/4、原始回放4/4通过；浏览器主机两圈日志显示total yaw 720.0°，无运行错误。新版固件尚未上板，时延/多圈实测仍待进行。

## 机体轴累计转角修正（2026-10-04）

按用户实际pitch越过90°的复现条件，删除连续ZYX展开，total改为校正body_rate乘实际dt的积分。每段从0开始，不积分重力反馈；保留无效帧、时序拒绝和缺样后新epoch约定。新增total_kind=body同步固件/原生回放/HTML/汇总，旧Euler total只显示q模型并提示更新固件。当前没有控制器使用total；累计值会漂移，不作为姿态反馈。

host Debug/Release各37/37（0.95s/0.86s），Python姿态集成各6/6、原始回放4/4，HTML20/20通过。回归包含±0.5°初始bank后绕body Y正负720°与噪声、实际800/1200us混合三轴积分、反转、静止重力修正不累计、初始化/无效/缺样恢复。两种MCU构建成功：Debug Flash105,644B/RAM_D1 69,464B；Release Flash67,276B/RAM_D1 69,416B。

build/p3/body-pitch-host.log是同一原生核心的合成轨迹，不是实机记录。初始横滚0.5°、Y轴90°/s、8秒，8001有效姿态，末尾total=(0,719.964722,0)°；约0.0353°为浮点累加误差。浏览器401帧回放显示Y累计720.0°、X/Z0.0°且曲线连续，没有页面运行错误；合成run/lat不作为硬件测量。Debug HEX SHA-256：2a937b48c3b832a8738b1b7116e4e4dfd523c6a78af0b4d600480f6e5c38b05e。

本轮未烧录、未打开串口、未输出电机信号；新版实机累计角/漂移/耗时待用户烧录后检查。main分支保留，未提交或推送；硬件配置与既有采集未变。

## 恢复当前姿态，单独保留连续yaw（2026-10-06）

用户明确需要姿态反馈，因此删除total_roll_rad/total_pitch_rad和三轴body积分。保持原来的四元数Mahony传播/滤波/重力修正；Roll/Pitch通过attitude_euler(state.q_nb,out)取得当前ZYX姿态，倾斜启动直接反映摆放倾角。

total_yaw_rad每个成功帧从同一q_nb提取yaw，执行total_yaw += remainder(yaw-previous_yaw,2*pi)，跨179°→-179°时继续181°，反转减小。首帧以当前yaw初始化；失效帧不提交新值，缺样后重新对齐重设参考并令total_epoch加一。它与Euler航向一致，倾斜时不等于body Z角速度积分；展开只有同参考段且相邻姿态可跟踪时有效。pitch±90°仍是yaw/roll欧拉奇异点，不承诺翻滚时连续独立航向。

USB保留rpy_md并改为独立total_yaw_md；移除total_rpy_md/total_kind。HTML数字/曲线使用(rpy.roll,rpy.pitch,total_yaw)，q模型保留；旧版日志恢复普通三轴Euler显示，明确提示Yaw未展开，不拿旧body累计量当姿态。native CSV只保留total_yaw_deg与total_epoch，两个汇总工具同步。

常规小倾角姿态反馈可使用Roll/Pitch；普通定向yaw误差按最短角差计算，明确多圈目标则应与total_yaw属于同一参考段。使用反馈前需检查有效性/新鲜度及段号，重新对齐不能当作连续航向。六轴yaw没有绝对航向，会漂移；跨竖直或翻滚控制使用四元数姿态误差，角速度环使用body_rate_rad_s。当前尚无PID/混控控制器，本轮没有接入控制输出。可参考 [PX4控制框图](https://docs.px4.io/main/en/flight_stack/controller_diagrams#multicopter-attitude-controller)。

验证：host Debug/Release各37/37（1.25s/1.02s）；Python姿态集成各6/6、原始回放4/4；HTML20/20。覆盖yaw正负4圈、±180°展开、反转、实际800/1200us时间、无效/重复/倒退保持、缺样恢复；倾斜body Z旋转时total_yaw与四元数Euler航向一致且与body积分不同；Roll/Pitch恢复当前值、旧字段忽略与旧日志回退。两种MCU编译链接通过：Debug Flash105,988B/RAM_D1 69,456B，Release Flash67,676B/RAM_D1 69,408B。

同一native核心生成build/p3/yaw-feedback-host.csv/.log/-report.json：保持roll12°/pitch-8°并绕导航Z转720°，2001/2001有效，末帧roll=12.000005°, pitch=-7.999994°, total_yaw=719.988098°。该记录为主机合成，不是板上测量；日志run/lat是占位值。Debug HEX SHA-256：99aef09f5049bb45ea9dc129c531af9d0ca3c48840851d6b437f8ec084717a4b。

新版尚未烧录，新增atan2/展开耗时和实机姿态输出待测；未打开串口或输出电机信号。仍使用main，未提交/推送。


## PX4源码解析与控制框架（2026-10-06）

按用户要求在线审阅固定v1.17.0的姿态外环、角速度PID、传感器角速度处理、分配去饱和及DShot高层输出路径；EKF2仅审阅IMU输入/姿态输出接口，不宣称完成全仓库或完整EKF算法审计。本机直接下载失败，源码通过官方在线页面读取。文件/函数范围、对照优化与P3+/P4/P5分期见[px4-code-review.md](px4-code-review.md)。

新增[control-framework.hpp](control-framework.hpp)为docs内原创接口草案，未加入CMake或固件；复用既有数据/数学，声明角速度反馈、四元数外环、角速度内环、混控轴饱和反馈与状态门控。主机MinGW和Cortex-M7 ARM编译器均通过C++17严格警告语法检查；这不代表算法已实现、已链接或已在硬件验收。

建议下一实施包P3+数据处理/重置语义与P4A软件输出/状态机；P4B实际接收机协议、四电机位置/转向与波形测量仍需明确。保持Roll/Pitch/total_yaw页面选择，后续控制内部建议q_nb和机体角速度。本轮只新增两份设计材料并更新进度，生产源码/固件未变，未烧录/串口采集/电机输出，main分支保持，未提交或推送。

## P3+ / P4A实施（2026-10-07，含桌面反馈和SBUS接收验收）

- gyro低通统一为共享RateFeedbackFilter，固件/native回放共用ImuPipeline，Mahony内部gyro低通旁路；新增真实dt角加速度差分和独立30Hz低通。在线残余偏置不进入差分，同帧rate使用新偏置；缺样/首帧D无效，不把旧值作为新反馈。
- FlightGuard实现启动/标定/上锁/解锁/故障状态，先OFF再ON、低油门解锁、有效新鲜RC与反馈、故障锁存和明确恢复；参考改变、控制超期触发停止并通知未来PID清积分。
- DShot encode/prepare原创实现四路软件批次、标准校验、STOP/油门映射；非法/过期/禁止输出时全STOP。真实固件controller_ready和output_driver_ready为false，不能解锁，没有电机输出调用。
- 用户确认TRS SBUS已接UART6 RX后，实际重跑CubeMX配置PC7、100000/8E2、MCU RX反相、IRQ7；接收ISR入固定128条队列，优先级9线程解析16通道。丢帧不刷新有效指令，failsafe立即禁止，部分帧/接收错误恢复已实现。通道配置暂未分配，先以USB原始值确认实际映射。
- 日志增加50Hz RATE和1Hz FLIGHT/RC，保持ATT与HTML当前Roll/Pitch/total_yaw及四元数模型。快速服务run/lat覆盖滤波/姿态/软件状态和帧准备；新链最坏时序尚未在板上测量。
- host Debug/Release最终各59/59；Python姿态集成各6/6、原始回放4/4、HTML20/20通过。旧静止及六面8份实机输入由新核心在两种native构建回放通过，记录在build/p3p4/replay-report.json；不是本轮板上数据。
- H743 Debug Flash134508B/RAM_D1 74480B；Release Flash84824B/RAM_D1 74448B；均编译链接成功，ELF/HEX/BIN/map已生成。Debug HEX SHA-256为d079648d25f44abe6152a1da21ea1bf4ddf4886afa5d74a1981bb8e5889c9b29，Release为909f629d796fb59d7a07a1b93f05bcda8c220049911ec74a7b85101fde7f21a3。
- 新增6个UART厂商文件与锁定H7 V1.12.1归档逐字节一致，旧411个摘要核对通过，总417个；原电机GPIO低电平/下拉、400MHz CPU、USB/SPI和USART1未占用配置保留。仍在main，未提交/推送、未自动烧录、未操作电机。

用户自行烧录后在COM51做两轮20秒采集，ATT与RATE/D各1001/1001有效。稳定连接轮更新998.914Hz，窗口抽样最大处理251us/延迟337us，累计最大370us/509us；采样/姿态/日志没有新增丢样、错误、超期或超时。标定Accepted，所有FLIGHT为Disarmed/allowed=0/四路STOP。SBUS首末诊断间新增2706帧/67650字节，20条RC诊断全部fresh且无lost/failsafe，坏帧/错误/队列丢失无增加；上电已有1次接收错误。原始与汇总在build/p3p4/board-static*、board-linked*，COM51已释放。

继续确认真实通道端点/方向、解锁开关和失联行为；确定映射后填receiver-profile.hpp。P4B真实DShot波形和P5控制器/混控尚未开始。详细实现、理由、验收与限制见[p3plus-p4a.md](p3plus-p4a.md)。

用户进一步逐项检查确认AETR：CH1横滚、CH2俯仰、CH3油门、CH4偏航。CH1/2/3端点173/1811，中点992；CH4这次右端1807。已保存四轴配置和FRD俯仰反向，解锁辅助通道仍未观察到，保持未分配/map=0。数据在build/p3p4/rc-right-horizontal.csv、rc-controls.csv及docs/receiver-micoair-trs-20261007.json；连续采集已结束，COM51已释放。待用户确认遥控器开关与通道配置后继续。

补充构建含四轴配置及PILOT诊断，尚未烧录。当前Debug Flash134940B/RAM_D1 74528B、Release Flash85136B/RAM_D1 74496B；当前HEX摘要分别bea31c8ffd44dfab28de094506cb185360e365d94cdee60f15b2c3547e2eeb25、2fc603a8df046dae5baeab75d11200c424d493aa323518ac21531c76284af1e4。此前两轮板上时序/接收结果对应初轮固件，不作为此补充构建的新板上测量。
