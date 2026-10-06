# P3+ 与 P4A：共享反馈及软件输出链

本轮完成软件实施、主机回归、CubeMX生成和 H743 编译链接。用户随后确认微空TRS接收机的SBUS输出已接UART6 RX，已接入完整解析及收帧服务；用户烧入新版后，桌面静止反馈/时序及SBUS持续接收已通过。实际通道映射与失联实测待完成；真实电机定时器/DMA尚未实现。

## 当前运行链

```text
BMI088 → 已有标定/FRD变换 → RateFeedbackFilter（一次80Hz gyro低通）
                              ├→ Mahony（内部gyro低通旁路）→ q_nb、total_yaw
                              └→ 实际dt差分 → 30Hz D低通 → RateFeedback
         → FlightGuard → DShot四路软件批次 → Snapshot/USB日志
```

采样线程优先级5、快速姿态线程优先级6、SBUS解析线程优先级9、日志线程优先级15。UART中断优先级7，单字节接收只入固定128条队列；解析不在中断/姿态线程进行。没有新增快速线程、HAL电机输出调用、动态内存或第二份IMU采样所有权。HTML保持当前 Roll/Pitch/total_yaw 显示，板卡模型仍读取四元数。

## P3+ 实现

- `rate_feedback.hpp/.cpp`：一阶低通抽成共享模块；RateFeedback含滤波gyro、去残余偏置的角速度、角加速度及单独的derivative_valid。
- `imu_pipeline.hpp/.cpp`：固件与native回放共用相同组合核心。gyro只低通一次；Mahony仍提供独立算法入口供回归。正常连续输入与原Mahony四元数在3000帧主机对照中逐分量差小于1e-6。
- D差分对象为标定、安装旋转、低通后的gyro，**不差分在线残余偏置**。完成同帧Mahony后，rate的偏置与姿态发布保持一致；人为重力反馈不进入机体角速度或D。
- 使用真实测量dt。第一帧初始化gyro，并将D低通状态初始化为0；derivative_valid=false。第二个连续帧才开始有效差分。重复/倒退拒绝；正向过大间隔或序号缺口拒绝并重置历史，下一帧重新初始化，不对未知时间求导或传播。
- 拒绝的独立滤波调用不修改out；数值运算成功后才提交滤波状态。组合状态拒绝时明确invalid。加计可信度、启动标定、参考epoch及相对yaw语义保留。
- runtime/latency现在覆盖共享反馈、Mahony、状态机和软件帧准备，截止时间仍为测量到处理完成1000us。达到截止时间使反馈invalid并调用deadline_missed；若已Armed，立即锁存Fault并替换为全STOP的软件批次。该统计截止到迟到故障处理及快速服务结果快照复制之前，包含任务抢占/等待，不宣称纯函数CPU时间。

80Hz/30Hz是起始软件参数，未经过带电机振动或飞行调参。合成100Hz正弦在30Hz D低通后，微分RMS为旁路的0.269531倍；这只是主机滤波响应检查，不是实机噪声或闭环验收。

## P4A 状态机

状态为 Boot → Calibrating → Disarmed → Armed，解锁后故障进入Fault。

解锁必须同时满足：标定接受、有效且新鲜的RC（默认100ms）、同一帧有效姿态/角速度及D反馈（默认5ms）、控制器和硬件驱动ready、油门≤5%、没有急停，以及先观察到有效arm-OFF再观察到ON边沿。启动/标定中看到ON、首次接收RC就是ON、被拒绝后仅保持ON，都不会自动解锁。

Armed后油门可以增加；5%仅用于解锁/故障恢复。RC失联/failsafe、反馈失效、参考epoch变化、急停、校准丢失、控制器/驱动不可用或状态机更新间隔异常会禁止输出并锁存故障。显式关闭解锁开关、低油门、急停解除且健康恢复后才进入Disarmed，随后需要新的ON边沿。

`reset_sequence()`在进入/退出Armed时递增，供P5控制器清积分和重新建立目标；当前没有PID积分可清。Fault不会自动重新解锁。正向时间缺口拒绝本周期但重建时间基线，避免故障恢复永久卡死；重复/倒退不推进时间。

`GuardReason`位定义在flight_guard.hpp：0标定、1RC、2姿态、3rate/D、4控制器、5驱动、6解锁油门、7急停、8参考改变、9更新时序、10配置、11处理截止时间。`arm_block_reasons`显示本周期检查，`fault_reasons`锁存首次故障原因；飞行中的油门位不会撤销输出许可。

## TRS / UART6 SBUS

根据用户实际接线和[本板官方说明](https://micoair.com/flightcontroller_micoair743v2_aio_35a/)，直接RX6是PC7；专用SBUS焊盘另有硬件反相。当前实际IOC配置为USART6仅接收、100000波特率、8位数据+偶校验+2停止位，MCU RX反相开启。HAL的9B配置包含1位偶校验，因此有效数据是8位。若改接专用SBUS焊盘，需要关闭MCU反相并重新生成；不能连续反相两次。

[TRS官方资料](https://micoair.cn/zh/docs/telemetry/trs/trs)列出50/100/150Hz链路模式；57600/115200是其独立数传UART参数，不用于设置SBUS波特率。本轮只接SBUS，不实现TRS数传、回传或调参。

原创`drivers/sbus`解码25字节帧、16路11bit模拟通道、17/18数字位、frame_lost和failsafe；帧头/尾及标志位语义与[PX4 v1.17.0](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/lib/rc/sbus.cpp)核对。解析接受常见0/04/14/24/34尾字节；字节时间倒退拒绝，部分帧遇到>3ms字节间隔重置，可覆盖TRS150Hz模式的帧间隔。UART校验/帧/溢出错误或队列丢字节会清除部分帧并重新对齐。SBUS没有CRC，帧结构检查不能保证发现任意数据位损坏。

frame_lost单独置位时不刷新上一个有效PilotCommand，但也不直接判定failsafe；有效指令超过100ms会因失联禁止输出。failsafe置位会立即发布receiver_failsafe=true，包括同时frame_lost的情况。原始RC快照的fresh只表示收到帧的时间，不能替代PilotCommand有效性。

`boards/micoair743v2_aio35/receiver-profile.hpp`保存零基通道及实测端点，启动时尝试接入；必要通道未分配时map=0，仅显示原始值。可选模式与急停可留255，未分配模式默认为Rate、急停false。映射拒绝重复/非法索引、无序端点；三轴分段归一化及反向，油门映射[0,1]。不猜测解锁开关。TRS的CH12用于RSSI（见官方手册），不配置为飞行控制开关。

用户逐项配合的结果：CH1右杆左右、CH2右杆上下、CH3左杆上下油门、CH4左杆左右偏航，即AETR。CH1/2端点173/1811、中点992；CH3端点173/1811；CH4左端173、中点992、这次右端最大1807。采用统一SBUS标度173/992/1811，当前记录中偏航右满约+0.995；不把手动采集称为精密端点标定。FRD方向设置为右横滚/右偏航正，右杆后拉（向下）抬头正，因此pitch反向。解锁开关辅助通道尚未观察到，保留未分配并禁止发布PilotCommand；只保存已验证四轴。结果见[receiver-micoair-trs-20261007.json](receiver-micoair-trs-20261007.json)。

## 四路软件 DShot

原创encode仅允许STOP=0及48..2047油门，拒绝1..47命令区；16bit包含11bit值、telemetry请求bit和三段nibble异或校验。正常单向校验与[Betaflight 4.5.2协议编码实现](https://github.com/betaflight/betaflight/blob/4.5.2/src/main/drivers/dshot.c#L104)核对；没有复制其模块或添加依赖。遥测请求bit不等于双向DShot，当前不实现RPM回传。

`prepare`把四路作为一批准备。油门0输出STOP，正值映射48+round(v*1999)，不实现armed idle。需要ActuatorCommand和FlightStatus两处许可、Armed状态、有效有序且新鲜的元数据（默认3ms）和四路有限的[0,1]值。任一拒绝都会返回**四路全STOP**；不会只保留其他三路，也不把非法输出钳成可运行值。

`flight/services/flight.cpp`由原快速任务调用，提供publish_pilot给SBUS服务，保存状态/帧/计数。固件中controller_ready=false、output_driver_ready=false，尚未映射时PilotCommand保持无效且failsafe。因而真实固件无法解锁，软件帧始终全0；不包含任何电机定时器、DMA或GPIO操作。

准备软件停止帧不等于电调已经收到停止命令。P4B还必须实现独立输出时效检查、快速任务停更时的硬件停止策略、四通道统一触发、H7 DMA/cache和波形验收。

## 日志与回放

- 原`# ATT`字段保持，新增独立约50Hz `# RATE`：rate_u为rad/s×1e6，accel_m为rad/s²×1000，另有valid、d_valid、rate error。
- 每秒`# FLIGHT`：状态0..4、allowed、block/fault mask、控制重置序号、RC更新计数、dry=1、批次/停止计数、四个16bit帧与stop_reason。stop_reason位0为许可，1为过期/时序，2为无效/范围。当前四帧应为0000。
- 每秒`# RC`：帧序号、年龄/fresh、丢帧/failsafe标志、映射状态、字节/错误/队列丢失/坏帧/接收重启计数以及按CH1至CH16顺序的原始值。
- 每秒`# PILOT`输出归一化三轴/油门（乘1000）、解锁/模式/急停/failsafe和命令fresh；该诊断加入于四轴检查后的构建，初轮板上记录中尚无此行。
- native CSV新增wx/wy/wz（rad/s）、ax/ay/az（rad/s²）、rate_valid/d_valid。原Roll/Pitch/total_yaw及段号保持。
- 仍为最新帧事件交接；没有声称软件队列能恢复BMI寄存器覆盖的样本。增加负载后的丢样/延迟需在板上重新测量。

## 验证与待验收

主机Debug/Release各59/59：原37项加6项rate、8项guard、3项DShot和5项SBUS。覆盖不等间隔、零偏不产生D、滤波响应、数值失败事务、缺样/回绕、与原Mahony对照；启动高开关、带油门解锁、失联/failsafe/急停、过期/失效/参考改变、故障恢复、处理超期和四路假输出；完整允许编码范围与两种telemetry位、全部16种单比特错误、映射及许可/范围门控；16路打包/解码、标志/尾字节、坏帧/部分帧/150Hz恢复、时间与序号、通道归一化，以及50Hz RC与1kHz状态机的丢帧策略。Release测试不会因NDEBUG而关闭。

Python姿态集成各6/6、原始回放4/4、HTML20/20通过。旧静止与六面记录由新native组合核心回放，结果保存在build/p3p4/replay-report.json及对应CSV；这是历史实机输入上的主机执行，不是本轮板上测量。

CubeMX实际重跑并生成USART6/IRQ/HAL UART模块；原400MHz CPU、SPI/USB及电机GPIO配置保留。新增6个UART厂商文件逐字节匹配锁定官方H7 V1.12.1归档，旧411个文件摘要检查通过，共417个；没有修改厂商代码。

初轮H743 Debug/Release编译链接成功。Debug Flash134508B/RAM_D1 74480B，Release Flash84824B/RAM_D1 74448B。HEX SHA-256分别为`d079648d25f44abe6152a1da21ea1bf4ddf4886afa5d74a1981bb8e5889c9b29`、`909f629d796fb59d7a07a1b93f05bcda8c220049911ec74a7b85101fde7f21a3`；以下实机测量对应用户自行USB DFU烧入的初轮Debug版本。本工具未自动烧录或产生电机输出。

2026-10-07，COM51连续两轮20秒采集：首轮接收机在后段开始输出SBUS，ATT与RATE/D均1001/1001有效。随后稳定连接轮同样1001/1001有效，姿态更新998.914Hz；窗口抽样run最大251us、lat最大337us，板上累计最大run370us/lat509us。累计统计包含启动过程，不能把它们当作纯函数耗时。窗口AHRS reject/timing/missed/timeout/late、IMU missed/spierr/overlap/stale/logdrop/wait及PIPE late均无增加。启动静止标定Accepted；水平静止20秒Roll/Pitch标准差0.0175°/0.0132°，相对yaw变化+0.152°；这不是飞行或振动验收。

稳定连接轮20条RC诊断均fresh=1、frame_lost=0/failsafe=0，首末诊断间新增2706帧/67650字节，接收错误/队列丢失/坏帧/重启计数无增加。上电已有一次接收错误，不将其描述为全程零错误。所有FLIGHT诊断均Disarmed、allowed=0、四路0000，所有软件批次都是STOP；map=0/rc_updates=0和block=50对应未映射RC及尚未完成的控制器/驱动。原始数据与汇总在build/p3p4/board-static.csv、board-linked.csv及各自-report.json；COM51已释放。

四轴检查后补入已测通道/方向、PILOT诊断和可结束的连续采集。当前最终构建Debug Flash134940B/RAM_D1 74528B，Release Flash85136B/RAM_D1 74496B；HEX SHA-256分别为`bea31c8ffd44dfab28de094506cb185360e365d94cdee60f15b2c3547e2eeb25`、`2fc603a8df046dae5baeab75d11200c424d493aa323518ac21531c76284af1e4`，同步在build/p3p4/replay-report.json。该补充构建尚未烧录；通道配置仍因未分配arm而map=0，不要求立即重复烧录，待解锁开关明确后一次更新。连续controls采集已经结束、串口已释放。

下一步据实测填写接收机映射并检查真实失联；P4B开始前确认电气M1～M4的机臂/转向，并准备波形测量。角速度PID、混控和姿态外环属于P5，尚未实现。
