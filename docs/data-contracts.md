# 坐标、数学、时间与数据契约

## 坐标与单位

机体系为 FRD：X 前、Y 右、Z 下。导航系为 NED：X 北、Y 东、Z 下。无绝对航向观测时，首版水平轴只是初始化朝向，不能宣称已经指北。

正转遵循右手定则。内部角度 rad、角速度 rad/s、加计比力 m/s²、距离 m，`dt` 秒。姿态使用 Hamilton WXYZ 四元数：`q_nb` 将机体向量变换到导航系。

```text
v_n = q_nb ⊗ [0,v_b] ⊗ conjugate(q_nb)
q_ac = q_ab ⊗ q_bc             # 右侧变换先作用
```

例如正 90° 绕 Z，把 X 轴向量旋转到 Y 轴；在 FRD 中对应右转方向。X、Z 轴旋转不可交换。`q` 与 `-q` 表示同一旋转，不按四元数分量直接比较姿态相等。

理想水平静止、body 与 navigation 重合时，加计比力约 `(0,0,-g)`。未来导航加速度是 `R_nb*f_b + (0,0,g)`。芯片轴上的偏置/比例先应用，再执行芯片→板→机体完整变换；不能仅对欧拉角加安装角。

## 数学 API

- `Vec3`：加、减、乘标量、dot、cross、有限值检查。
- `Quaternion`：Hamilton 乘法、共轭、有限值检查。
- `try_normalize`：先按最大绝对分量缩放，避免平方范数溢出；范数必须严格大于门限。
- `try_from_axis_angle`：轴必须可归一化，角度必须有限，即使角度为零也拒绝零轴。
- `try_rotate`：局部归一化输入四元数，检查输入及计算结果有限。

checked 操作返回 bool，失败时保持输出原值，也支持输入/输出别名。默认最小范数为 `1e-6`，门限须有限且非负。输出身份四元数不代表估计有效，必须同时检查数据有效性。

基础加减、dot、cross、multiply 属于原始算术，不保证任意大输入不溢出。发布结果前检查有限值；`conjugate` 只有对单位四元数才是旋转逆。`try_rotate` 可以拒绝中间运算溢出的输入，不保证覆盖浮点极限附近的所有向量。

禁止 fast-math；它可能使 NaN/Inf 检查失效。主机浮点测试不能替代目标 FPU 行为验证，尤其是次正规数和 flush-to-zero。未来 MCU 应按真实编译参数验证。

## 时间

`TimestampUs` 为 uint64，单调微秒。零是有效 epoch，不用零表示“无数据”；由 metadata.valid 表达。传感器 DRDY 时间是测量时刻的估计，尚包含内部滤波延迟；available 时间是完整数据可用时间。

`checked_delta(previous,current,maximum_gap)` 返回秒以及 None/Duplicate/Backward/GapExceeded/InvalidLimit。最大间隔为零非法；恰好等于最大间隔合法。失败时 seconds=0，调用者必须检查 valid，不能把零间隔继续交给控制器。

比较顺序先处理倒退，再进行无符号减法。时间超过 32 位范围或接近 64 位上界时仍可计算短间隔。uint64 回绕被视为倒退，不能靠 core 恢复硬件未扩展的回绕。

`is_fresh` 要求 measured≤now 且年龄≤门限，允许同一时刻、零年龄门限。它不检查样本有效性。`is_usable(VectorSample)` 额外检查 valid、有限值、measured≤available≤now。

## 数据对象

| 对象 | 说明 |
| --- | --- |
| SampleMetadata | 测量/可用时间、序号、有效性；不是原子对象 |
| VectorSample / ImuSample | 分别保留陀螺和加计时间；accel_is_new 表达新加计样本 |
| Calibration | 器件/板身份、传感器轴零偏/比例、温度、静止质量和格式版本 |
| AttitudeState | q_nb、机体角速度/零偏、输入时间、绝对航向有效性；连续偏航与参考段编号 |
| PilotCommand | 归一化三轴摇杆、油门、模式、解锁/急停请求、接收机失效 |
| ControlSetpoint | 倾角、角速度和油门目标，mode 决定字段解释 |
| ActuatorCommand | 电气 M1–M4 的归一化输出、饱和、输出许可 |
| FlightStatus | 状态与统计；P4A GuardReason定义故障/禁止解锁bit |

所有 metadata 默认 invalid；PilotCommand 默认 failsafe；FlightStatus 默认 Boot；电机输出为零且许可 false。校准默认 Unknown，比例为单位值不代表已完成标定。

P1 不含状态机、校准计算、消息同步或序列连续性检查。序号未来按 uint32 模运算处理回绕，并明确重启代次，不能简单用无符号大小判定新旧。结构体不直接序列化为日志/Flash：后续定义稳定编码、字节序、版本和校验，避免 padding 和 ABI 差异。


P2B已增加原创标定计算：传感器SI轴上先扣偏置/乘比例，再执行安装旋转。完整quality仅在加计参数通过且启动gyro静止窗口通过后为Accepted；gyro_calibrated/accel_calibrated区分两路状态。原始metadata.valid与标定质量独立。身份tag仅代表固定板型/器件配置，不能唯一识别物理单板；Flash参数服务尚未实现。详见imu-calibration.md。

2026-10-06恢复姿态反馈语义：删除total_roll_rad/total_pitch_rad，只保留total_yaw_rad。Roll/Pitch由attitude_euler(q_nb,out)取得当前ZYX姿态；total_yaw_rad从同一四元数的yaw按相邻帧最短角差展开，不是body Z角速度积分。无效帧保留旧值且metadata.valid=false；缺样重新对齐时重设偏航并令total_epoch加一。偏航目标必须与反馈属于同一参考段；通常定向的误差按remainder(target-current,2*pi)处理，明确多圈转动才使用同段连续目标。六轴相对航向会漂移，俯仰±90°时欧拉/yaw均有奇异性；全姿态控制使用q_nb，角速度环使用body_rate_rad_s。

P3+新增RateFeedback：filtered_gyro_rad_s在在线残余偏置前，body_rate_rad_s为同帧Mahony偏置扣除后的反馈，angular_accel_rad_s2由filtered_gyro差分再低通，独立derivative_valid在首帧/断流恢复时false。失败不能以旧D冒充当前D。P4A的FlightHealth要求rate/attitude属于同一采样时刻和序号；GuardReason和reset_sequence供未来控制器处理参考变化/清积分。SBUS Frame的时间是完整帧收到的时刻；frame_lost不刷新有效PilotCommand，failsafe显式发布接收机失效，100ms过期也会撤销输出。通道映射当前未分配，原始帧有效不代表已经有有效飞手指令。DShot Batch是软件准备结果，即使STOP批次valid也不表示已有硬件发送。详见[p3plus-p4a.md](p3plus-p4a.md)。
