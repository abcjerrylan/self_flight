# P3：原创六轴姿态估计

日期：2026-10-03。算法、实时服务、日志及回放已实现；桌面静止与三轴硬件方向验收通过。

## 算法与坐标

机体FRD、导航NED，Hamilton WXYZ的q_nb把body向量转到navigation。正横滚为右侧压低，正俯仰为抬机头，正偏航为水平从上方看顺时针。提供ZYX欧拉姿态转换，常规倾角可用于姿态反馈；pitch接近±90°时roll/yaw有表示奇异性，判断姿态应使用四元数/重力方向。

四元数传播为normalize(q_nb * [1,omega*dt/2])，每步归一化，不把Euler角直接累加。采用一阶离散传播，默认陀螺时间间隔上界3000us，序号缺口也拒绝传播；不是跨丢样的固定1ms积分。

加计是比力：静止水平body为(0,0,-g)，观测的down方向=-normalize(accel)，预测down=R(q_nb)^T*(0,0,1)。原创实现Mahony形式的e=cross(observed_down,predicted_down)*weight，omega=filtered_gyro-bias+kp*e，bias-=ki*e*dt。公式参考[Mahony/Hamel/Pflimlin原论文](https://openresearch-repository.anu.edu.au/items/db91f02d-9a17-4ead-bc64-86ed5bc9c6f4)，源码自行实现，没有复制PNX或第三方算法模块。

初始化从可信重力方向得到roll/pitch，yaw设0；这个零点是初始化参考，不是地理北。启动gyro偏置已经在IMU服务中扣除；估计器的bias是剩余body-axis偏置，和CAL里的sensor-axis启动bias不同。bias仅在weight≥0.5且去偏置后角速度模长<0.35rad/s时学习，模长限制为0.05rad/s。不对无可信重力的外力、快速旋转或无效加计继续积分偏置。

无磁力计/航向观测时绝对yaw不可用，absolute_yaw_valid始终false。水平静止时重力轴零偏无法由加计识别，相对yaw仍会漂移；不宣称绝对航向保持。P5偏航先做角速度控制。

## 滤波与可信度

一阶RC低通系数alpha=dt/(dt+1/(2*pi*cutoff))，首个有效样本直接作为初值。gyro默认80Hz、accel20Hz；系数分别使用真实gyro/accel测量间隔，800Hz加计重复观测不会重复更新滤波。cutoff=0可旁路软件滤波。现有BMI088 gyro硬件带宽116Hz，原始日志不改变；这些是桌面验证的起始参数，尚未经带电机振动或飞行调参。

可信度同时看原始/滤波比力模长：离g越远权重越低，偏差达到15%时为0。零、非有限、未来或过期加计不提供纠正；幅值明显异常的突发不会进入低通缓存，后续可信观测重新建立滤波初值。没有加计时可继续有效gyro传播，但不能用这种输出宣称重力/航向观测正常。恒定外力仍可能伪装成重力，模长门限不能解决全部动态加速度问题。

## 时序与运行

- core不获取硬件时间、不含HAL/ThreadX、不分配内存。now由服务传入，输入需满足metadata时间顺序和5000us新鲜度。
- imu线程优先级5独占SPI；attitude线程优先级6等待gyro发布事件；USB线程优先级15。事件可能合并，使用序号统计缺口，不把它当成逐样本队列。
- 每消费者一个ImuCursor：重复/倒退gyro不消费，未来accel暂时不用，已用加计可在新gyro时保持最近观测。gyro/accel两种标定均成功才进入姿态链。
- gyro序号缺口或过大正向时间间隔使姿态无效；下一可信加计重新初始化tilt/yaw及残余bias。重初始化会把相对yaw设0，必须由以后P4健康状态机处理，不能在飞行中当成连续航向。
- NaN/无效gyro不更新姿态与bias；重复/倒退时间不传播。姿态可用时间记录解算完成时刻；gyro测量到完成达到1000us时服务将其标记无效并统计late，快照年龄超过5000us也无效。
- 连续yaw在快速姿态线程由四元数逐帧提取/展开，Roll/Pitch仅在需要时由attitude_euler计算；快照发布四元数、body rate、bias和total_yaw。新增开销尚未在板上测量。四路电机仍是低电平GPIO。

## USB记录

约50Hz的ATT包含seq/t、q_u（WXYZ×1e6）、rpy_md（度×1000）、total_yaw_md（四元数偏航展开，度×1000）、total_epoch（偏航参考段编号）、bias_u（body rad/s×1e6）、aw_m（可信度×1000）、dt_us、run_us（算法耗时）、lat_us（gyro测量到解算完成）、valid、yaw_abs、err。每秒AHRS包含updates/reject/timing/missed/timeout/late及累计maxrun/maxlat。错误None=0、WaitingForAccel=1、InvalidSample=2、Timing=3、Config=4、Numerical=5。

STATS、CAL、TEMP、PIPE和原始IMU CSV保留。USB未打开时logdrop增长表示日志丢弃，不等于采样或姿态丢样；验收看采集窗口的计数增量。启动外设初始化期间的AHRS timeout会计数，正常连续采样时不得继续增长。

## 主机测试与回放

```powershell
cmake --build --preset host-debug
ctest --preset host-debug
python -B tests/host/replay_tests.py
python -B tests/host/attitude_replay_tests.py
python tools/replay-attitude.py build/p3/board-static.csv --output build/p3/board-static-replay.csv
python tools/analyze-attitude.py build/p3/board-static.csv
# 较老的无CAL日志需要显式提供此物理板卡参数：
python tools/replay-attitude.py build/calibration/body-z-minus.csv --calibration build/calibration/calibration.json
```

replay-attitude.py调用native attitude_replay，实际链接同一C++ core，不使用另一套Python Mahony。默认读取文件中首个Accepted CAL的微单位参数；也可用显式JSON覆盖。原始行保留顺序并经同一ImuCursor消费。回放在文件起点重新初始化，所以yaw/bias初值和真实固件历史不相同，不应逐点强求绝对yaw重合。首个gyro先于accel时会有一帧WaitingForAccel，这是显式启动等待。

Debug/Release各34/34：保留原26组，新增低通实际dt、三轴传播/组合/长时间归一化、800Hz加计合成旋转、静止倾斜/倒置/纠正、可观测残余bias收敛/上限/不可观测yaw、异常比力/NaN、时间/序号异常与恢复。Python原始回放4/4，P3集成每种native构建3/3。旧六面回放说明pitch接近±90°时Euler roll/yaw变化不能误当作四元数发散。

## 本轮硬件证据

用户自行烧入P3 Debug，通过COM51采集。20秒静止有1001条ATT且全有效，更新999.02Hz，日志量化四元数最大模长误差6.16e-7；roll/pitch均值-0.402°/-1.126°，标准差0.0102°/0.0119°；相对yaw变化-0.119°。roll/pitch非零包括摆放倾斜，不代表已知水平误差。AHRS累计最大解算137us/延迟256us；采集窗口无新增拒绝、时间错误、丢样、超期、SPI错误或logdrop。

右侧压低后roll约+36.05°，抬机头后pitch约+14.76°；各8秒、400条ATT全有效，方向正确。手动摆放没有精密角度参考，抬机头时存在约13.5°–16.5°晃动，不能宣称角度精度达标。用修正加计均值独立算出的俯仰为+14.766°，与姿态均值+14.759°一致，支持实际摆放角度约15°而非精确30°。水平顺时针转动采集60秒，相对yaw从-1.601°到+90.439°，增加+92.04°；3001/3001帧有效，方向正确。此动态窗口整体标准差包含有意转动，不能当作静止噪声。最后AHRS累计最大解算180us、延迟282us，动态窗口无新增错误、丢样、超期或日志丢弃。

原始记录与报告位于build/p3：board-static/roll/pitch/yaw.csv及-report.json、old-static-replay-report.json、old-six-face-replay-report.json、board-static-replay-report.json。新实机数据也成功通过原生回放，首帧等待后19,993条有效；初始化历史不同带来的倾角收敛/偏航零点差异已保留。

尚未进行：磁航向/飞行验证、振动滤波调参、P4输出/RC/状态机；这些不属于已完成P3软件功能。

## 本地实时姿态查看

新增 [单文件 HTML](../tools/attitude-viewer.html)，模型可读取原P3日志；Roll/Pitch显示当前姿态，连续Yaw需要含total_yaw_md的新版固件；旧日志恢复普通Euler显示并提示。连接和显示边界见 [使用说明](attitude-viewer.md)。

## 机体轴累计转角（2026-10-04，历史实现；已由10月6日姿态反馈版本替换）

用户实测模型正确，但pitch越过90°时数值折返、roll/yaw跳变。先前的最近ZYX等价分支只能展开欧拉表示，不能保证带初始倾斜的局部轴旋转独立累计。初始roll偏0.5°，持续绕body Y旋转即可在同一C++ Mahony中复现；旧测试的固定roll/yaw欧拉合成路径未覆盖这条实际运动轨迹。旧 continuous_euler 已删除。

保留 total_roll_rad、total_pitch_rad、total_yaw_rad 名称，含义改为绕FRD机体X/Y/Z轴的有符号累计转角（弧度）。每个成功解算帧执行 total += (filtered_gyro - residual_bias) * actual_dt；启动gyro偏置已由IMU服务扣除。不积分Mahony的kp重力反馈，避免静止时姿态纠正被计作实际转动。

正向转动经过90°、180°、360°继续累计，反转相应减小；初始化姿态无论水平、倾斜或竖直，累计值都从0开始。无效/重复/倒退样本不提交新累计值；缺样或过大正向时间间隔拒绝传播，下一可信姿态重新开始累计段，归零且total_epoch加一，网页清除旧曲线。上电不恢复以前的累计量。

这些积分随时间漂移，而且机体轴随姿态转动；混合转动时三轴积分不能唯一重建姿态，不等于对地欧拉角。普通rpy_md仍是ZYX诊断，会在竖直附近折返/换支；q_nb驱动模型。当前没有PID/混控/输出控制器，未发现任何控制链使用total或Euler。后续姿态控制需用四元数误差，角速度控制使用校正后的body_rate，不能把累计量当成姿态反馈。可参考 [PX4官方控制框图](https://docs.px4.io/main/en/flight_stack/controller_diagrams)。

USB ATT保留total_rpy_md与total_epoch，新增total_kind=body。HTML只将此标记的total用于数字/曲线；旧欧拉total日志及更旧的无total日志仍显示q模型，累计栏为—并提示烧入新版。native回放CSV同样追加total_kind，Python回放会拒绝未带body语义标记的旧回放程序；USB汇总不把旧欧拉累计解释成机体轴积分。网页演示改为单机体Y轴转动，文案明确积分语义和漂移边界。

### 本轮验证

- host Debug/Release各37/37：三轴正负4圈、混合三轴实际800/1200us积分、反转抵消、倾斜/竖直启动归零、无效/重复/倒退保持、缺样恢复epoch，以及gyro=0但重力反馈改变姿态时累计量保持0。
- 专项回归：初始横滚±0.5°，默认滤波/Mahony，带小幅加计噪声，绕机体Y正负各转720°；累计pitch单调且每帧步长<0.002rad，total roll/yaw为0，四元数与已知姿态一致。
- Python原生姿态集成Debug/Release各6/6；原始IMU回放4/4；HTML 20/20。页面检查旧欧拉total的兼容提示和pitch跨90°到720°的独立累计。原四份实机日志4802帧仍可解析和显示模型。
- 同一native Debug核心生成build/p3/body-pitch-host.csv、.log及-report.json：初始roll0.5°、body Y 90°/s、8秒，8001/8001帧有效，末尾total=(0,719.964722,0)°，浮点累计误差约0.0353°；不把此积分宣称为无漂移或精密角度测量。
- 浏览器载入该主机日志401帧，末尾显示Y累计720.0°、X/Z均0.0°，曲线连续，无页面运行错误。日志中的run_us=0/lat_us=30是合成占位值，不是硬件时延；此例不是新版实机采集。
- MCU Debug/Release编译链接通过。Debug Flash105,644B/RAM_D1 69,464B，Release Flash67,276B/RAM_D1 69,416B。Debug HEX SHA-256：2a937b48c3b832a8738b1b7116e4e4dfd523c6a78af0b4d600480f6e5c38b05e。

本轮没有烧录、打开串口或输出电机信号，代码仍在main且未提交/推送；新版实机total/漂移和算法耗时需要用户烧录后验证。

## 恢复当前姿态，单独保留连续yaw（2026-10-06）

用户明确需要姿态反馈，因此删除total_roll_rad/total_pitch_rad和三轴body积分。保持原来的四元数Mahony传播/滤波/重力修正；Roll/Pitch通过attitude_euler(state.q_nb,out)取得当前ZYX姿态，倾斜启动直接反映摆放倾角。

total_yaw_rad每个成功帧从同一q_nb提取yaw，执行total_yaw += remainder(yaw-previous_yaw,2*pi)，跨179°→-179°时继续181°，反转减小。首帧以当前yaw初始化；失效帧不提交新值，缺样后重新对齐重设参考并令total_epoch加一。它与Euler航向一致，倾斜时不等于body Z角速度积分；展开只有同参考段且相邻姿态可跟踪时有效。pitch±90°仍是yaw/roll欧拉奇异点，不承诺翻滚时连续独立航向。

USB保留rpy_md并改为独立total_yaw_md；移除total_rpy_md/total_kind。HTML数字/曲线使用(rpy.roll,rpy.pitch,total_yaw)，q模型保留；旧版日志恢复普通三轴Euler显示，明确提示Yaw未展开，不拿旧body累计量当姿态。native CSV只保留total_yaw_deg与total_epoch，两个汇总工具同步。

常规小倾角姿态反馈可使用Roll/Pitch；普通定向yaw误差按最短角差计算，明确多圈目标则应与total_yaw属于同一参考段。使用反馈前需检查有效性/新鲜度及段号，重新对齐不能当作连续航向。六轴yaw没有绝对航向，会漂移；跨竖直或翻滚控制使用四元数姿态误差，角速度环使用body_rate_rad_s。当前尚无PID/混控控制器，本轮没有接入控制输出。可参考 [PX4控制框图](https://docs.px4.io/main/en/flight_stack/controller_diagrams#multicopter-attitude-controller)。

验证：host Debug/Release各37/37（1.25s/1.02s）；Python姿态集成各6/6、原始回放4/4；HTML20/20。覆盖yaw正负4圈、±180°展开、反转、实际800/1200us时间、无效/重复/倒退保持、缺样恢复；倾斜body Z旋转时total_yaw与四元数Euler航向一致且与body积分不同；Roll/Pitch恢复当前值、旧字段忽略与旧日志回退。两种MCU编译链接通过：Debug Flash105,988B/RAM_D1 69,456B，Release Flash67,676B/RAM_D1 69,408B。

同一native核心生成build/p3/yaw-feedback-host.csv/.log/-report.json：保持roll12°/pitch-8°并绕导航Z转720°，2001/2001有效，末帧roll=12.000005°, pitch=-7.999994°, total_yaw=719.988098°。该记录为主机合成，不是板上测量；日志run/lat是占位值。Debug HEX SHA-256：99aef09f5049bb45ea9dc129c531af9d0ca3c48840851d6b437f8ec084717a4b。

新版尚未烧录，新增atan2/展开耗时和实机姿态输出待测；未打开串口或输出电机信号。仍使用main，未提交/推送。
