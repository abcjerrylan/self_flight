# IMU 标定：静止零偏与六面加计

本阶段补齐原方案P2B的标定：便携C++算法、静止质量判定、六面偏置/比例拟合、实际板卡采集和修正接入。原始CSV保持不变，标定在传感器SI坐标执行，然后转为body FRD。

## 实现与质量

`flight/core/calibration` 不依赖HAL/RTOS，不分配内存。Welford在线均值/方差使用float，与目标FPU一致。单一采样线程持有启动标定器；静止条件不合格时清空两路候选窗口，不接受部分旧窗口。

| 条件 | 当前门限 |
| --- | --- |
| 连续合格时间 | 两路共同覆盖至少3秒，gyro≥2500点、accel≥2000点 |
| 序号与时间 | 同一路序号按uint32连续，时间严格递增；gyro间隔≤3000us、accel≤4000us |
| 输入有效性 | metadata有效、数值有限、测量≤可用、延迟≤5000us；另一传感器年龄≤5000us |
| 即时运动检查 | gyro模长≤0.20rad/s；加计模长8.5–11.0m/s² |
| 至少30点后 | gyro均值模长≤0.05rad/s，最大轴方差≤0.0009(rad/s)²；加计均值模长8.8–10.8m/s²，最大轴方差≤0.04(m/s²)² |
| 六面姿态 | 对应轴±8.8–10.8m/s²，其他轴绝对均值≤0.8m/s² |
| 参数范围 | gyro零偏模长≤0.05rad/s；加计偏置模长≤1m/s²；各比例0.9–1.1 |
| 六面拟合残差 | 各面修正均值的模长与9.80665之差≤0.15m/s² |

这些是首版工程门限，根据当前板卡静止噪声设定，不是Bosch给出的静止认证。仅靠IMU无法排除门限以下的恒定偏航转动或恒定外力；标定时用户仍必须固定板卡。没有静止合格窗口就继续等待，不设置假偏置，也不把定时等待结束作为成功。

六面算法按传感器轴+X/-X/+Y/-Y/+Z/-Z配对：`bias=(positive+negative)/2`，`scale=2*g/(positive-negative)`。只估计三轴偏置和对角比例，不估计交叉轴、不正交或温漂曲线。

## 当前板卡参数

2026-10-03用户实际固定六个姿态，每面采集8秒。共38,600条加计、47,987条陀螺记录；六面两路均0无效/序号缺口/重复/倒退。原始文件在`build/calibration/body-*.csv`，其摘要和统计保存在[本板记录](calibration-micoair743v2-20261003.json)。

| 参数（传感器轴） | X | Y | Z |
| --- | --- | --- | --- |
| accel bias m/s² | -0.019264221 | +0.074710846 | +0.011115074 |
| accel scale | 1.000836850 | 0.998527408 | 1.002721430 |
| 静止参考gyro bias rad/s | -0.001097233 | +0.001612638 | +0.000778613 |

六面拟合数据修正后比力模长均值9.80790–9.81803m/s²。它们用于拟合，不能当作独立验证数据。gyro参考来自元件面朝上的文件在共享C++核心中找到的第一个合格3秒窗口；固件不会固定沿用这组gyro偏置，而是在每次启动重新估计。

加计参数生成在`boards/micoair743v2_aio35/imu-calibration.hpp`，随固件编译保留。首版没有Flash参数写入服务，也没有USB标定命令/自动六面识别。板型tag `0x74323535`、器件配置tag `0xb0880f1e`和format_version=1用于兼容检查；tag不是此物理板卡的唯一序列号，换板必须重新标定，不能直接复制本板参数。

## 板上使用

启动时验证加计参数版本/身份/有限值/范围；不合格就维持accel_calibrated=false。gyro找到连续3秒静止窗口后记录bias、方差、时间与温度。两者完成后CalibrationQuality才为Accepted；原始样本在此之前仍可能有效，不能把原始样本valid等同于标定合格。

加计按`(sensor_si-bias)*scale`修正，陀螺按`sensor_si-bias`修正，再执行安装变换。修正结果在快照中发布，available时间包含修正计算；原始CSV的available仍表示原始读取完整时间。接受偏置后正常转动不会重新学习零偏；要重做启动零偏时正常重启并静止。

温度来自[BMI088数据手册1.9](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)的accel 0x22/0x23，11位补码，0.125°C/LSB加23°C，拒绝无效码；仅在gyro接受时读取一次。读失败或超出-40–85°C时temperature_valid=false，不伪造温度，不进行温漂补偿。旧固件采集的六面记录没有温度，离线参考明确temperature_valid=false。

每秒USB新增：

- `# CAL`：gyro/accel完成标志、quality、err、采样数和restart；sensor-axis bias/accel_bias/scale以百万分之一单位记录，var_n为最大gyro轴方差×1e9，temp_mc为毫摄氏度。
- `# CORR`：body FRD修正后的最新两路序号、三轴值与valid；ax_u等为m/s²×1e6，gx_u等为rad/s×1e6。可用相同序号在原始CSV中找到记录核对修正。

err为None=0、Collecting=1、InvalidSample=2、Timing=3、Motion=4、Face=5、Parameters=6；quality为Unknown=0、Rejected=1、Accepted=2。Rejected仅作为数据类型取值，本实现不把一轮运动永久锁死，而清空候选窗口重新等待。

## 可复现工具

在仓库根目录关闭占用COM51的串口工具：

```powershell
./tools/capture-imu.ps1 -Port COM51 -Seconds 8 -Output build/calibration/body-z-minus.csv
# 对x-plus/x-minus/y-plus/y-minus/z-plus/z-minus六面分别保存同名文件
cmake --build --preset host-debug
python tools/calibrate-imu.py build/calibration --header boards/micoair743v2_aio35/imu-calibration.hpp
python tools/replay-imu.py build/calibration/body-z-minus.csv --calibration build/calibration/calibration.json --output build/calibration/body-z-minus-corrected.csv
```

六面工具调用原生`calibration_fit`，实际复用固件C++静止和拟合核心，没有另一套Python拟合公式。全采集窗口有无效/缺口/重复/倒退、过短或过噪时拒绝生成新参数。回放加`--calibration`后保留原始sensor SI，body SI为修正结果；陀螺使用JSON中离线参考bias，不自动等于板上新一次启动bias。

校准后要再采集独立静止数据、验证板上CAL=1/1、质量Accepted和CORR对应原始值的修正，并检查采样速率/延迟/丢样。实际状态见[验证记录](validation.md)和[进度](progress.md)。电机输出仍为低电平GPIO，姿态估计后续在P3实现。
