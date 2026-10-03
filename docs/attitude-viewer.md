# USB 实时姿态页面

`tools/attitude-viewer.html` 是单个本地 HTML，无外部依赖、不需要安装网页框架。机体轴累计版本需要新版固件的 total_rpy_md、total_epoch 与 total_kind=body；原先 P3 固件需重新烧录本轮 build/mcu-debug/self_flight.hex。

## 使用

1. 关闭占用飞控串口的工具。
2. 用桌面 Chrome 或 Edge 打开 `tools/attitude-viewer.html`（可以把文件拖入浏览器）。Codex 内置预览不保证支持设备选择。
3. 点击“连接 USB”，在浏览器的设备选择框中选择飞控串口，当前板卡通常为 COM51。
4. 正常上电后先静止等待标定完成，再转动板卡。页面显示板卡模型、绕机体X/Y/Z轴的累计转角、最近 20 秒曲线、温度、标定状态与解算诊断。
5. 使用完点击“断开 USB”释放串口。物理拔插后重新点击连接。

串口参数为 115200，页面自动开启 DTR；此固件 DTR 未开启就不会输出日志。浏览器需要用户主动点击连接并选择设备。页面只读取串口，不发送控制指令。

若直接打开本地文件时浏览器不支持串口，可从仓库根目录启动本地服务：

```text
python -m http.server 8765 --bind 127.0.0.1 --directory tools
```

然后用 Chrome / Edge 打开 `http://127.0.0.1:8765/attitude-viewer.html`。服务仅绑定本机，结束后停止服务。网页本身不上传数据。

## 显示与回放

- 模型直接使用固件 WXYZ 四元数 q_nb，body FRD→navigation NED；total_rpy_md 是校正后机体角速度的有符号积分，单位毫度；total_kind=body 时数字与曲线使用total，解算重初始化的total_epoch改变时清除旧曲线。正 roll 右侧下、正 pitch 抬机头、正 yaw 上方看顺时针。拖动模型区域只调整观察视角。
- 六轴估计无磁航向，q模型的yaw是相对角度。三轴累计值会漂移、从本段起点0开始，反转减小，跨90°/360°继续累计；它们不是当前对地欧拉角，不用于姿态控制。旧欧拉total日志缺少body标记时累计值不显示并提示更新固件。
- ATT 日志约 50 Hz，解算约 1 kHz，两者分开显示。seq 是 gyro 样本序号，不能用 ATT 序号差判断丢帧。
- valid=0 保留最后有效模型；600 ms 没有 ATT 标记断流。断开后保留模型并标记已断开；实时设备时间倒退会清除旧趋势。
- “演示”使用明确标记的合成数据；“回放日志”在本地读取包含 # ATT 的 P3 CSV/TXT/LOG，支持暂停、进度拖动和从头重播。回放不加载温度与标定诊断，显示为 —；一次文件须属于同一次启动。
- 原始 IMU 数据仍同时经过 USB，页面按完整行分块处理并忽略非姿态行；显示约 30 Hz，曲线和尾行缓存有上限。

## 本次验证（2026-10-04）

```text
node tests/host/attitude_viewer_tests.cjs
```

当前20项通过，新增旧欧拉total拒绝误标及pitch穿过90°/两圈独立累计；其他覆盖：真实单位、每个分块边界、CRLF/混合日志、超长行恢复、无效与非有限值拒绝、三轴符号/90°俯仰、无效模型冻结与断流、设备重启、模拟 USB 的 DTR/取消/释放锁/关闭/重连、取消设备选择、读取错误、导入竞态、回放拖动和从末尾重播。已有四份实机日志共 4802 条 ATT 均成功解析。

Codex 内置浏览器已检查演示、桌面布局、实际 board-yaw.csv 文件回放、末尾重播和控制状态；没有页面运行错误。浏览器实际 USB 设备选择与持续直连接收仍需在桌面 Chrome / Edge 上确认；模拟串口检查和旧实机日志回放不等同于本次浏览器直连验收。原始HTML版本不涉及固件；本轮累计角加入核心和USB输出，固件编译通过但尚未烧录，电机输出未变。

Web Serial 的用户手势、DTR 与取消读取流程依据 [Chrome 官方文档](https://developer.chrome.com/docs/capabilities/serial)，安全上下文与接口依据 [Web Serial 规范](https://wicg.github.io/serial/)。

本轮浏览器验证：载入 build/p3/body-pitch-host.log（同一C++核心的主机生成记录，初始横滚0.5°），Y轴转两圈末端显示pitch累计720.0°、roll/yaw累计均0.0°，曲线连续，无页面运行错误。新CSV实值719.964722°的约0.0353°误差来自浮点累加；日志中的耗时/延迟为合成值，此例不是实机采集。需要USB DFU烧入新版HEX，再重新打开HTML连接USB。
