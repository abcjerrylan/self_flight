# USB 实时姿态页面

`tools/attitude-viewer.html` 是本地单文件HTML，无外部依赖。2026-10-06恢复Roll/Pitch当前姿态，只保留从四元数展开的连续Yaw；需要本轮输出total_yaw_md的新版build/mcu-debug/self_flight.hex。

## 使用

1. 通过USB DFU烧入新版固件，松开BOOT后正常重插USB；保持静止等待启动标定完成。
2. 关闭占用串口的工具，用桌面Chrome/Edge打开tools/attitude-viewer.html。
3. 点击“连接USB”，在设备框选择飞控串口（当前通常COM51）。参数115200，页面自动开启DTR。
4. 页面显示Roll/Pitch当前ZYX姿态、连续Yaw、四元数板卡模型、20秒曲线及诊断。
5. 结束时点击“断开USB”释放串口；物理拔插后需重新连接。

页面只读串口、不发送控制指令。浏览器需用户主动选择设备，Codex内置预览不保证支持设备选择。若本地文件的串口API不可用，可从仓库根目录运行：

```text
python -m http.server 8765 --bind 127.0.0.1 --directory tools
```

再用Chrome/Edge打开http://127.0.0.1:8765/attitude-viewer.html。服务仅绑定本机；数据不上传。

## 数据含义

- q_nb是WXYZ、body FRD→navigation NED；模型始终使用q。正Roll为右侧压低，正Pitch为抬机头，正Yaw为上方看顺时针。
- Roll/Pitch来自rpy_md（当前姿态，毫度）；Yaw来自total_yaw_md（同一q的ZYX偏航展开，毫度），跨±180°继续计圈、反转减小。它不是机体Z角速度积分。
- 无total_yaw_md的旧日志恢复普通Roll/Pitch/Yaw，并提示Yaw跨±180°可能跳变；忽略旧total_rpy_md机体轴积分。不会从日志起点臆造此前圈数。
- 重新上电或解算重对齐会重设偏航参考；total_epoch或设备时间改变时清除旧曲线。六轴航向会漂移，pitch±90°的欧拉奇异性仍存在；跨竖直姿态反馈应使用四元数。
- ATT约50Hz、解算约1kHz；ATT seq是gyro样本号，不能用相邻ATT序号差判断采样丢失。
- valid=0保留最后有效模型；实时600ms无ATT标断流。断开保留最后模型并标断开。
- “演示”是明确标记的合成姿态；“回放日志”支持包含ATT的CSV/TXT/LOG、本地暂停/进度拖动/重播。一次文件需来自一次启动；回放温度/标定诊断为—。

## 验证（2026-10-06）

```text
node tests/host/attitude_viewer_tests.cjs
```

20/20通过：Roll/Pitch使用当前Euler、只有Yaw展开、epoch清图、旧日志普通姿态回退、旧body积分忽略、pitch翻转恢复有限范围、数据单位/分块/有限值、无效/断流、模拟USB DTR/读取取消/锁释放/重连、文件竞态及回放控制。原四份实机记录4802条ATT仍可解析。

固件与主机验证详见p3-attitude.md。新版尚未烧录；本轮未进行浏览器USB设备选择或实机直连接收。历史浏览器演示/实机日志回放的检查发生于10月4日，不代表当前固件实机验证。

Web Serial依据 [Chrome官方文档](https://developer.chrome.com/docs/capabilities/serial) 与 [接口规范](https://wicg.github.io/serial/)。
