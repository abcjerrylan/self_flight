# P0 只读参考审查

日期：2026-10-03。参考对象：共同工程父目录下的 `pnx_template`；本仓库为独立 `self_flight`。

参考 HEAD：`237b4a2f3baf26fd32f59fe67f3e4ec90c8201d1`。工作区存在未提交修改，因此本次结论描述当时工作区，并非只描述该提交。

## 实际阅读范围

阅读参考 AGENTS.md、顶层 CMake、`docs/project-structure.md`、`docs/startup.md`、`docs/thread.md`，SPI/DMA/DWT API 文档；查看 `app/app.cpp`、H723 `Core/Src/app_threadx.c`、SPI 公共接口、DWT 时间扩展实现以及 BMI088 读取/标定/温控相关路径。

参考 AGENTS.md 用于理解设计，不把其 H7/F4 迁移、双板验收、子模块分支规则作为新工程要求。检查了目标目录祖先工作规则文件，当时未发现适用 AGENTS.md；新项目规则由本仓库 AGENTS.md 建立。

## 观察与采用方式

| 实际观察 | 新工程处理 |
| --- | --- |
| board/BSP/device/module/app 职责分离；公共 SPI 接口没有 HAL handle | 借鉴边界，自写小接口，不复制模块 |
| Board ThreadX 初始化调用 app_start；当前 app_start 调用 app::motor::start | 仅借鉴启动所有权，不移入 CAN 电机应用 |
| startup 文档仍描述诊断条件启动，与当前 app 源码有差异 | 以源码为当前事实，不修改参考文档 |
| 顶层 CMake 在 project 前默认选择 ARM 工具链与 H723 profile | 新 host 构建独立，未读取 board/ARM/HAL |
| SPI 接口区分阻塞/IT/DMA并要求调用方保持 buffer 生命周期 | 后续 BSP 明确所有权/完成/超时；P1 未实现 SPI |
| DMA 文档用专用内存段、对齐和缓存处理 | H743 按自己的内存/控制器重新验证，不照搬 H723 |
| DWT 通过计数下降发现回绕，使用静态 volatile lock | 参考思路；平台仍需验证并发和最大读取间隔，不把 volatile 当同步 |
| BMI088 静止标定失败路径回退预置陀螺偏置，关闭温控路径仍有≥20°C就绪条件 | 新板独立标定，不以回退固定偏置或固定温度替代合格标定 |
| BMI088 read_acc 中处理坐标/温度，read 汇总 sensor_time，但该读取路径未填入测量时间 | 新数据契约分别保存两路测量/可用时间，尚待 P2 实现来源 |

没有复制源码，没有在参考工程生成配置、运行构建、修改子模块或 Git 状态。历史文档 pass 不作为本轮验证。

## 已有修改记录

初次 `git --no-optional-locks status --short`：

```text
 M .clangd
 M AGENTS.md
 M app/app.cpp
 M boards/h723_mc02/Core/Src/fdcan.c
 M boards/h723_mc02/h723_mc02.ioc
 M configs/boards/h723_mc02/params.json
 M configs/boards/h723_mc02/robot.json
?? app/app_motor.hpp
?? app/motor.cpp
```

`git submodule status` 在当前沙箱中因 Git 子进程创建 signal pipe 被拒而未成功；未重试会写入参考目录的操作，也不把它当作子模块完整性证明。只读实际文件已经足够完成本阶段分层审查。

只读保护的结束检查和文件摘要证据记录在 [验证记录](validation.md)。该检查可证明本轮观测到的状态/文件一致，不声称检测了参考目录所有隐藏文件的外部变化。
