# 验证记录

2026-10-03。以下为本轮实际执行结果，工作目录均为最终独立新仓库根目录。

## 原生构建

工具：CMake 3.28.1、Ninja 1.11.1、MinGW GCC/G++ 8.1.0、Git 2.51.0.windows.2。

```text
cmake --preset host-debug -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build --preset host-debug
ctest --preset host-debug
cmake --preset host-release -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build --preset host-release
ctest --preset host-release
```

| 检查 | 实际结果 |
| --- | --- |
| Debug configure / build | 通过，生成静态核心库与原生测试程序，警告作为错误 |
| Debug CTest | 14/14 通过，0 失败，约 0.26 秒 |
| Release configure / build | 通过，使用优化/NDEBUG，测试 CHECK 仍执行 |
| Release CTest | 14/14 通过，0 失败，约 0.21 秒 |

测试覆盖向量与右手坐标、身份和已知轴旋转、非交换组合顺序、逆与 q/-q 等价、缩放归一化/大数、零/近零/NaN/Inf、失败不修改输出、时间边界/倒退/重复/超期、样本有效性和安全默认值。另有 128 组确定性旋转检查，验证长度、内积、逆旋转和组合；不是硬件模拟。

本机最初在临时工作目录内的沙箱 configure 停留在编译器 ABI 检测，未产生完成结果，已停止该次进程。未把这次尝试记作通过，也没有把旧 build 复制到新仓库。最终目标目录以正常本机工具运行上面的全新构建并通过；环境停滞的具体原因未进一步确定。

## 独立性与边界

1. 检查 Debug `compile_commands.json`：3 个编译项全部来自本仓库，只包含本仓库核心 include，原生 G++ 编译；没有 PNX、HAL、ThreadX 或 ARM 工具链 include/link 输入。共享 CMake/presets 没有本机绝对路径，也没有子模块。
2. `cmake -S . -B build/mcu-gate -DSELF_FLIGHT_TARGET=mcu`：按预期失败，明确要求 P2 真实 H743 生成；诊断保存在 `build/validation/mcu-gate.txt`。
3. `cmake -S . -B build/host-toolchain-gate -DCMAKE_TOOLCHAIN_FILE=unused-do-not-load.cmake`：按预期失败，host 不加载交叉工具链；诊断保存在 `build/validation/host-toolchain-gate.txt`。
4. 独立 Git 当前使用 `main`，没有提交、推送或添加远程；构建目录被忽略。P0 目标原先不存在，未覆盖用户项目。
5. 参考工程的结束 `git --no-optional-locks status --short` 与初次结果一致；[12 个抽查文件的 SHA-256](reference-fingerprints.json) 前后逐项一致。未在参考目录构建或生成文件。

## 同一核心的 ARM 编译检查

发现本机 GNU Tools for STM32 13.3.rel1，G++ 13.3.1。对 `math.cpp` 和 `time.cpp` 分别执行以下命令形式，两个目标文件均编译成功：

```text
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror -Iflight/core/include -c flight/core/src/math.cpp -o build/validation/math-arm.o
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror -Iflight/core/include -c flight/core/src/time.cpp -o build/validation/time-arm.o
```

这仅证明当前纯核心源可以编译为 ARM 对象；未链接启动/HAL/RTOS，未在目标执行，也不冻结未来完整固件的全部编译参数。

## 硬件

CubeMX 生成：未执行。MCU 固件编译链接：未执行。烧录/IMU/电机/飞行：未执行。
