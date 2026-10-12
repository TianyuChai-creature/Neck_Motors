# Neck_Motors

F32C 双轴云台控制 SDK。两台电机共用一条 TTL 串口总线，默认 **yaw = ID 1**、**pitch = ID 2**。仓库提供 C++ 共享库、命令行示例、PySide6 控制界面。

## 准备

- Linux、CMake ≥ 3.16、支持 C++17 的编译器；图形界面另需 Python 3 和 PySide6。
- 按[电机手册](docs/F32C无刷电机使用手册-2026-07-21.pdf)接线：电机供电 **8–15 V**，串口为 **3.3 V TTL**，USB-TTL 与电机电源共地。默认通信参数为 **115200 baud**。
- 确认两台电机的 ID 分别为 1、2，并确保当前姿态附近有足够活动空间。机械限位与零点尚未在仓库中标定，首次使能和移动请先小幅操作。
- 当前用户需要有串口设备的读写权限。自动识别会扫描 `/dev/serial/by-id/*`、`/dev/ttyACM*` 和 `/dev/ttyUSB*`，并只读查询两个 ID；也可在代码或界面启动时指定设备路径。

## 构建与首次检查

在仓库根目录运行：

```bash
cmake -S . -B build
cmake --build build
./build/test_protocol
./build/test_serial_log
./build/f32c_ping
```

前两个测试不需要电机；`f32c_ping` 只查询并打印两轴角度，不使能电机。若显示 `open failed` 或 `ping failed`，检查供电、共地、ID、串口路径和访问权限。默认自动识别要求 **两个 ID 都有有效回包**。

## 图形界面

先完成上面的普通构建。安装界面依赖后启动：

```bash
python3 -m pip install -r requirements-ui.txt
python3 examples/ui.py
```

如果要固定串口设备，将路径作为第一个参数：

```bash
python3 examples/ui.py /dev/ttyACM0
```

界面启动时连接电机，但初始为失能状态。点击“使能”后会按界面平滑速度将两轴移向已有的 0°，随后可用 yaw/pitch 滑块设置目标角度（yaw 范围为 −180°～180°，pitch 为 −180°～40°），右侧显示实测角度；“直通平滑速度”控制界面的目标推进速度，默认 5°/s。点击“失能”或关闭窗口会发送失能命令。

0.2.2 的高增益组合在用户 UI 使能时出现剧烈振荡，已撤回。0.2.3 默认采用 **yaw 速度 PI=50/8、位置 PI=63/0；pitch 速度 PI=30/8、位置 PI=60/0**，Python、C/C++ 和 UI 一致。真实 Qt UI 的三轮使能、5°/s 回零和保持测试通过，最长保持 30 秒，末段 yaw 摆幅最大约 0.1°、pitch 未记录到摆幅。相机并行取流、重新上电和 yaw ±60° 全范围尚未验证；使能时临时下发，不保存电机参数或零点。

“到 0°”会移动到**已有零点**；“当前设为零点”会把当前姿态写成零点并保存到电机，重启后仍生效。使能现在会自动运动；确认机械姿态和活动空间后再点击“使能”或使用这两个按钮。使能前会自动开启 Log，在 `logs/` 保存带时间戳的原始串口收发日志，也可手动使用 Log 开关。读反馈或发目标失败、跟随误差超过 3°，以及固定目标 1 秒窗口内摆幅超过 1°时，UI 会请求失能并停止目标下发。

## 命令行示例与 C++ 接口

```bash
./build/f32c_hold  # 回车后使能并保持当前位置；Ctrl-C 失能退出
./build/f32c_zero  # 回车后使能、把当前姿态设为零点并持久保存
```

`f32c_zero` 会修改持久化零点，请在确认当前姿态后运行。C++ 公共接口见 [`include/f32c/gimbal.hpp`](include/f32c/gimbal.hpp)，C 接口见 [`include/f32c/c_api.h`](include/f32c/c_api.h)。典型调用顺序为 `open → ping → enable → set_deg/read → disable`。C++ 中可通过 `Gimbal2Dof::Config` 指定串口路径、ID、速度、加速度和 PI 参数；`set_deg()` 立即依次下发两轴目标，界面的 50 Hz 平滑逻辑不在 SDK 内。

## 安装 SDK（C++ / Python）

在仓库根目录运行 `./package_sdk.sh`，产物固定写入 `wheel/`：一个包含 Python 电机接口、`HeadTracker`、电机共享库和 C++ 开发文件的 Linux 平台轮子。若默认 Python 缺少构建工具，可设置 `F32C_PYTHON=/path/to/python3`。安装只需：

```bash
python3 -m pip install --upgrade wheel/f32c_neck_motors-0.2.3-py3-none-linux_x86_64.whl
```

Python 可直接 `import f32c`，无需单独安装 gz 包或设置库路径。C++ 工程可通过 `f32c.native_prefix()` 得到同一轮子里的 CMake 包前缀，并链接 `f32c::f32c`。完整示例见 [SDK 使用说明](docs/SDK使用说明.md)；打包时也会复制到 `wheel/SDK使用说明.md`。

0.2.3 在失能状态下先下发完整 PI，再按正常模式/速度/初始目标顺序使能，减少启动期间的中间增益组合。新增 `enable_at(yaw, pitch)` 指定立即生效的初始目标，不平滑也不自动限制机械范围。

SDK 的 `enable()` 使能并保持当前位置；`set_deg()` 立即下发目标，不包含 UI 的平滑轨迹；`set_zero()` 将**当前位置**写为持久零点。连接或命令失败会抛出 `F32CError`，上下文管理器退出时关闭设备。

0.1.1 新增 `set_pi(axis, speed_kp, speed_ki, pos_kp, pos_ki)`，可在持续使能时更新该轴临时 PI，保留模式与目标，不保存参数；C++/C 也提供对应接口。全部公开 API 的参数、返回值和资源生命周期见 [SDK 使用说明](docs/SDK使用说明.md)。

0.2.0 whl 提供 Python `HeadTracker` 跟随控制层，可在 `set_deg()` 上方抑制头部角度噪声，生成受速度/加速度约束的最新目标，保护有符号 pitch 路径，并处理输入丢失与控制循环停顿。它不调整 PI、不自动使能，标定范围由调用方显式提供；通过 `TrackingConfig` 配置滤波、防抖和轨迹参数。参数设置方式与接入示例见 SDK 使用说明 §3、§4 和 §4.1。

0.2.0 的 whl 和 CMake 导出仅包含电机组件；图像采集由上层应用单独管理。电机库在打包时重新编译，构建需要 CMake、C++17 编译器及带 pip、setuptools、wheel 的 Python，运行电机 SDK 无需其它设备的开发环境。

```bash
python3 tests/test_tracking.py
python3 examples/tracking_demo.py  # 无硬件模拟，只评估生成目标
```

本轮实测结果与未通过位置见 [2026-10-12 整定记录](docs/整定记录_2026-10-12.md)。当前整套 ±60° yaw 验收未完成；0.2.3 纠正 0.2.2 的 UI 启动问题，使用新的回零保持默认参数，保留历次失败记录。

## 固定负载调参

工具直接使用当前仓库的 Python SDK 和已构建的 `build/libf32c.so`，不需要额外 Python 依赖，试验不修改源码默认 PI、不置零、不保存电机参数。运行期间两轴都会使能，退出或异常时尝试失能。当前标定：pitch 增大为低头，减小为抬头；第一人称 yaw 增大向左、减小向右。yaw 无已知机械限位，负载线缆会增加旋转阻力。pitch 低头极限约 43°，工具的测试目标上限为 40°，给极限留 3° 余量；验证位置的正负阶跃也必须满足此上限。pitch 沿有符号角度推进，避免最短路径绕过低头边界；yaw 仍按单圈最短方向处理。

自动整定先预览，添加 `--enable` 才会连接并执行：

```bash
python3 examples/autotune.py --device /dev/ttyACM0 --center 0 0
python3 examples/autotune.py --device /dev/ttyACM0 --center 0 0 --enable
```

自动流程以启动时实测姿态为中心，逐轴比较速度 KP/KI、位置 KP/KI，在中心及默认 ±15° 位置执行正负 3° 小步验证。`--axis yaw` 或 `--axis pitch` 可只整定一个轴；`--validation-offsets 0 -30 30` 可调整验证位置；`--baseline-only` 只测量给定参数；`--gains speed_kp speed_ki` 可仅搜索速度环。八个 `--yaw-speed-kp` / `--pitch-pos-ki` 等参数可覆盖初始值。默认验收为保持峰峰值 ≤0.6°、静差及阶跃位置误差 ≤1°、恢复至 ±1° 并连续保持 0.5 秒的时间 ≤2 秒；这些是实验指标，均可通过 `--help` 所列参数调整。任一轴跟随误差超过默认 15° 时终止实验并尝试失能，该阈值不是机械限位。

`--center YAW PITCH` 可指定测试中心；例如 `--center 0 0` 可在已标定零位进行整定。全部计划位置在使能前检查；不允许把越界目标静默裁剪后计入验收。若任一轴连续 1.5 秒跟随误差大于 6°，实测角度变化却不超过 0.3°，工具会记录停滞故障并终止，不能通过自动重试或继续增加积分推动阻挡。阈值可通过 `--stall-error`、`--stall-seconds`、`--stall-motion` 调整。2026-10-10 的 56.8° pitch 试验已超出用户后来确认的约 43° 低头极限；该记录保留，但不作为正常工作范围内增加 PI 的依据。整定保留当前线缆负载，只测试边界内的运动。

每次运行独立保存到 `logs/autotune_<时间戳>_<随机后缀>/`。整轮试验持续使能，通过运行时 PI 接口更新参数，结束或异常时失能；避免每个候选之间失能导致头部落回低头极限。根目录保存整轮 `serial.log` 和 `run.json`，每个候选保存 `samples.csv`、`result.json`，其中记录开始时间和共享串口日志路径。仅全部验证通过时 `recommendation` 才有值。运行被打断或验证失败时，不能把 `best_config` 当成验收合格。搜索只覆盖工具列出的有限候选和测试位置，不保证其它位置稳定。

自动测试施加的是目标位置阶跃，不是外力扰动，也不测量绝对刚度。人工受扰实验可用交互工具：

```bash
python3 examples/tune.py --axis pitch --speed-kp 10 --speed-ki 4 --pos-kp 40 --pos-ki 0 --enable
python3 tests/test_tuning.py
```

确认使能后，`t 15` 缓慢移向 15°；目标推进结束后输入 `hold` 记录静止抖动，施加外力前输入 `push`，松手时输入 `release`，等待后输入 `report` 查看恢复指标，`q` / Ctrl-C / EOF 退出。`--yaw-range MIN MAX` 和 `--pitch-range MIN MAX` 可选。该工具每次运行调整一个轴，另一轴采用 SDK 默认 PI；更换参数需退出后重新运行。数据保存于独立的 `logs/tune_.../` 目录。

统计将位置偏差与抖动分开：均值误差代表静差，峰峰值和标准差描述角度变化；恢复时间要求记录末尾连续落在目标容差内，后续越界会撤销“已恢复”，采样空缺不能算作连续稳定。记录实际采样率；CSV 的两轴角度是串行查询结果，不是同步采样。未测得足够扰动或观察时间不足时，恢复时间为 `null`。受扰时的峰值偏移只能在相同外力条件下比较刚度，无法换算为 Nm/rad；按键标记也包含人的操作延迟。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/f32c/` | C++/C 公共接口与协议 |
| `src/` | 电机、串口及云台实现 |
| `examples/` | 命令行示例和 PySide6 界面 |
| `tests/` | 无硬件协议与串口日志测试 |
| `docs/` | F32C 电机手册 |
