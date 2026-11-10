# Neck_Motors

F32C 双轴云台（颈部）控制。两台电机共用一条 TTL 串口总线，默认 **yaw = ID 1**、**pitch = ID 2**。

纯 Python 标准库。当前主路径是简化核心：`neck/protocol.py` 组帧，`neck/link.py` 非阻塞串口与收发调度，`neck/neck.py` 100 Hz 跟随、使能/失能和置零。旧的 `neck.driver` / `neck.control` 保留给实验工具与回归参考。设计与状态机见 [`docs/简化架构.md`](docs/简化架构.md)，安装与 API 用法见 [`docs/使用说明_0.6.0.md`](docs/使用说明_0.6.0.md)。

## 准备

- Linux、Python ≥ 3.10。界面工具另需 PySide6-Essentials（`pip install -r tools/requirements.txt`）。
- 按[电机手册](docs/F32C无刷电机使用手册-2026-07-21.pdf)接线：电机供电 **8–15 V**，串口 **3.3 V TTL**，USB-TTL 与电机电源共地，**115200 baud**。
- 两台电机 ID 分别为 1、2；零位存在电机 flash 里；可在界面里把当前位置“写零位”（两轴一起，不可撤销，见下）。首次使能前确认当前姿态附近有足够活动空间。
- 当前用户需有串口读写权限。不指定串口时会扫描 `/dev/serial/by-id/*`、`/dev/ttyACM*`、`/dev/ttyUSB*`，只发查询，选中两个 ID 都应答的那个。

## 安装为包（可选）

```bash
pip install wheel/f32c_neck_motors-0.6.0-py3-none-any.whl     # 纯 Python，只含简化核心的 neck 包，约 18 KB
```

**0.6.0 已做 yaw 提速台架 A/B：同一组 ±50° 动作下 yaw 45 RPM 实测峰值约 91°/s、0.5 s 持续约 88°/s；pitch 保持 15 RPM。** 100 Hz、1.5 ms 帧间隔、长时间负载变化和置零写 flash 仍需继续验收。安装、API、失败策略与置零流程见 [`docs/使用说明_0.6.0.md`](docs/使用说明_0.6.0.md)。重新构建：`pip wheel . --no-deps -w wheel`（`tools/` 与测试不在包里）。

**旧 `wheel/0.4.0` 和已发布 AppImage 仍是旧两层实现**，没有 `NeckConfig` / `SerialLink` / 新 `Neck` API；`0.5.0` 是上一版简化核心，`0.6.0` 只包含新核心，旧 `neck.driver` / `neck.control` 只在源码仓库里供实验工具使用。

## 免安装运行 UI（Linux x86_64）

从 [Releases](https://github.com/TianyuChai-creature/Neck_Motors/releases) 下载其一（内含 Python 与 PySide6，无需另装）：

```bash
chmod +x f32c-neck-ui-linux-x86_64 && ./f32c-neck-ui-linux-x86_64 [/dev/ttyACM0]    # 单文件，首次启动要解压几秒
chmod +x F32C-Neck-UI-x86_64.AppImage && ./F32C-Neck-UI-x86_64.AppImage             # 无 FUSE 时加 --appimage-extract-and-run
```

- 在 glibc 2.34 的系统上构建，glibc 更老的发行版（如 Ubuntu 20.04）跑不起来；需要桌面环境（X11/Wayland）和串口权限（`dialout` 组）。
- 打不开串口会弹窗提示并以非零码退出；日志写到 `~/f32c_logs/`。
- **只做过假总线冒烟测试，未在真机和真实显示器上验证。**
- Windows 暂无 .exe：驱动层用 `termios`，Windows 没有串口支持，需先补 Windows 串口层。
- 自行构建：`pip install pyinstaller PySide6-Essentials`，装好 appimagetool 后运行 `packaging/build_linux.sh`，产物在 `build/`。

## 最小用法

```python
from neck import Neck, NeckConfig

with Neck(NeckConfig(port=None)) as neck:  # None: 扫描两个 ID 都应答的口；也可显式传 /dev/ttyACM0
    neck.enable(home=True)                 # 以实测位姿使能，再平滑回已存零位
    while running:
        neck.set_target(yaw, pitch)        # 最新目标胜出；控制器线程按 100 Hz 推进
        status = neck.status               # enabled/link_ok/reason/measured/command/target
```

`status.reason`：空为正常；`input_idle` / `feedback_stale` / `lead_guard` 只冻结目标、保持使能；`link_lost` 是配置为失能时的链路丢失。`status.torque` 表示“请求已发/硬件未知”，不是电机确认。置零用 `neck.zero(save=True)`，两轴一起写入并校验；失败返回 `ZeroResult`，不会自动把整个系统锁死。

## 工具

```bash
python3 tools/ui.py [/dev/ttyACM0]         # 面板：使能并到零位 / 失能 / 到 0° / 暂停输入 / 写零位 / yaw、pitch 滑条
python3 tools/tune.py --axis pitch --speed-kp 10 --speed-ki 4 --pos-kp 40 --pos-ki 0   # 默认仅预览，加 --enable 并输入 ENABLE 才通电
```

固定负载对照入口 `tools/load_compare.py` 仍走旧两层 `neck.control.Neck`（legacy 路径），默认只打印计划，不打开串口：

```bash
.venv/bin/python tools/load_compare.py --load A --device /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E676396-if00 --output logs/load_compare_SESSION/A
```

实际串口、运动范围和停止条件经人工确认后，才加 `--run`。B 状态改用 `--load B` 和新目录 `logs/load_compare_SESSION/B`；变更负载前必须失能并由人确认断能/支撑。两种状态执行同一计划：先 pitch 0° 8 秒，各中心 0°/20°/30° 依次 `c → c+3° → c−3° → c`、每段 6 秒、一次，最后回 0° 6 秒；共约 86 秒。默认从使能前回读的 yaw 原位保持，首次 tick 前替换 SDK 排队的回零目标，不执行 yaw 回零；实验二候选用 `--yaw-target` 沿用基线日志中的 yaw 目标。按用户最新要求取消工具额外的固定 yaw 起始容差，SDK 限位和保护不变；使能前仍要求实测 yaw 在 SDK 的 ±60°范围内、pitch 在 −3°…45°，不符合则不使能。这里只声明目标范围，不能保证机械运动无过冲。

SDK 轨迹、电机参数和保护默认值不改。任何 SDK 保护事件（包括通信等待）、指令误差持续 >1° 且 1.5 秒实测变化 ≤0.3°、段末一秒未持续到位 ±1° 或 pitch 摆幅 >1°、异常/中断均结束本次运行；现场异常振动须立即 Ctrl-C 或采取现场断能措施，不自动重启。默认运行写基线 PI；实验二分支A可用 `--pitch-speed-kp 12` 只替换 pitch 速度KP（仅允许基线10和唯一候选12），不置零、不保存参数；没有参数接受确认。

每次使用独占新目录，保存 `run.json`（请求参数、计划、初始位姿、失能与关闭结果）、`samples.jsonl`（主机时间、输入、SDK 指令、反馈及保护状态）、`serial.log`（实际 TX/RX）。电流、母线电压、直接速度反馈未测。清理分别尝试两轴失能及关闭，沿用驱动的帧截止时间；写成功仅代表请求已发送，不能证明物理停止。实验二分支A复用同一计划，在实际使用负载下分别以 `--pitch-speed-kp 10` 和 `--pitch-speed-kp 12` 写入不同目录；两次命令均须事先人工确认；前次异常或清理失败则停止整组，不继续候选。实验二不执行输入死区分支，也不追加候选。

界面启动时连接电机，初始未使能、扭矩状态未知；“使能并到零位”会运动。

**滑条是实时输入流**：使能后界面每个周期（约 33 ms）调用一次 `neck.set_target(滑条值)`，和上游（Quest）跟随走同一条路径。“暂停输入”停止发送，约 0.2 s 后进入 `input_idle`（保持位置），再点恢复，用来验证输入过期保护。

**写零位**：先用滑条把两轴调到理想位置，等它保持稳定，再点“写零位（两轴）”并确认；也可以失能后用手拨到位再写。两轴一起写入并保存到 flash，写完两轴读数应为 0°，滑条同步回 0。点击后界面先暂停输入再写；如果还没稳定（指令与实测差 > 1°）会提示稍后再点。代码里是 `neck.zero(save=True)`。第一次真机使用请先偏离零位约 2° 试。

日志写入 `logs/`（原始串口收发 `.log`，关闭 `.close.json`）。

## 看懂它是怎么搭的

- [`docs/简化架构.md`](docs/简化架构.md)：当前主路径的模块、100 Hz 循环、失败策略与置零事务。
- [`docs/架构图.md`](docs/架构图.md)：旧 `neck.driver` / `neck.control` 的详细图，保留参考。
- 按需生成回放：`python3 tools/replay.py`（写入 `docs/回放.html`，不需要硬件）。

## 默认参数

| 项 | yaw（ID 1） | pitch（ID 2） |
| --- | --- | --- |
| 速度 PI / 位置 PI | 40/4、42/0 | 10/4、40/0 |
| 电机速度 / 加速度 | 45 RPM / 2 | 15 RPM / 2 |
| 软件限位 | ±60° | −45°…43° |
| 软件速度 / 加速度 | 90°/s / 1440°/s² | 90°/s / 1440°/s² |

起步采用限加速度的二阶平滑跟随（响应 20 Hz），最大 90°/s、1440°/s²；输出按 0.1° 协议量化，不再叠加 1€ 低通、死区或制动距离求解。2026-11-10 台架 A/B 中 yaw 45 RPM 实测峰值约 91°/s、0.5 s 持续约 88°/s，pitch 保持 15 RPM。

需要解除软件速度/加速度钳位时，可设 `NeckConfig(max_speed=None, max_accel=None, max_lead=None)`；二阶跟随仍保留，机械限位仍生效。先小角度试，详见 [`docs/使用说明_0.6.0.md`](docs/使用说明_0.6.0.md)。

**2026-11-10 台架 A/B：yaw 15→30→45 RPM 时实测峰值约 48→91→91°/s，0.5 s 持续约 41→88→88°/s；pitch 保持 15 RPM。直通模式下速度/加速度的完整行为仍需更多工况验证**；其余代码和长期工况仍需实机验收。

## 测试

```bash
python3 -m unittest discover -s tests -t .      # 不需要硬件，也不需要 pytest；UI 测试用 Qt offscreen
```

## 目录

```
neck/            protocol.py link.py neck.py types.py    当前简化核心
neck/driver/     device.py motor.py port.py protocol.py types.py   旧层 1（实验工具）
neck/control/    neck.py trajectory.py supervisor.py               旧层 2（实验工具）
tools/           ui.py tune.py load_compare.py replay.py            操作员工具
tests/           fake_link.py bus.py fakes.py 及各层测试
docs/            简化架构.md、使用说明_0.6.0.md、设计说明.md、架构图.md、电机手册 PDF、archive/
agent.md         智能体角色、现场参数与变更记录
```

旧的 C++ SDK / ZED 相机 / `f32c` Python 包 / wheel 已删除，可通过 git tag `pre-refactor` 找回。
