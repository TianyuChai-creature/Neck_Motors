# Neck_Motors

F32C 双轴云台（颈部）控制。两台电机共用一条 TTL 串口总线，默认 **yaw = ID 1**、**pitch = ID 2**。

纯 Python 标准库，分两层：`neck.driver` 让两台电机听话并如实上报角度，`neck.control` 把“看向哪里”变成安全的角度序列并分级保护。设计与状态机见 [`docs/设计说明.md`](docs/设计说明.md)。

## 准备

- Linux、Python ≥ 3.10。界面工具另需 PySide6（`pip install -r tools/requirements.txt`）。
- 按[电机手册](docs/F32C无刷电机使用手册-2026-07-21.pdf)接线：电机供电 **8–15 V**，串口 **3.3 V TTL**，USB-TTL 与电机电源共地，**115200 baud**。
- 两台电机 ID 分别为 1、2；零位存在电机 flash 里；可在界面里把当前位置“写零位”（两轴一起，不可撤销，见下）。首次使能前确认当前姿态附近有足够活动空间。
- 当前用户需有串口读写权限。不指定串口时会扫描 `/dev/serial/by-id/*`、`/dev/ttyACM*`、`/dev/ttyUSB*`，只发查询，选中两个 ID 都应答的那个。

## 最小用法

```python
from neck.driver import Driver
from neck.control import Neck

with Neck(Driver.open()) as neck:      # 退出时自动失能并关闭串口
    neck.enable()                      # 以实测位姿使能，然后平滑回已存零位 (0, 0)
    while running:
        neck.set_target(yaw, pitch)    # 最新目标胜出；默认限位 yaw ±60°，pitch −180°…45°
        snap = neck.tick()             # 同一线程，约 20–33 ms 一次；返回 Snapshot
```

`snap.phase`：`off / moving / following / hold / stale / degraded / fault`；`snap.level`：0 正常、1 扰动、2 降级、3 故障。故障时 `tick()` 已失能并抛 `NeckError`，恢复需显式 `neck.reset_fault()`。只用底层时见 `neck.driver.Driver`（`enable/set/read/disable/close/state`）。

## 工具

```bash
python3 tools/ui.py [/dev/ttyACM0]         # 面板：使能并到零位 / 失能 / 到 0° / yaw、pitch 滑条
python3 tools/tune.py --axis pitch --speed-kp 10 --speed-ki 4 --pos-kp 40 --pos-ki 0   # 默认仅预览，加 --enable 并输入 ENABLE 才通电
```

界面启动时连接电机，初始失能；“使能并到零位”会运动。

**写零位**：先用滑条把两轴调到理想位置，等它保持稳定，再点“写零位（两轴）”并确认；也可以失能后用手拨到位再写。两轴一起写入并保存到 flash，写完两轴读数应为 0°，滑条同步回 0。代码里是 `neck.set_zero()`（只在失能或保持稳定时可用）。第一次真机使用请先偏离零位约 2° 试。日志写入 `logs/`（原始串口收发 `.log`，故障 `.fault.json`，关闭 `.close.json`）。

## 看懂它是怎么搭的

- [`docs/回放.html`](docs/回放.html)：**双击用浏览器打开**。一次完整运行的回放（使能回零、跟随、过期、反馈中断降级、写零位、故障），同步显示角度曲线、phase/level 色条、层间数据流和每一帧串口字节。
- [`docs/架构图.md`](docs/架构图.md)：分层依赖、tick 时序、使能、写零位、状态机、保护分级（Mermaid，GitHub/Notion 直接渲染）。
- 重新生成回放：`python3 tools/replay.py`（约 13 秒，不需要硬件）。

## 默认参数

| 项 | yaw（ID 1） | pitch（ID 2） |
| --- | --- | --- |
| 速度 PI / 位置 PI | 40/4、42/0 | 10/4、40/0 |
| 电机速度 / 加速度 | 15 RPM / 2 | 15 RPM / 2 |
| 软件限位 | ±60° | −180°…45° |
| 软件速度 / 加速度 | 90°/s / 720°/s² | 90°/s / 720°/s² |

**提速（15 RPM、90°/s）与直通模式下速度/加速度是否生效尚未实机验证**；所有代码目前只做过离线测试（pty 假总线 + FakeDriver）。

## 测试

```bash
python3 -m unittest discover -s tests -t .      # 不需要硬件，也不需要 pytest；UI 测试用 Qt offscreen
```

## 目录

```
neck/driver/   protocol.py port.py motor.py device.py types.py     层 1：硬件
neck/control/  trajectory.py supervisor.py neck.py                 层 2：安全运动
tools/         ui.py tune.py replay.py requirements.txt            操作员工具（ui/tune 只依赖 neck）
tests/         假总线 bus.py、FakeDriver fakes.py、各层测试
docs/          设计说明.md、架构图.md、回放.html、电机手册 PDF、archive/（旧版记录）
agent.md       智能体角色、现场参数与变更记录
```

旧的 C++ SDK / ZED 相机 / `f32c` Python 包 / wheel 已删除，可通过 git tag `pre-refactor` 找回。
