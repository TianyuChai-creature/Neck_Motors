# F32C Neck Motors SDK 0.3.2 使用说明

## 1. 发布内容与安装

发布文件为 `wheel/f32c_neck_motors-0.3.2-py3-none-linux_x86_64.whl`，本说明同时复制至 `wheel/SDK使用说明.md`。轮子包含 Python 包 `f32c`（含 `HeadTracker`）、电机共享库 `libf32c.so`、电机 C/C++ 公开头文件、CMake 包配置和本说明。本 SDK 负责电机通信、参数设置和跟随目标生成。Python >=3.10；平台为 Linux x86_64，原生库需要兼容的 glibc/libstdc++。

```bash
python3 -m pip install --upgrade wheel/f32c_neck_motors-0.3.2-py3-none-linux_x86_64.whl
python3 -c 'import f32c; from importlib.metadata import version; print(version("f32c-neck-motors")); print(f32c.native_prefix())'
python3 -c 'from f32c import HeadTracker, TrackingConfig, TrackingError; print("HeadTracker API available")'
```

`native_prefix() -> pathlib.Path` 返回已安装包内的 `native/` 目录，可用于定位头文件、库和 CMake 包。正常安装不需要设置 `F32C_LIBRARY_DIR`；这个环境变量只用于开发时覆盖库目录，指向旧库时新 API 可能不可用。

0.2.0 将发布内容收敛为电机 SDK，保留 `HeadTracker`、`TrackingConfig`、`TrackingError` 和运行时 `set_pi()`，沿用原有电机配置结构和默认 PI。图像采集相关的 Python 接口和原生组件不再提供，输入数据由上层应用管理。调参工具与示例位于源码仓库 `examples/`，不随 whl 安装。

0.3.0 新增同步 `HeadController`：统一使用 `HeadTracker` 生成参考，并负责反馈读取、参考超前限制、误差/无进展/保持振荡保护和故障清理。UI 和集成应用调用同一入口，无后台线程。原有 `Gimbal2Dof` 和纯软件 `HeadTracker` 保持可用；低层 API 不自动加入控制策略。0.3.1 将高层使能后的默认目标设为 yaw=0°、pitch=35°，通过 `DEFAULT_HOLD_POSITION` 导出。**未发布改动**：`DEFAULT_HOLD_POSITION` 改为已存零位 yaw=0°、pitch=0°；`HeadController` 默认参考上限 90°/s、720°/s²；保护改为分级（见 §4.2）；`Config.travel_rpm` 默认 15、`accel` 默认 2。这些改动仅经无硬件测试，未实机验证。0.3.2进一步统一实测电机参数和集成入口默认值。固定目标不等于实际到位或机械锁止，应以回读验证。

## 2. 现场方向、目标与控制语义

零位由用户手动写入，SDK 不自动标定。yaw 增大为第一人称向左；pitch 增大为低头。上游 Quest 角度的反号、比例、身份和授权由集成应用负责。

`HeadController` 将**新期望 pitch 目标**截断到 +45°（配置的目标上限更小时采用较小值），没有 40°/43°反馈故障边界。初始实测即使超过 45°，也保留实测坐标，不因此失能、不改零位；`HeadController.enable()` 默认从实测生成到已存零位 yaw=0°、pitch=0°的轨迹，持续调用 `tick()` 推进，其初始中间参考可能仍大于 45°。整定工具的自动试验计划要求位置及阶跃均不超过 45°，避免把截断后的动作记成原计划通过；这不用于诊断反馈角度。

低层 `Gimbal2Dof.set_deg()` 仍立即下发，不裁剪到 45°或生成轨迹。单圈角度按协议 0.1°分辨率量化并对 360°取模。低层 `read()` 保留原始反馈；`HeadController` 将其转换到有符号坐标。两轴在同一串口串行读写，不是同步采样。同一控制实例由一个线程串行访问。

### 2.1 控制职责

| 模块 | 职责 |
| --- | --- |
| `Gimbal2Dof` / C++ / C | 原始串口协议、PI、命令、两轴反馈、有界写入及关闭 |
| `HeadTracker` | 无 I/O 的滤波、速度/加速度轨迹、量化滞环 |
| `HeadController` | 组合以上两层；持续反馈、目标裁剪、发送权限复核、反馈保护、显式回零/到位、故障清理 |
| 集成应用 | 输入源及时间戳、Quest 身份/授权、设备进程锁、操作员确认和恢复、调用周期 |
| 固件与供电 | 电流/力矩限制及最终断能；SDK 没有电流控制或物理停止确认接口 |

### 2.2 写入和关闭

普通命令写入使用 100 ms 单调时钟总截止时间，覆盖短写、EINTR/EAGAIN 和零写，不因部分进展重置。失败向上报告；成功只表示串口写入，没有命令 ACK。原有命令后 8 ms 间隔保留，不宣称整轮稳定 50 Hz。

`disable()` 尝试两轴，失败保留待清理状态并停止位置/PI 下发。C/C++/Python 的关闭现在都仅对本实例已使能或待清理状态发送一轮两轴失能请求，再释放串口；被动 open/read/close 不发送失能。单轮关闭串口写入等待预算至多 216 ms，另有调度及日志开销，不是实时急停保证。之前显式失能失败时，关闭会再尝试这一轮。

关闭失败仍释放句柄，错误必须立即记录，不能复用旧句柄。`HeadController` 保留故障和清理失败，清理失败后禁止仅靠 reset 自动恢复。请求发送成功也不等于硬件已经停止。

## 3. Python 配置与数据类型

```python
from f32c import Config, F32CError, Gimbal2Dof

with Gimbal2Dof(Config(device="/dev/ttyACM0")) as motor:
    print(motor.read())
```

此例不使能、不下发位置；退出时不发送失能命令。不要与另一个控制进程同时占用该总线。

### Config

`Config(...)` 是数据类，在 `open()` 时提交配置；使能时下发速度、加速度和两轴 PI。

| 字段 | 默认值 | 含义 / 取值 |
| --- | --- | --- |
| `device` | `None` | 串口路径；空值扫描 `/dev/serial/by-id/*`、`ttyACM*`、`ttyUSB*`，要求两轴回应 |
| `baud` | `115200` | 支持 9600、57600、115200，8N1 |
| `id_yaw`, `id_pitch` | `1`, `2` | 地址 1～255，必须不同 |
| `travel_rpm` | `15` | 目标速度参数，0～1000 RPM；不等同于主机轨迹速度 |
| `accel` | `2` | 加速度参数，0～65535，协议单位转/s² |
| `yaw_speed_kp`, `yaw_speed_ki` | `40`, `4` | yaw 速度环 PI，整数 0～256 |
| `yaw_pos_kp`, `yaw_pos_ki` | `42`, `0` | yaw 位置环 PI，整数 0～256 |
| `pitch_speed_kp`, `pitch_speed_ki` | `10`, `4` | pitch 速度环 PI，整数 0～256 |
| `pitch_pos_kp`, `pitch_pos_ki` | `40`, `0` | pitch 位置环 PI，整数 0～256 |

0.2.2 的高增益组合在实际 UI 使能时振荡，已撤回。0.3.2 将已用于实际回零及10秒保持的参数固化为 C++/C/Python 默认值：yaw 速度 PI=40/4、位置 PI=42/0，pitch 速度 PI=10/4、位置 PI=40/0，5 RPM、加速度1转/s²。同一配置下分段回零、35°直接回零及直接使能到0°均完成；一次0°保持10秒的 yaw 范围−0.2°～−0.1°、pitch范围−0.3°～−0.1°。这些实测使用低层 Gimbal2Dof，不能据此宣称 Quest 连续跟随已通过硬件验收。HeadController/UI 默认参考上限30°/s、360°/s²，分别按5 RPM和1转/s²换算，是保守软件约束而非新增实测结果。0.2.5 增加按实际反馈暂停参考推进的处理，通过两轮 pitch=0° 时 yaw 的 0°→+60°→−60°→0° 实机动作，以及每 0.2 秒改一次 yaw 目标的连续拖动模拟；固定保持末段两轴峰峰值最大约 0.1°、均值误差绝对值最大约 0.1°。此前 yaw/pitch ±15° 双向切换末段峰峰值最大约 0.4°。这些记录不代表所有 pitch 姿态、外力、相机并行和重新上电均已验证。0.3.0 的故障路径检查为无硬件测试，不新增实机姿态验收。`set_pi()` 更新当前实例，不修改全局默认值。

### 参数设置方式

| 需要设置的参数 | 公开接口 | 生效时机 |
| --- | --- | --- |
| 电机连接、ID、初始 PI、速度和加速度 | `Config(...)` → `Gimbal2Dof(config)` | `open()` 复制配置到原生实例；`enable()` 下发控制配置 |
| 单轴四项电机 PI | `motor.set_pi(axis, speed_kp, speed_ki, pos_kp, pos_ki)` | 已打开且已使能时立即串行下发，临时生效 |
| 跟随滤波、防抖、轨迹速度/加速度和角度范围 | `TrackingConfig(...)` → `HeadTracker(initial, config)` | 创建跟随生成器时生效；详见 §4.1 |

初始配置示例（数值沿用 SDK 默认值，仅演示设置方法）：

```python
from f32c import Config, Gimbal2Dof

motor_config = Config(
    device="/dev/ttyACM0",
    travel_rpm=15,
    accel=2,
    yaw_speed_kp=40, yaw_speed_ki=4, yaw_pos_kp=42, yaw_pos_ki=0,
    pitch_speed_kp=10, pitch_speed_ki=4, pitch_pos_kp=40, pitch_pos_ki=0,
)
motor = Gimbal2Dof(motor_config)  # 此处不打开串口、不使能。
```

`Config` 的 ID、速度、加速度和 PI 数值应传整数，不能传浮点数或字符串。`open()` 后直接修改 `motor.config` **不会自动下发**，也不会同步修改原生实例已经复制的配置。运行时 PI 必须调用 `set_pi()`；`travel_rpm`、`accel`、地址等目前没有运行时 setter，需要关闭连接并使用新的 `Config` 创建新实例。`TrackingConfig` 只改变软件生成的目标，不修改电机 PI 或固件参数。

| 类型 | 字段 / 用途 |
| --- | --- |
| `State` | 不可变数据类：`yaw_deg: float`、`pitch_deg: float`，来自角度反馈，单位度 |
| `F32CError` | 库加载、连接或原生操作失败时的异常，继承 `RuntimeError` |

0.2.3 的 `enable()` 在使能前查询保持位置，先向两轴发送失能和完整 PI 配置，再逐轴执行使能→单圈直通模式→速度→初始目标→加速度，避免在已使能期间逐项初始化 PI。参数写入没有 ACK，不能将下发视作回读验证。`enable_at()` 指定初始目标，已经使能时直接更新目标；低层 SDK 不自动回零或执行本装置的目标上限。

## 4. Python 电机 API

`Gimbal2Dof(config: Config | None = None)` 创建对象并加载电机库，不打开串口、不使能。配置省略时使用新的默认 `Config()`。

| 函数 / 签名 | 参数 | 返回值与行为 |
| --- | --- | --- |
| `open() -> Gimbal2Dof` | 无 | 连接并检测两轴，返回自身；已打开则直接返回；失败抛 `F32CError` |
| `close() -> None` | 无 | 0.3.0 使用可检查的关闭；完成清理后返回，清理失败抛 `F32CError`，同时仍释放句柄；重复关闭已关闭对象无操作 |
| `ping() -> bool` | 无 | 查询两轴是否回应；查询失败返回 `False`，可读 `last_error()`；未打开则抛异常 |
| `enable() -> None` | 无 | 使能两轴，选择单圈直通模式，应用实例配置并保持当前角度；不自动回零；已使能时不重复初始化 |
| `enable_at(yaw_deg: float, pitch_deg: float) -> None` | 两个有限角度，单位度 | 0.2.1 新增。0.2.3 先失能下发 PI，再使能并锁定这两个初始目标；目标立即下发，不平滑、不限制机械范围。已经使能时相当于 `set_deg()`，不重复初始化；无效角度抛 `F32CError` |
| `disable() -> None` | 无 | 尝试两轴失能；失败抛 `F32CError` 并保留待清理状态、暂停位置/PI 下发；关闭时继续清理。写成功不代表收到电机确认 |
| `read(*, timeout_s=None) -> State` | 省略时两轴总等待预算 100 ms；显式值 0 < timeout_s ≤ 1 | 两轴共用截止时间，任一轴失败不返回部分 State；预算包含查询写入和重试，另有帧间隔及调度开销 |
| `set_deg(yaw_deg: float, pitch_deg: float) -> None` | 两个有限角度，单位度 | 依次下发 yaw、pitch；要求已使能；不平滑、不限机械角度 |
| `set_yaw_deg(deg: float) -> None` | 有限角度 | 只下发 yaw 目标；要求已使能 |
| `set_pitch_deg(deg: float) -> None` | 有限角度 | 只下发 pitch 目标；要求已使能 |
| `set_pi(axis: str, speed_kp: int, speed_ki: int, pos_kp: int, pos_ki: int) -> None` | `axis` 为 `"yaw"` 或 `"pitch"`；四个整数均在 0～256 | 0.1.1 新增。已使能时更新该轴四个 PI，保留模式、目标和使能状态；不保存参数。四帧串行发送，不是原子更新；错误时抛异常 |
| `set_zero() -> None` | 无 | 独立标定接口，要求已使能。把两轴当前位置设为零点并分别保存到电机；宿主必须暂停旧坐标目标生产，并在置零后回读/重建轨迹。UI 不再提供此入口 |
| `start_log(path) -> None` | 路径类型 str 或 os.PathLike；父目录需已存在 | 打开原始串口日志，采用追加写入；记录 TX/RX/FLUSH/TX_ERROR；失败抛异常 |
| `stop_log() -> None` | 无 | 停止原始日志；要求连接已打开 |
| `last_error() -> str` | 无 | 返回底层最近错误；打开或关闭失败后查询当前线程的最近全局错误，应立即复制 |

`with Gimbal2Dof(...) as motor` 相当于进入时 `open()`、退出时 `close()`。除 `ping()` 的查询失败外，操作失败通常抛 `F32CError`；配置或 `set_pi()` 参数校验失败抛 `ValueError`。参数更新中途失败时，电机可能只接受了部分 PI，应终止该次实验并失能，不将它计入已验证配置。

`close()` 的清理异常不会被吞掉；若控制代码先有异常，退出时的清理失败会保留原异常作为异常上下文。显式关闭时应捕获并记录；析构兜底会向 stderr 报告失败，不能以对象销毁推断物理失能。

运行时 PI 接口要求 `axis` 精确为 `"yaw"` 或 `"pitch"`，四项增益都必须显式提供，且为 0～256 的 Python `int`；浮点数、字符串和 `bool` 均拒绝。此接口不自动使能，不切换控制模式，不改变当前目标，不发送持久保存命令。

运行时调整示例（会使能；这里的数值仅演示 API）：

```python
from f32c import Config, Gimbal2Dof

with Gimbal2Dof(Config(device="/dev/ttyACM0")) as motor:
    if input("确认使能并保持当前位置，输入 ENABLE: ") != "ENABLE":
        raise SystemExit
    try:
        motor.enable()
        motor.set_pi("yaw", speed_kp=40, speed_ki=0, pos_kp=42, pos_ki=0)
        motor.set_pi("pitch", speed_kp=10, speed_ki=4, pos_kp=40, pos_ki=0)
        print(motor.read())
    finally:
        motor.disable()
```

该例不改零点、不调用持久保存，也不自动移到 0°。运行期间参数仍需按实际负载验证。`set_zero()` 会发送保存参数命令，不应在未验收的 PI 试验期间调用，以免保存当前参数。

### 4.1 Python 头部跟随控制层

`HeadTracker` 在 `set_deg()` 上方生成受约束的目标，不改变原接口的立即下发语义，不加载原生库、不使能、不调整 PI，也不写串口。安装 0.2.0 whl 后即可导入；旧版 0.1.1 whl 不包含此模块。

该层使用速度自适应低通滤波：静止时抑制小幅输入噪声，运动时提高截止频率以减少滞后；随后生成受速度、加速度约束的阻尼轨迹，并用 0.1° 输出滞回避免量化边界反复跳动。连续轨迹保留浮点精度，慢速转头不会因为每帧变化小于一格而被不断舍弃。滤波原理参考 [1€ Filter 作者说明](https://gery.casiez.net/1euro/)，实现不依赖第三方库。

```python
from f32c import HeadTracker, State, TrackingConfig

# 这两个范围必须由调用方提供实际标定结果，不能照抄模拟示例。
settings = TrackingConfig(
    yaw_limits=calibrated_yaw_limits,
    pitch_limits=calibrated_pitch_limits,
    max_speed_deg_s=180.0,
    max_accel_deg_s2=1800.0,
    response_hz=8.0,
    min_cutoff_hz=3.0,
    beta=0.15,
    derivative_cutoff_hz=1.0,
    deadband_deg=0.05,
    hysteresis_deg=0.02,
    stale_after_s=0.2,
    max_gap_s=0.1,
)
tracker = HeadTracker(initial=motor.read(), config=settings)
```

坐标沿用电机原始方向，单位为度。头部传感器的反号、零位偏置和幅度映射由调用方先完成。初始化/`reset()` 的 `State` 可来自原生 0～360° 反馈，内部转为有符号角度；目标输入必须使用标定后的有符号坐标。pitch 始终走有符号路径；提供 `yaw_limits` 时 yaw 同样不绕圈，省略时才采用最短循环路径。超界目标抛出 `ValueError`，不会静默裁剪成另一个目标。

| 接口 | 行为 |
| --- | --- |
| `HeadTracker(initial: State, config: TrackingConfig, *, timestamp=None)` | 以当前反馈或模拟姿态初始化；不自动回零 |
| `set_target(yaw_deg, pitch_deg, *, timestamp=None)` | 更新最新目标，不排队、不推进轨迹、不下发命令 |
| `step(*, timestamp=None) -> State` | 按经过的时间推进轨迹，返回 0.1° 的生成目标；需要定时调用 |
| `reset(initial: State, *, timestamp=None)` | 用新反馈重新同步，速度归零，清除控制循环超时状态 |
| `config` | 当前只读的 `TrackingConfig`，不可在运行中替换 |
| `target` / `filtered_target` | 最新接受的输入 / 经滤波的输入，均为生成过程的诊断值 |
| `command` / `continuous_command` | 量化目标 / 量化前连续轨迹，不是实测角度 |
| `velocity_deg_s` | 连续轨迹的 yaw/pitch 速度元组，不是编码器速度 |
| `stale` / `faulted` | 输入已超时 / 控制循环间隔过长而需要重新同步 |

所有调用使用同一线程和同一单调时钟。默认取 `time.monotonic()`；显式 `timestamp` 也必须来自同一时钟域，不能直接传网络设备时间戳。参数和时间异常不推进轨迹。控制循环长时间暂停后，`TrackingError` 会锁住生成器，直至用新反馈 `reset()`；这避免将积累的时间一次性推进成大幅运动。

配置不可在运行中替换；改变标定范围或轨迹参数时，暂停下发，创建新配置并以新反馈构建生成器，避免旧的路径状态沿用新的边界。可使用数据类的 `replace()`，例如：

```python
from dataclasses import replace
from f32c import HeadTracker

# 在同一个电机执行线程、暂停跟随循环后执行。
settings = replace(settings, min_cutoff_hz=2.5, beta=0.2)
tracker = HeadTracker(motor.read(), settings)
# 重新进入循环时将 last_sent 置为 None；不沿用旧生成器的目标和速度。
```

`TrackingConfig` 的默认值是软件起点，速度/加速度上限仍需在当前负载下验证：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `pitch_limits` | 必填 | 实际标定的有符号范围；应留出机械余量 |
| `yaw_limits` | `None` | 有限范围时走有符号路径；`None` 表示循环轴 |
| `max_speed_deg_s` / `max_accel_deg_s2` | `180` / `1800` | 生成的连续轨迹速度和加速度上限 |
| `response_hz` | `8` | 轨迹响应参数，0～20 Hz 内的正值；不是电机闭环带宽 |
| `min_cutoff_hz` / `beta` | `3` / `0.15` | 滤波最低截止频率 / 随角速度增加截止频率的系数 |
| `derivative_cutoff_hz` | `1` | 输入角速度估计的滤波截止频率 |
| `deadband_deg` / `hysteresis_deg` | `0.05` / `0.02` | 锚定目标的噪声门限 / 0.1° 输出格的附加滞回 |
| `stale_after_s` | `0.2` | 无新目标后开始制动，再保持生成的停止位置 |
| `max_gap_s` | `0.1` | 单次控制周期最长间隔；超过后须重新读取反馈并同步 |

截止频率、响应参数、速度/加速度上限和超时时间必须为有限正数；`beta`、死区及滞回为有限非负数，均不能传 `bool` 或字符串。另有约束：`response_hz ≤ 20`、`max_gap_s ≤ 0.1`、`max_speed_deg_s × max_gap_s ≤ 179`、`max_accel_deg_s2 ≤ 1e6`、`deadband_deg ≤ 0.5`、`hysteresis_deg < 0.05`。无效配置抛 `ValueError`。

降低 `min_cutoff_hz` 会加强静止滤波，也会增加小动作的滞后；增大 `beta` 会在快速转头时提高滤波响应。增大死区或滞回会减少微小目标更新，也会增加残余指令误差；增大 `response_hz` 或运动上限会更快跟随，但不能据此认定机械负载更稳定。应先保留默认值，并按实际输入噪声、跟随延迟和负载验证结果调整。

范围必须落在 −180°～180° 内，且包含至少一个 0.1° 可表示目标。不能把未知的机械边界当作已标定范围。输入超时制动不等于失能；重负载失能可能导致头部下落，是否失能由应用的退出/故障流程管理。生成的连续轨迹受速度和加速度约束，离散角度帧的差分及真实电机运动不等同于这些诊断值。

典型集成方式如下。调用方提供标定范围和头部角度来源；`latest_head_angles()` 是应用提供的非阻塞函数，仅在有**新样本**时返回映射后的 `(yaw, pitch)`，否则返回 `None`，不应积压旧姿态。反复把同一旧样本当作新样本提交，会使超时检测失效。

### 4.2 推荐硬件入口：HeadController

```python
import time
from f32c import Config, HeadController

with HeadController(Config(device="/dev/ttyACM0")) as head:
    # 仅在应用已取得操作员使能许可后调用。
    head.enable()                 # 默认从实测位置前往已存零位 yaw=0°、pitch=0°；不写零点。
    while head.phase == "moving":
        snapshot = head.tick()   # 以实测到位并稳定 0.5 s 判断完成。
        time.sleep(1 / 30)
```

默认目标由 `f32c.DEFAULT_HOLD_POSITION` 给出。`enable()` 只在从失能进入使能时建立此目标，重复调用不会覆盖已有目标。到位后应持续 `tick()` 保持反馈监测。自定义目标范围仍会裁剪默认目标。低层 `Gimbal2Dof.enable()` 继续原位保持。

仓库提供固定目标保持窗口：安装此 whl 和 `requirements-ui.txt` 后，运行 `python3 examples/hold_position.py /dev/ttyACM0`。它只调用低层 `enable_at(0, 0)`，不走回零流程，关闭窗口或点击“解锁并退出”会发送两轴失能请求。窗口直接采用0.3.2的 `Config()` 默认配置，不再维护另一份PI常量。窗口串行读取两轴角度和电压，电压超出8～15 V或通信异常时停止保持并请求失能。它会向固定目标运动，不置零、不保存参数，也不保证外力下完全无偏移。

`HeadController()` 不传自定义 tracking 时，默认 yaw 目标范围−60°～60°、pitch −180°～45°、软件速度上限90°/s、加速度720°/s²（`response_hz=15`、`min_cutoff_hz=6`）；电机侧 `travel_rpm=15`（约90°/s）、`accel=2`（720°/s²）与之对应。独立的 `TrackingConfig`/`HeadTracker` 保留通用算法默认值；显式传入自定义 `TrackingConfig` 会覆盖上述控制器基线，不应继续沿用主仓旧的高速配置。

Quest 集成见同目录[Quest集成说明](Quest集成说明.md)，重点是单线程设备所有权、只提交新样本、持续 tick、输入失效和坐标映射。

跟随模式仅在收到**新输入**时调用 `head.set_target(yaw, pitch, received_at=接收时单调时钟)`；持续调用 `head.tick(may_send=权限回调)`，回调必须无副作用、快速返回。SDK 在反馈之后及每轴发送前复核许可，撤销时停止新目标输出，不发送额外制动轨迹。`head.hold()` 同样停止新参考，仍读取反馈和检查误差。失联恢复应由宿主确认身份/授权，再提交新样本，轨迹从新反馈建立。

`set_target` 不得重复使用旧包来刷新时间；未来或倒退时间戳拒绝，过期输入进入 HOLD。`move_to` 是独立点到点动作，不受网络输入过期影响。`set_motion_limits(speed, accel)` 更新软件轨迹约束。`snapshot` 包含 phase、enabled、measured、command、target、feedback_age_s、reason 和 following_limited 和 level（0 正常、1 扰动、2 降级、3 故障）。command 仅表示成功写出的参考，measured 才表示事实。

`HeadController` 采用**分级保护**，`snapshot.level` 给出当前级别，`reason` 给出原因：

| 级别 | 含义 | 触发（默认值） | 处理 |
| --- | --- | --- | --- |
| 0 正常 | — | — | 按轨迹推进 |
| 1 扰动 | 短暂、可自愈 | 单次反馈读失败且反馈龄 <0.1 s；`tick()` 间隔 >`max_gap_s` 但 <1 s；保持时摆幅 >1° | **不失能、不改相位**：本周期暂停推进，保持上一条指令，轨迹从当前指令重新同步，随后继续。计入事件计数 |
| 2 降级 | 持续异常，但仍可能自行恢复 | 反馈龄 ≥0.1 s（<0.5 s）；持续 1.5 s 误差 >6° 且实测变化 ≤0.3°（无进展） | **保持使能**，进入 `degraded` 相位，停止推进；无进展时把指令**一次性拉回实测位姿**，让电机不再顶着阻力；误差 <3° 并稳定 0.3 s 后自动回到原相位（`reason=recovered`），期间到来的新目标会被记住 |
| 3 严重 | 无法继续安全运行 | 反馈丢失 ≥0.5 s；`tick()` 间隔 ≥1 s；降级持续 ≥5 s；30 s 内 1/2 级事件累计 ≥6 次（`repeated_disturbance`）；目标写失败；反馈非法；点到点超时；轨迹异常 | 停止下发并请求两轴失能，抛 `HeadControlError`，需显式 `reset_fault()` |

`ProtectionConfig` 其它默认：每 tick 反馈等待预算 80 ms、参考超前 18°等待（误差回到 3°以内清除等待标记）；已删除仅按跟随误差大小触发失能的判断，`max_error_deg` 为兼容旧调用保留、不再控制失能。点到点动作默认 15 s 截止（降级期间不消耗），实测及参考都接近目标并连续稳定 0.5 s 才结束。新增字段：`feedback_lost_s`、`control_gap_fault_s`、`degraded_timeout_s`、`recover_time_s`、`escalation_window_s`、`escalation_events`。分级只调整**何时**失能，不改变“写成功不等于电机确认”的约定。

`HeadControlError` 包含 reason、snapshot 和 cleanup_error；故障停止推进并尝试失能，不自动重新使能。成功清理后需显式 `reset_fault()` 才能再次 enable；关闭失败按硬件状态未知处理。宿主仍应在同一所有权锁内关闭对象并保存错误，不以 enabled=False 推断物理失能。

只安装 whl、没有电机也可以运行下面的纯软件示例；其中范围仅用于模拟：

```python
from f32c import HeadTracker, State, TrackingConfig

tracker = HeadTracker(State(0, 0), TrackingConfig(pitch_limits=(-30, 30)), timestamp=0)
for tick in range(1, 101):
    now = tick * 0.02
    tracker.set_target(10, -5, timestamp=now)
    command = tracker.step(timestamp=now)
print(command)  # 生成目标接近 State(yaw_deg=10, pitch_deg=-5)，没有串口操作。
```

可以在无硬件电脑运行：

```bash
python3 tests/test_tracking.py
python3 examples/tracking_demo.py
python3 examples/tracking_demo.py --csv /tmp/head_tracking_demo.csv
```

CSV 使用独占创建，重复文件名会报错。模拟分别记录原始输入、滤波输入和生成目标；控制层的软件滞后、静止指令摆幅和最终指令误差不代表机械或画面性能。首次无硬件验证中，±0.06° 均匀静止噪声未造成目标跳动，60°/s 转头示例的生成轨迹滞后约 48 ms；实际负载仍需检查静止负载振动、停稳后 ≤1° 误差、正常转头的跟随延迟，以及边界和失联行为。没有真机验收时不能承诺电机绝对不抖。

## 5. C++ / C 电机 API

### CMake 使用

```bash
SDK_PREFIX=$(python3 -c 'import f32c; print(f32c.native_prefix())')
cmake -S your_project -B your_project/build -DCMAKE_PREFIX_PATH="$SDK_PREFIX"
```

```cmake
find_package(f32c CONFIG REQUIRED)
target_link_libraries(your_app PRIVATE f32c::f32c)
```

### C++：`#include <f32c/gimbal.hpp>`

`f32c::Gimbal2Dof()` 使用默认配置；`Gimbal2Dof(Config cfg)` 使用自定义配置。`Config` 字段与 Python 对应：`id_yaw/id_pitch` 为 `uint8_t`，PI 和加速度为 `uint16_t`；构造前应检查宽整型数值，避免转换截断。对象不可复制。

| 函数 | 返回值 / 与 Python 的区别 |
| --- | --- |
| `open()` | `bool`；显式路径只打开串口，自动扫描时检查两轴回应；需用 `ping()` 检测显式路径的两轴 |
| `close()` | 保留 `void`；调用 `close_checked()`，失败向 stderr 报告，并保留 `last_error()` |
| `close_checked()` | 0.3.0 新增 `bool`；清理已使能/待清理状态后释放串口，即使失败也释放；失败返回 false 并保留未知状态。true 不等于电机已停止 |
| `ping()`、`enable()`、`disable()` | `bool`；失败返回 `false`，从 `last_error()` 取原因 |
| `enable_at(double yaw, double pitch)` | `bool`；显式启动目标，有限数校验，行为同 Python；失败从 `last_error()` 取原因 |
| `set_deg(double yaw, double pitch)`、`set_yaw_deg(double)`、`set_pitch_deg(double)` | `bool`；角度语义同 Python |
| `set_pi(unsigned axis, unsigned speed_kp, unsigned speed_ki, unsigned pos_kp, unsigned pos_ki)` | `bool`；`axis=0` 为 yaw，`axis=1` 为 pitch；校验轴和 0～256 范围，要求已使能；不修改模式或目标，不保存 |
| `read(State* out)` / `read_timeout(State* out, int timeout_ms)` | `bool`；`out` 不可为空。超时接口两轴共用 1～1000 ms 预算；State 含 `ok`、`yaw_deg`、`pitch_deg`；仅返回 true 时整体有效 |
| `set_zero()` | `bool`；当前位置置零并持久保存，与 Python 一致 |
| `start_log(const std::string& path)`、`stop_log()` | 前者返回 `bool`，后者 `void` |
| `last_error() const` | `const std::string&`，引用属于对象；需要跨后续操作保存时复制文本 |
| 析构函数 | 串口仍打开时执行有界关闭，失败向 stderr 报告；应主动使用 `close_checked()` 取得结果 |

### C：`#include <f32c/c_api.h>`

句柄为不透明 `f32c_gimbal*`。除打开函数及 void 函数外，操作以 **1 为成功、0 为失败**。

| 函数 | 参数 / 返回与作用 |
| --- | --- |
| `f32c_config_defaults(f32c_config* out)` | 填充默认配置；返回 void；配置地址字段名为 `yaw_id/pitch_id`，其余数值字段与 Python 对应 |
| `f32c_open_config(const f32c_config* config)` | 返回已打开且两轴 ping 成功的句柄；失败返回 NULL；device 指针在调用期间有效即可 |
| `f32c_open(const char* device, unsigned yaw_id, unsigned pitch_id)` | 简化打开入口，其余字段用默认配置；NULL/空 device 自动扫描；失败返回 NULL |
| `f32c_close(f32c_gimbal* g)` | 保留 void，调用可检查关闭，失败向 stderr 报告并保留全局错误；NULL 无操作 |
| `f32c_close_checked(f32c_gimbal* g)` | 0.3.0 新增；本实例使能/待清理时一轮两轴失能尝试并释放句柄，1=请求已发送，0=最终清理失败/硬件状态未知；NULL 返回1。失败用 `f32c_last_error(NULL)`；无论返回值均不能再使用旧句柄 |
| `f32c_ping(g)`、`f32c_enable(g)`、`f32c_disable(g)` | 与 C++ 对应 |
| `f32c_enable_at(g, double yaw, double pitch)` | 返回 1/0；显式启动目标，语义同 C++；NULL 句柄返回 0 |
| `f32c_set_deg(g, double yaw, double pitch)` | 两轴单圈目标 |
| `f32c_set_yaw_deg(g, double deg)`、`f32c_set_pitch_deg(g, double deg)` | 单轴目标 |
| `f32c_set_pi(g, unsigned axis, unsigned speed_kp, unsigned speed_ki, unsigned pos_kp, unsigned pos_ki)` | 运行时 PI；轴为 0/1，校验四个 unsigned 值均 ≤256，避免窄整型截断 |
| `f32c_read(g, double* yaw, double* pitch)` / `f32c_read_timeout(g, yaw, pitch, unsigned timeout_ms)` | 输出指针不可为空；仅返回 1 时同时使用两轴数据 |
| `f32c_set_zero(g)` | 两轴当前位置置零并保存 |
| `f32c_start_log(g, const char* path)`、`f32c_stop_log(g)` | 前者返回 1/0，后者 void |
| `f32c_last_error(const f32c_gimbal* g)` | 返回 SDK 拥有的错误字符串，不应 free；打开/关闭失败后传 NULL 查询当前线程的最近全局错误；后续操作前应复制 |

C/C++ 接口均无异常化错误约定；无效句柄等在 C 桥中直接返回 0，不保证每种参数失败都更新错误文本。调用方必须管理句柄生命周期。

## 6. 调参工具、验证与重新打包

在源码仓库中构建并运行无硬件检查：

```bash
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 tests/test_tuning.py
python3 tests/test_runtime_pi.py
```

自动工具预览不连接硬件；`--enable` 才执行自动 PI 写入及运动：

```bash
python3 examples/autotune.py --device /dev/ttyACM0 --center 0 0
python3 examples/autotune.py --device /dev/ttyACM0 --center 0 0 --enable
```

整轮试验只使能一次，通过 `set_pi()` 更新候选，保持其它参数及目标控制；结束或异常时尝试失能。输出根目录含 `run.json`、整轮 `serial.log`；每个候选含 `samples.csv` 和 `result.json`。仅完整验证通过时 `recommendation` 才非空。`--center`、`--validation-offsets`、`--step` 定义测试范围；所有位置和阶跃在使能前检查 pitch ≤45°，不把越界目标裁剪后当作测试通过。大跟随误差、持续无进展会中止。

人工受扰记录可用 `examples/tune.py` 的 `hold/push/release/report` 命令，详见仓库 readme。自动试验使用目标阶跃，不等同于真实外力冲击；无力传感器时不能给出 Nm/rad 的绝对刚度。线缆阻力可作为现有负载参与整定，实体极限不能用更高 PI 强推。

UI 的“使能并到保持位”使用 `HeadController.enable()` 的默认已存零位 yaw=0°、pitch=0°目标，滑条/到 0°调用 `move_to()`，共用同一轨迹和保护。计划周期约 33 ms，实际包含串口和界面耗时；yaw 滑条 ±60°，pitch −180°～45°。UI 仅展示状态并记录 `.fault.json` / `.close.json`，没有第二份滤波、误差保护或电机 ctypes 绑定。相机仍是独立可选组件。

```bash
F32C_PYTHON=/usr/bin/python3 ./package_sdk.sh
```

打包会重新编译电机库、运行四个 C++ 测试（含阻塞写入/失能失败注入）和 Python SDK 检查、伪终端 PI/跟随集成测试、调参、纯跟随、HeadController 和分片/噪声伪串口测试，然后生成平台 whl 并同步 `wheel/SDK使用说明.md`。UI 另用已安装 PySide6 的 Python 运行 `tests/test_ui_defaults.py`，覆盖关闭失败报告及无置零入口。构建机器需要 CMake、支持 C++17 的编译器和带 pip、setuptools、wheel 的 Python。跨平台需在目标平台重新构建。旧版本的原生库复用路径已移除，0.2.0 从源码构建电机库。

打包过程中还会验证 whl 的文件清单、校验值、Python 模块及配套说明，确保仅发布电机组件。构建缓存、原始现场日志及本机配置不提交 Git；新 whl 与配套说明存放在 `wheel/`。
