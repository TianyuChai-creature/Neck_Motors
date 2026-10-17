# Neck Motors 0.3.2：Quest 跟随集成

0.3.2把本轮成功回零与短时保持使用的电机配置设为 C++、C、Python 的默认值。主仓应删除旧的 PI、速度和加速度覆盖，直接使用 `Config(device=...)`；显式传入的配置仍会覆盖 SDK 默认值。

| 项目 | yaw（ID=1） | pitch（ID=2） |
| --- | ---: | ---: |
| 速度 KP / KI | 40 / 4 | 10 / 4 |
| 位置 KP / KI | 42 / 0 | 40 / 0 |
| 电机目标速度参数 | 5 RPM | 5 RPM |
| 电机加速度参数 | 1转/s² | 1转/s² |
| `HeadController()` 默认目标范围 | −60°～60° | −180°～45° |
| 默认软件参考速度 / 加速度上限 | 30°/s / 360°/s² | 30°/s / 360°/s² |

使用单圈直通模式 `0004`。电机参数和软件参考限制是两层约束，不能将它们等同于实际电机速度。独立 `TrackingConfig` 仍保留通用算法默认值；如果主仓显式传入它，应明确配置30°/s、360°/s²及范围，不能继续套用旧的120/600或180/1800高速配置。

## 已验证范围

2026-10-17，低层 `Gimbal2Dof` 使用上述默认电机配置，完成35→30→…→0°、35°一次回0°以及直接使能到0°。一次0°保持10秒期间，yaw反馈为−0.2°～−0.1°、pitch为−0.3°～−0.1°，pitch母线电压11.06～11.38 V。用户确认前两轮没有外力辅助。

这些实测是低层点到点控制。`HeadController` 的连续流、过期输入、目标覆盖和权限处理通过离线测试；Quest网络、坐标映射、相机并行、实际端到端延迟和高速运动尚未完成主仓真机验收。历史失败的单一根因未确定，不应将本次成功推广到任意负载、起点或供电条件。

## 控制线程中的调用方式

主仓提供三个回调：`take_latest_new_sample()` 非阻塞取走最新且尚未消费的授权源样本（没有则返回 `None`）；`may_send()` 快速、无副作用地检查当前发送许可；`running()` 表示应用是否继续运行。网络线程只交接最新样本，不直接操作电机。

样本中的 `yaw_motor_deg`、`pitch_motor_deg` 已映射到电机坐标；`received_monotonic_s` 是主机收到该样本时记录的 `time.monotonic()`，不是Quest设备时钟，也不是每次控制循环重新生成的时间。

```python
import time
from f32c import Config, HeadController


def follow_quest(take_latest_new_sample, may_send, running, device=None):
    with HeadController(Config(device=device)) as head:
        # 调用前由主仓取得操作员的使能许可。
        head.enable()
        head.hold()  # 取消默认0°/35°待机移动，原位等待首个授权样本。
        while running():
            started = time.monotonic()
            sample = take_latest_new_sample()
            if sample is not None and may_send():
                head.set_target(
                    sample.yaw_motor_deg,
                    sample.pitch_motor_deg,
                    received_at=sample.received_monotonic_s,
                )
            snapshot = head.tick(may_send=may_send)
            # 主仓用 snapshot.measured / reason 展示反馈和状态。
            time.sleep(max(0, 1 / 30 - (time.monotonic() - started)))
```

控制周期建议约30 Hz，实际包含串口读写耗时；不要在此线程阻塞等待网络或相机。`tick()` 应持续调用，即使没有新Quest样本。超过100 ms的活跃控制间隔会故障并尝试失能。

首个新 `set_target()` 会覆盖使能后的默认待机目标；后续样本采用 latest-wins，不排队执行过期目标。不要每帧调用 `enable()`、`move_to()` 或重复发送旧包刷新时间，也不要在 SDK 外再叠加一套滤波/轨迹。

## 坐标、输入和授权

- 输入单位是度，yaw增大向第一人称左侧，pitch增大低头。若Quest pitch抬头为正，主仓应反号；已经转换成电机坐标的数据不能再次反号。
- 四元数/Euler约定、坐标参考、比例及相对零点属于主仓。可在授权开始时将Quest参考姿态与电机回读对齐，避免切换来源时跳变；不要调用电机 `set_zero()` 或保存参数来实现软件对齐。
- 同一对象由一个控制线程拥有，任何 UI、调参程序或第二个进程都不能同时占用串口。
- 默认输入超过0.2秒未更新会进入 `input_stale` 保持，撤销许可会进入 `permission_lost` 保持；都停止生成新参考，电机可能继续追踪最后已发送目标，不等同于物理急停。
- 来源或身份切换时先 `head.hold("source_changed")`，重新取得许可并对齐参考后才提交新样本。不要直接把另一个人的姿态接续到旧轨迹。
- 故障通过 `HeadControlError` 报告；不要自动重新使能。记录原因、处理后由操作员明确恢复。退出用 `close()`/上下文管理器尝试清理，失能发送成功不是电机状态ACK。

## 待机、回零与底层API

`DEFAULT_HOLD_POSITION` 仍是 yaw=0°、pitch=35°。独立UI调用 `HeadController.enable()` 后持续 `tick()`，会前往该位置；上面的Quest循环调用 `hold()` 取消这段自动移动。显式回零使用 `head.move_to(0, 0)` 并持续 `tick()`，不要与实时输入竞争。

低层 `Gimbal2Dof.enable()` 继续保持当前位置；`enable_at(yaw,pitch)` 和 `set_deg()` 是直接目标接口，复现本轮低层测试时可用，但不提供流式输入新鲜度、主机轨迹和控制器反馈保护。默认保持坐标不应作为Quest样本上的强制常量。

## 更新与核对

```bash
python3 -m pip install --upgrade wheel/f32c_neck_motors-0.3.2-py3-none-linux_x86_64.whl
python3 -c 'import f32c; from importlib.metadata import version; print(version("f32c-neck-motors")); print(f32c.__file__); print(f32c.Config())'
```

确认导入位置来自主仓的实际运行环境，而不是旧源码路径或其它虚拟环境。原生库应与此whl配套；不要保留指向旧库的 `F32C_LIBRARY_DIR`。包内文档位于 `f32c.native_prefix()/"share/doc/f32c"`。
