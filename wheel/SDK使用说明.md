# F32C Neck Motors SDK 0.1.1 使用说明

## 1. 发布内容与安装

发布文件为 `wheel/f32c_neck_motors-0.1.1-py3-none-linux_x86_64.whl`，本说明同时复制至 `wheel/SDK使用说明.md`。轮子包含 Python 包 `f32c`、电机共享库 `libf32c.so`、ZED Mini 适配库 `libf32c_zed.so`、C/C++ 公开头文件、CMake 包配置和本说明。Python >=3.10；平台为 Linux x86_64，原生库需要兼容的 glibc/libstdc++。相机还需要目标系统提供兼容的 Stereolabs ZED SDK、CUDA 和驱动；这些系统依赖不在 whl 中。只控制电机不加载相机库。

```bash
python3 -m pip install --upgrade wheel/f32c_neck_motors-0.1.1-py3-none-linux_x86_64.whl
python3 -c 'import f32c; from importlib.metadata import version; print(version("f32c-neck-motors")); print(f32c.native_prefix())'
```

`native_prefix() -> pathlib.Path` 返回已安装包内的 `native/` 目录，可用于定位头文件、库和 CMake 包。正常安装不需要设置 `F32C_LIBRARY_DIR`；这个环境变量只用于开发时覆盖库目录，指向旧库时新 API 可能不可用。

0.1.1 新增运行时 `set_pi()`，便于持续使能下整定，保留原有配置结构和默认 PI。调参工具与示例位于源码仓库 `examples/`，不随 whl 安装。

## 2. 现场方向、边界与控制语义

本设备当前零位由用户手动调整：

| 关节 | 增大角度 | 减小角度 | 现场边界 |
| --- | --- | --- | --- |
| yaw，默认 ID=1 | 第一人称向左 | 第一人称向右 | 无已知机械限位；相机线缆可能增加阻力 |
| pitch，默认 ID=2 | 低头 | 抬头 | 低头极限约 43°；正常抬头范围未发现限位 |

SDK 保留电机原始方向，不自动换符号或加零位偏置。这些方向是本机安装约定，不适用于所有安装。若上游 pitch 向上为正，需要在调用方反号映射。

单圈目标以 0.1° 为协议分辨率，按当前实现截断量化并对 360° 取模；例如 -10° 与 350° 指向同一位置。电机选择单圈最短路径。`read()` 保留电机反馈值，不转成 ±180°；需要有符号显示时可用 `(value + 180) % 360 - 180`。

`set_deg()` 立即下发目标，不生成轨迹，也不校验本机 43° 机械边界。连续运动应由调用方生成中间目标，并保证整条路径避开极限。调参工具的 pitch 测试目标上限为 40°，包含测试位置和正负阶跃，留出 3° 余量；该保护不是 SDK 全局限位。

两轴共用一条串口，命令和反馈按轴串行处理，不是同步更新/采样。同一个实例应由一个线程串行访问。串口写成功只代表写入成功，协议没有命令 ACK；不得把它等同于到位、PI 已被反馈验证或成功持久保存。

## 3. Python 配置与数据类型

```python
from f32c import Config, F32CError, Gimbal2Dof

with Gimbal2Dof(Config(device="/dev/ttyACM0")) as motor:
    print(motor.read())
```

此例不使能、不下发位置；退出时 Python `close()` 仍会尝试发送失能命令。不要与另一个控制进程同时占用该总线。

### Config

`Config(...)` 是数据类，在 `open()` 时提交配置；使能时下发速度、加速度和两轴 PI。

| 字段 | 默认值 | 含义 / 取值 |
| --- | --- | --- |
| `device` | `None` | 串口路径；空值扫描 `/dev/serial/by-id/*`、`ttyACM*`、`ttyUSB*`，要求两轴回应 |
| `baud` | `115200` | 支持 9600、57600、115200，8N1 |
| `id_yaw`, `id_pitch` | `1`, `2` | 地址 1～255，必须不同 |
| `travel_rpm` | `80` | 目标速度参数，0～1000 RPM；不等同于主机轨迹速度 |
| `accel` | `10` | 加速度参数，0～65535，协议单位转/s² |
| `yaw_speed_kp`, `yaw_speed_ki` | `40`, `0` | yaw 速度环 PI，整数 0～256 |
| `yaw_pos_kp`, `yaw_pos_ki` | `42`, `0` | yaw 位置环 PI，整数 0～256 |
| `pitch_speed_kp`, `pitch_speed_ki` | `10`, `4` | pitch 速度环 PI，整数 0～256 |
| `pitch_pos_kp`, `pitch_pos_ki` | `40`, `0` | pitch 位置环 PI，整数 0～256 |

本版本未改变这些默认 PI；它们不代表当前固定负载已完成验收。`set_pi()` 成功后更新当前实例的 `config`，不会修改 `Config()` 的默认值。

| 类型 | 字段 / 用途 |
| --- | --- |
| `State` | 不可变数据类：`yaw_deg: float`、`pitch_deg: float`，来自角度反馈，单位度 |
| `F32CError` | 库加载、连接或原生操作失败时的异常，继承 `RuntimeError` |
| `Frame` | 不可变图像数据类：`data: bytes`、`width`、`height`、`channels`、`stride_bytes`、`timestamp_ns` |
| `View` | `LEFT=0`、`RIGHT=1`、`SIDE_BY_SIDE=2` |
| `Setting` | 相机设置枚举，见第 6 节 |

## 4. Python 电机 API

`Gimbal2Dof(config: Config | None = None)` 创建对象并加载电机库，不打开串口、不使能。配置省略时使用新的默认 `Config()`。

| 函数 / 签名 | 参数 | 返回值与行为 |
| --- | --- | --- |
| `open() -> Gimbal2Dof` | 无 | 连接并检测两轴，返回自身；已打开则直接返回；失败抛 `F32CError` |
| `close() -> None` | 无 | 底层尝试失能并关闭连接；重复关闭已关闭对象无操作 |
| `ping() -> bool` | 无 | 查询两轴是否回应；查询失败返回 `False`，可读 `last_error()`；未打开则抛异常 |
| `enable() -> None` | 无 | 使能两轴，选择单圈直通模式，应用实例配置并保持当前角度；不自动回零；已使能时不重复初始化 |
| `disable() -> None` | 无 | 下发两轴失能；失败抛 `F32CError`，写成功不代表收到电机确认 |
| `read() -> State` | 无 | 串行读取两轴角度；任一轴失败则抛异常，不返回部分有效 State |
| `set_deg(yaw_deg: float, pitch_deg: float) -> None` | 两个有限角度，单位度 | 依次下发 yaw、pitch；要求已使能；不平滑、不限机械角度 |
| `set_yaw_deg(deg: float) -> None` | 有限角度 | 只下发 yaw 目标；要求已使能 |
| `set_pitch_deg(deg: float) -> None` | 有限角度 | 只下发 pitch 目标；要求已使能 |
| `set_pi(axis: str, speed_kp: int, speed_ki: int, pos_kp: int, pos_ki: int) -> None` | `axis` 为 `"yaw"` 或 `"pitch"`；四个整数均在 0～256 | 0.1.1 新增。已使能时更新该轴四个 PI，保留模式、目标和使能状态；不保存参数。四帧串行发送，不是原子更新；错误时抛异常 |
| `set_zero() -> None` | 无 | 要求已使能。把两轴当前位置设为零点并分别保存到电机，重启仍生效；不能用于移动到已有 0° |
| `start_log(path) -> None` | 路径类型 str 或 os.PathLike；父目录需已存在 | 打开原始串口日志，采用追加写入；记录 TX/RX/FLUSH/TX_ERROR；失败抛异常 |
| `stop_log() -> None` | 无 | 停止原始日志；要求连接已打开 |
| `last_error() -> str` | 无 | 返回底层最近的错误文本；连接失败时可查询打开错误 |

`with Gimbal2Dof(...) as motor` 相当于进入时 `open()`、退出时 `close()`。除 `ping()` 的查询失败外，操作失败通常抛 `F32CError`；配置或 `set_pi()` 参数校验失败抛 `ValueError`。参数更新中途失败时，电机可能只接受了部分 PI，应终止该次实验并失能，不将它计入已验证配置。

运行时调整示例（会使能；这里的数值仅演示 API）：

```python
from f32c import Config, Gimbal2Dof

with Gimbal2Dof(Config(device="/dev/ttyACM0")) as motor:
    if input("确认使能并保持当前位置，输入 ENABLE: ") != "ENABLE":
        raise SystemExit
    try:
        motor.enable()
        motor.set_pi("yaw", speed_kp=40, speed_ki=0, pos_kp=42, pos_ki=0)
        print(motor.read())
    finally:
        motor.disable()
```

该例不改零点、不调用持久保存，也不自动移到 0°。运行期间参数仍需按实际负载验证。`set_zero()` 会发送保存参数命令，不应在未验收的 PI 试验期间调用，以免保存当前参数。

## 5. C++ / C 电机 API

### CMake 使用

```bash
SDK_PREFIX=$(python3 -c 'import f32c; print(f32c.native_prefix())')
cmake -S your_project -B your_project/build -DCMAKE_PREFIX_PATH="$SDK_PREFIX"
```

```cmake
find_package(f32c CONFIG REQUIRED)
target_link_libraries(your_app PRIVATE f32c::f32c)
# 相机功能另加：target_link_libraries(your_app PRIVATE f32c::f32c_zed)
```

### C++：`#include <f32c/gimbal.hpp>`

`f32c::Gimbal2Dof()` 使用默认配置；`Gimbal2Dof(Config cfg)` 使用自定义配置。`Config` 字段与 Python 对应：`id_yaw/id_pitch` 为 `uint8_t`，PI 和加速度为 `uint16_t`；构造前应检查宽整型数值，避免转换截断。对象不可复制。

| 函数 | 返回值 / 与 Python 的区别 |
| --- | --- |
| `open()` | `bool`；显式路径只打开串口，自动扫描时检查两轴回应；需用 `ping()` 检测显式路径的两轴 |
| `close()` | `void`；如本对象已使能则尝试失能，随后关闭串口 |
| `ping()`、`enable()`、`disable()` | `bool`；失败返回 `false`，从 `last_error()` 取原因 |
| `set_deg(double yaw, double pitch)`、`set_yaw_deg(double)`、`set_pitch_deg(double)` | `bool`；角度语义同 Python |
| `set_pi(unsigned axis, unsigned speed_kp, unsigned speed_ki, unsigned pos_kp, unsigned pos_ki)` | `bool`；`axis=0` 为 yaw，`axis=1` 为 pitch；校验轴和 0～256 范围，要求已使能；不修改模式或目标，不保存 |
| `read(State* out)` | `bool`；`out` 不可为空。State 含 `ok`、`yaw_deg`、`pitch_deg`；仅返回 true 时整体有效 |
| `set_zero()` | `bool`；当前位置置零并持久保存，与 Python 一致 |
| `start_log(const std::string& path)`、`stop_log()` | 前者返回 `bool`，后者 `void` |
| `last_error() const` | `const std::string&`，引用属于对象；需要跨后续操作保存时复制文本 |
| 析构函数 | 如对象已使能则尝试失能并释放串口资源 |

### C：`#include <f32c/c_api.h>`

句柄为不透明 `f32c_gimbal*`。除打开函数及 void 函数外，操作以 **1 为成功、0 为失败**。

| 函数 | 参数 / 返回与作用 |
| --- | --- |
| `f32c_config_defaults(f32c_config* out)` | 填充默认配置；返回 void；配置地址字段名为 `yaw_id/pitch_id`，其余数值字段与 Python 对应 |
| `f32c_open_config(const f32c_config* config)` | 返回已打开且两轴 ping 成功的句柄；失败返回 NULL；device 指针在调用期间有效即可 |
| `f32c_open(const char* device, unsigned yaw_id, unsigned pitch_id)` | 简化打开入口，其余字段用默认配置；NULL/空 device 自动扫描；失败返回 NULL |
| `f32c_close(f32c_gimbal* g)` | 尝试两轴失能并释放句柄；返回 void；NULL 无操作；释放后禁止再次使用句柄 |
| `f32c_ping(g)`、`f32c_enable(g)`、`f32c_disable(g)` | 与 C++ 对应 |
| `f32c_set_deg(g, double yaw, double pitch)` | 两轴单圈目标 |
| `f32c_set_yaw_deg(g, double deg)`、`f32c_set_pitch_deg(g, double deg)` | 单轴目标 |
| `f32c_set_pi(g, unsigned axis, unsigned speed_kp, unsigned speed_ki, unsigned pos_kp, unsigned pos_ki)` | 运行时 PI；轴为 0/1，校验四个 unsigned 值均 ≤256，避免窄整型截断 |
| `f32c_read(g, double* yaw, double* pitch)` | 输出指针不可为空；仅返回 1 时同时使用两轴数据 |
| `f32c_set_zero(g)` | 两轴当前位置置零并保存 |
| `f32c_start_log(g, const char* path)`、`f32c_stop_log(g)` | 前者返回 1/0，后者 void |
| `f32c_last_error(const f32c_gimbal* g)` | 返回 SDK 拥有的错误字符串，不应 free；打开失败时传 NULL；后续操作前应复制需要保留的文本 |

C/C++ 接口均无异常化错误约定；无效句柄等在 C 桥中直接返回 0，不保证每种参数失败都更新错误文本。调用方必须管理句柄生命周期。

## 6. ZED Mini 相机 API

Python `ZedMini()` 构造对象并加载相机适配库，不连接相机。默认连接参数为 HD720@60、关闭深度、CPU BGRA8；`serial_number=0` 选择首台相机，非零指定序列号，连接后必须是 ZED Mini。

| Python 函数 | 参数、返回值与行为 |
| --- | --- |
| `open(serial_number: int = 0) -> ZedMini` | 序列号 0～2³²−1；连接并返回自身；失败抛 `F32CError` |
| `close() -> None` | 关闭相机；重复关闭可用；不影响电机对象 |
| `is_open() -> bool` | 是否已连接 |
| `grab() -> None` | 采集新帧；要求已连接；失败抛异常 |
| `retrieve(view: View = View.LEFT) -> Frame` | 必须先成功 grab；返回该帧指定视图，复制为 Python 自持有的 BGRA8 bytes |
| `set_setting(setting: Setting, value: int) -> None` | value 必须能表示为 int32；合法值范围由本机 ZED SDK 校验；EXPOSURE/GAIN 的 -1 为自动模式 |
| `get_setting(setting: Setting) -> int` | 查询当前设置，要求已连接 |
| `serial_number() -> int` | 返回连接相机的序列号；关闭后为 0 |
| `last_error() -> str` | 最近错误文本 |
| `with ZedMini() as camera` | 进入时 open，退出时 close；对象释放时销毁原生句柄 |

`Frame.data` 长度为 `stride_bytes * height`，每行可能有步长填充；`channels=4`，顺序 BGRA。`timestamp_ns` 为 ZED 图像时间戳，单位纳秒。再次 grab/retrieve 或关闭相机不会破坏已返回的 Python bytes。

| Setting | 数值 | 用途 |
| --- | --- | --- |
| `BRIGHTNESS` | 0 | 亮度 |
| `CONTRAST` | 1 | 对比度 |
| `SATURATION` | 2 | 饱和度 |
| `SHARPNESS` | 3 | 锐度 |
| `GAIN` | 4 | 增益；-1 恢复自动 |
| `EXPOSURE` | 5 | 曝光；-1 恢复自动 |
| `WHITE_BALANCE_TEMPERATURE` | 6 | 白平衡色温 |
| `LED_STATUS` | 7 | 相机 LED 状态 |

```python
from f32c import Setting, View, ZedMini

with ZedMini() as camera:
    camera.set_setting(Setting.EXPOSURE, -1)
    camera.set_setting(Setting.GAIN, -1)
    camera.grab()
    frame = camera.retrieve(View.LEFT)
    print(frame.width, frame.height, frame.stride_bytes, frame.timestamp_ns)
```

C++ 头文件为 `<f32c/camera/zed_mini.hpp>`，类为 `f32c::camera::ZedMini`：

| 函数 | 返回 / 参数 |
| --- | --- |
| `open(unsigned serial_number=0)` | bool |
| `close()`、`is_open() const` | void、bool |
| `grab()` | bool |
| `retrieve(View view, ImageView* out)` | bool；out 不可为空 |
| `set_setting(Setting setting, int value)` | bool |
| `get_setting(Setting setting, int* value)` | bool；输出指针不可为空 |
| `serial_number() const`、`last_error() const` | unsigned、const std::string& |

C++ 枚举名为 `View::Left/Right/SideBySide` 与 `Setting::Brightness/Contrast/Saturation/Sharpness/Gain/Exposure/WhiteBalanceTemperature/LedStatus`。`ImageView` 字段对应 Frame，但 `data` 为借用的 `const uint8_t*`；同一视图再次 retrieve 或相机 close 后可能失效，调用方若需保留应复制。

C 头文件为 `<f32c/camera_c_api.h>`，函数如下：

| 函数 | 返回 / 行为 |
| --- | --- |
| `f32c_zed_create()`、`f32c_zed_destroy(camera)` | 创建句柄 / 释放；destroy 后禁止再使用 |
| `f32c_zed_open(camera, unsigned serial_number)` | 1/0，序列号 0 自动选择 |
| `f32c_zed_close(camera)`、`f32c_zed_is_open(camera)` | void、1/0 |
| `f32c_zed_grab(camera)` | 1/0 |
| `f32c_zed_retrieve(camera, unsigned view, f32c_zed_image_view* out)` | 1/0，视图 0/1/2；输出指针不可为空；data 借用内存 |
| `f32c_zed_set_setting(camera, unsigned setting, int value)` | 1/0，设置枚举 0～7 |
| `f32c_zed_get_setting(camera, unsigned setting, int* value)` | 1/0，输出指针不可为空 |
| `f32c_zed_serial_number(camera)` | unsigned，关闭或 NULL 为 0 |
| `f32c_zed_last_error(camera)` | SDK 拥有的 const char*；NULL 返回句柄错误文本 |

C 图像结构含 `data`、`size_bytes`、`stride_bytes`、`width`、`height`、`channels`、`timestamp_ns`。枚举常量前缀为 `F32C_ZED_VIEW_*` 与 `F32C_ZED_SETTING_*`，名称见公开头文件。错误处理及借用内存寿命与 C++ 对应。

## 7. 调参工具、验证与重新打包

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

整轮试验只使能一次，通过 `set_pi()` 更新候选，保持其它参数及目标控制；结束或异常时尝试失能。输出根目录含 `run.json`、整轮 `serial.log`；每个候选含 `samples.csv` 和 `result.json`。仅完整验证通过时 `recommendation` 才非空。`--center`、`--validation-offsets`、`--step` 定义测试范围；所有位置和阶跃在使能前检查 pitch ≤40°，不把越界目标裁剪后当作测试通过。大跟随误差、持续无进展或 pitch 越界会中止。

人工受扰记录可用 `examples/tune.py` 的 `hold/push/release/report` 命令，详见仓库 readme。自动试验使用目标阶跃，不等同于真实外力冲击；无力传感器时不能给出 Nm/rad 的绝对刚度。线缆阻力可作为现有负载参与整定，实体极限不能用更高 PI 强推。

目前真机整定尚未完成完整工作范围验收；本次发布提供整定能力，不将未验收候选替换默认值。

```bash
F32C_PYTHON=/usr/bin/python3 ./package_sdk.sh
```

打包会构建电机与 ZED 适配库、执行原生测试、安装 SDK 开发文件并生成平台 whl。构建机器需要 ZED/CUDA 开发依赖；源码仅构建电机时可不启用 `F32C_BUILD_ZED`。跨平台需在目标平台重新构建。构建缓存、原始现场日志及本机配置不提交 Git；新 whl 与使用说明存放在 `wheel/`。
