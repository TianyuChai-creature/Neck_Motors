# 代码流程、工程架构与 API 审查

| 项目 | 记录 |
| --- | --- |
| 审查日期 | 2026-10-10（Asia/Shanghai） |
| 基线 | `main`，`bc9b579ff1437c9a69a1797e4694915e5b1fdcb2`，SDK 0.1.1 |
| 审查分支 | `codex/review-flow-architecture-api` |
| 交付范围 | 审查报告、复现方法和整改路线；本报告按上述基线审查，后续同分支功能改造不属于该审查基线 |
| 验证环境 | Linux x86_64、g++ 15.2.0、Python 3.14.4；伪终端、模拟对象和系统调用故障注入 |

## 1. 审查结论与证据边界

现有协议、串口、单电机、两轴控制、C 桥接和 Python 绑定的分层清晰，核心规模较小，适合渐进改进。优先处理失能状态、写入截止时间和界面的 pitch 边界，再改善反馈解析及测试保障，最后整理线程归属、公共 API 和发布工程。

本次确认六项缺陷，其中三项为 P1。现有无硬件测试通过，但尚未覆盖这些失败场景。另列五项架构/API 建议，不能将建议中的目标设计误读为本版本已经具备的能力。

**优先级**：P1 为应优先处理的可靠性或运动边界问题；P2 为需安排修复的条件性缺陷或工程改进。**证据类型**：已确认缺陷以静态代码及复现结果为依据；架构建议说明收益和代价；待真机验证只提出实验条件，不宣称机械效果。

SDK 文档已明确单实例应串行访问、写成功不等于电机确认、多轴操作非同步。这些约定本身不是缺陷。接口可以进一步表达未知状态、部分完成和反馈时序，但不能凭空增加协议 ACK 或声称固件支持原子更新。

核对材料为[SDK 使用说明](SDK使用说明.md)、[仓库设计与现场记录](../agent.md)及[本地电机手册](F32C无刷电机使用手册-2026-07-21.pdf)第 7、8、13、14 页。现场记录中的 pitch 低头极限约 43°、调参目标上限 40°属于当前装置约定，不应成为通用协议库的硬编码限位。

### 问题索引

| 编号 | 优先级 | 维度 | 判定 | 内容 |
| --- | --- | --- | --- | --- |
| F01 | P1 | 流程 | 已确认缺陷，必须修 | 失能失败后清除使能标记，UI 显示失能成功 |
| F02 | P1 | 流程 | 已确认缺陷，必须修 | 串口写入没有截止时间，持续背压导致无限等待 |
| F03 | P2 | 流程 | 已确认缺陷，必须修 | 合法反馈被误判为短帧，前置回显阻挡实际反馈 |
| F04 | P1 | 流程/控制 | 已确认缺陷，必须修 | pitch 缺少目标及路径边界检查 |
| F05 | P2 | API | 已确认缺陷，必须修 | 配置只校验数值范围，类型错误延迟至 ctypes 转换 |
| F06 | P2 | 工程 | 已确认缺陷，必须修 | `NDEBUG` 使测试调用和断言一起消失 |
| A01 | P2 | 架构/流程 | 架构建议 | 单一总线执行线程、相机解耦及控制周期预算 |
| A02 | P2 | 架构 | 架构建议 | UI 复用 Python SDK，提取共享运动与统计逻辑 |
| A03 | P2 | API | 架构建议 | 结构化错误、每轴有效性和采样时间 |
| A04 | P2 | API/流程 | 架构建议 | 表达部分完成、持久保存副作用及 C ABI 演进 |
| A05 | P2 | 工程 | 架构建议 | 构建开关、头文件安装、可选相机打包和 CI |

## 2. 已确认缺陷

### F01 / P1：失能失败后的状态与清理路径不可靠

- **位置**：[Gimbal2Dof 失能与清理](../src/gimbal.cpp#L140)，第 60–65、99–102、122–133、140–146 行；[C 关闭入口](../src/c_api.cpp#L80)，第 80–84 行；[UI 失能按钮](../examples/ui.py#L337)，第 337–340 行。
- **事实/复现**：`disable()` 无论两轴写入是否成功都会设置 `armed=false`；使能中途失败的回滚也忽略两次失能结果。C++ `close()` 和析构依赖该标记决定是否再次失能。将 UI 的 `f32c_disable` 模拟为返回 0，调用 `Panel.disable()` 后仍得到 `armed=False` 和显示文本“失能”；附录 B 可无 PySide6 复现。
- **影响**：某轴可能仍处于使能状态，却被上层视为已失能。C++ 路径在已清除标记后不会自动重试。C `f32c_close()` 会额外尝试一次失能，因其忽略结果且返回 `void`，也无法向调用方交付最终失败原因；不能笼统说所有关闭路径都完全没有重试。
- **改进方案**：内部区分关闭、连接但尚未使能、使能请求已发送、失能请求已发送、故障/状态未知，并保留每轴操作结果。失败回滚不能覆盖原始错误，应同时记录清理错误。停止目标下发后，在总线仍可用时执行有界清理重试；UI 对失败显示“失能失败/状态未知”。协议无 ACK，成功写入只能表达“失能命令已发送”。
- **兼容性**：内部状态替换可保留现有布尔 API；新增状态及可检查关闭结果的入口，让旧 `close()` 保持尽力清理的生命周期契约。UI 状态文案属于必要行为修正。
- **验证方法**：分别注入 yaw、pitch、两轴失能写失败；测试部分使能后回滚失败、重复关闭、断线关闭、显式失能后析构，检查两轴尝试次数、时间上限、错误保留及 UI 提示。物理失能结果需真机观察，伪终端只能验证调用与状态逻辑。

### F02 / P1：串口写入没有截止时间

- **位置**：[SerialPort::write_all](../src/serial_port.cpp#L103)，第 103–116 行；[读取错误处理](../src/serial_port.cpp#L127)，第 127–138 行。
- **事实/复现**：遇到 `EAGAIN` 或 `EINTR` 立即循环重试，没有等待、截止时间或取消点；写入返回 0 时 `off` 不推进。用链接器 `--wrap=write` 将写调用固定为 `EAGAIN`，进程 0.5 秒后仍不返回，由测试父进程终止；附录 C 提供完整命令。
- **影响**：串口背压可能导致忙等，并阻塞控制、失能和退出。`read_for()` 有总截止时间，但未区分正常暂无数据与实际 I/O 错误，错误原因可能被上层概括成“读失败”。
- **改进方案**：内部传递基于单调时钟的操作截止时间，使用 `poll`/`ppoll` 等待可写并处理剩余预算；`EINTR` 后重算剩余时间，零进展必须受截止时间约束。记录已写字节数及 `errno`。部分帧写出后失败应进入通信故障路径，避免盲目重发导致重复或损坏帧。
- **兼容性**：先给现有调用增加内部有界默认预算；开放超时配置时采用新增配置入口。终止无限等待是必要修正；不要将超时返回解释为电机已停止。
- **验证方法**：覆盖持续/短暂 `EAGAIN`、重复 `EINTR`、零字节写、短写后失败和设备断开，检查返回时间及 CPU 忙等。设备 DMA 对分段写入和帧间隔的实际要求保留真机验证，不仅凭主机写成功判断有效。

### F03 / P2：反馈解析没有按预期帧筛选

- **位置**：[通用解析器](../include/f32c/protocol.hpp#L106)，第 106–128 行；[Motor::query_mech](../src/motor.cpp#L49)，第 49–58 行。
- **事实/复现**：解析器按 5、6、7、9 字节试探，找到第一个校验正确的候选便返回。`7A 78 02 00 00 00 7B 7B 7B` 是 ID 120、数据值 123 即 12.3° 的合法九字节帧；其七字节前缀也满足当前校验，于是只得到两字节数据，随后被 `feedback_i32()` 拒绝。另将完整查询回显拼在合法反馈前，`find_frame()` 返回功能码 `0x0E` 的回显，查询立即失败，没有继续寻找后面的功能码 `0x02` 反馈。
- **影响**：允许的特定 ID/角度组合无法读取；有查询回显或混入其它合法帧时，可能错误判断设备离线。伪终端完整 Python/C/C++ 路径中，ID 1/2 正常连接，ID 120/121 返回上述有效角度却报 `yaw read failed`，ID 1/2 加回显同样失败。没有证据证明当前真机适配器存在回显。
- **改进方案**：查询入口按预期地址、反馈类型及固定九字节长度匹配；遍历候选并跳过回显/非匹配帧。在截止时间内保留未完成帧，按帧边界与 BCC 重同步。不能仅把长度顺序倒置，那不足以处理回显和多帧场景。
- **兼容性**：可在内部新增针对反馈的解析入口，保留已公开的通用解析辅助函数。修正匹配逻辑不改变线上协议；公开头文件中的行为变化需在发布说明中注明。
- **验证方法**：黄金向量必须包含上述 ID 120 帧、ID 1/2 普通反馈、载荷内 `0x7A/0x7B`、回显加反馈、其它地址帧加目标帧、噪声、分片和坏 BCC 后合法帧。附录 C 为最小解析复现，附录 D 为完整绑定路径复现。

### F04 / P1：pitch 可生成越过当前装置边界的目标

- **位置**：[UI 角度推进](../examples/ui.py#L43)，第 43–53、119–120、298–316 行；[调参边界](../examples/tune.py#L24)，第 24–29 行；[SDK 单轴目标](../src/gimbal.cpp#L187)，第 187–190 行。
- **事实/复现**：UI 两轴滑块范围都是 ±180°，且共同采用最短角度推进；SDK 低层 setter 只检查有限数值和使能标记。调用 `advance_angle(-170, 40, 10)` 得到 `180.0`。即使两个端点都落在调参工具的目标范围内，中间目标也会进入 pitch 正方向的禁区；限制终点本身不能解决路径问题。附录 B 只执行纯函数，没有运动电机。
- **影响**：UI 未落实仓库已记录的 pitch 约 43° 低头极限，可能下发超界中间目标。复现确认的是生成的目标值，不证明真机从 −170° 出发的可达性或碰撞结果。
- **改进方案**：给当前装置提供显式轴配置，包含有符号角度约定、边界和循环/非循环路径策略；pitch 按有符号距离推进，校验起点、终点和量化后的每个中间目标，拒绝越界操作。当前装置可采用 40° 目标上限，但未知的抬头范围仍需标定。保留 SDK 立即下发的低层定位，在上层受约束控制入口实施设备策略。
- **兼容性**：UI 接受范围收窄需要告知用户；不将当前装置边界写进通用协议函数，也不静默裁剪后声称目标已执行。轴配置通过新增上层接口提供，旧低层 API 的语义仍应明确。
- **验证方法**：覆盖 −170° 到 40°、正负 180°、边界量化、反向运动、初始反馈越界和运行中配置变化；验证所有中间目标合法，yaw 循环策略独立。真机验证从小范围开始，机械边界与单圈固件最短路径行为须共同核对。

### F05 / P2：Python 配置校验未落实整型契约

- **位置**：[Config._validate](../python/f32c/__init__.py#L35)，第 35–47 行；[构造 C 配置](../python/f32c/__init__.py#L167)，第 167–172 行；[运行时 PI 校验](../python/f32c/__init__.py#L202)，第 202–208 行。
- **事实/复现**：`Config(accel=0.5)._validate()` 成功，但构造 `_CConfig` 时抛出 `TypeError: 'float' object cannot be interpreted as an integer`。范围比较同样允许部分浮点数和布尔值，而 `set_pi()` 已严格拒绝非 `int` 和布尔值。
- **影响**：配置错误绕过 SDK 的参数错误契约，直到绑定转换才以不同异常暴露；接口之间的有效输入范围不一致。
- **改进方案**：全部整型配置先执行显式类型检查，再做范围检查；默认采用与 `set_pi()` 一致的 Python 原生 `int` 契约，拒绝 `bool`，错误包含字段名。建立 C++/C/Python 全字段默认值及 ctypes 布局一致性检查，避免只比较少数字段。
- **兼容性**：文档中的合法整数输入不受影响；此前偶然被接受的布尔值或非整型输入会更早得到 `ValueError`。若未来支持其它整数标量，应在单独的规范化入口中明确转换规则。
- **验证方法**：逐字段覆盖浮点、布尔、负值、上下边界、超宽整数、字符串及 NaN；参数失败应在设备打开之前发生。附录 B 给出最小复现。

### F06 / P2：Release 构建可能让测试不执行关键操作

- **位置**：[串口日志测试](../tests/test_serial_log.cpp#L15)，第 15–41 行；[SDK API 测试](../tests/test_sdk_api.cpp#L10)，第 10–30 行。
- **事实/复现**：打开伪终端、打开串口、收发和多数 SDK 调用都写在 `assert(...)` 内。定义 `NDEBUG` 会连表达式求值一起移除。以 `-DNDEBUG` 分别编译并运行两项测试，仍返回 0，日志测试仍打印 `serial log passed`；静态代码确认这些关键操作已不执行。
- **影响**：Release/带 `NDEBUG` 的发布验证可能假通过，不能作为功能正确或串口日志有效的证据。
- **改进方案**：使用始终执行表达式并返回非零失败码的检查辅助函数，将设备初始化和操作从 `assert` 内移出。无需为此引入大型测试框架。Python 的独立烟雾脚本也应避免在开启优化时依靠 `assert` 作为唯一验收。
- **兼容性**：只改变测试保障，不改变 SDK API。发布流水线应同时覆盖 Debug 和 Release，而不是只通过取消 `NDEBUG` 掩盖测试写法。
- **验证方法**：Debug、Release 都执行相同关键操作；对故意错误的预期向量、默认值和日志文本，两种构建都必须非零退出。执行命令见附录 A 的 `-DNDEBUG` 检查。

## 3. 工程架构与 API 改进建议

### A01 / P2：让单一执行线程拥有串口，并明确控制周期预算

- **位置**：[UI 定时器](../examples/ui.py#L278)，第 278–285、298–316、380–389、465–492 行；[总线等待常量](../src/motor.cpp#L12)，第 12–23、49–65 行；[自动调参采样循环](../examples/autotune.py#L90)，第 90–131 行。
- **事实/依据**：电机读取、目标写入、相机抓帧及图像复制都由 UI 槽函数同步完成。双轴目标固定包含两次 8 ms 等待；反馈还包含串行查询及接收空闲等待。立即返回反馈的伪终端中，连续十次 `set_deg + read` 首次测得平均 **24.87 ms（约 24.9 ms）**，已超过 50 Hz 循环的 20 ms 预算；附录 D 可重复测量。该值是当前主机的软件路径样本，不是真机吞吐上限，UI 也没有每 20 ms 读取两轴，不能据此断言 UI 恒定只能达到某一频率。
- **影响**：同步 I/O 使 UI 响应、运动推进及关闭时机受相机和串口延迟影响；调参实际采样率可能低于请求值。SDK 文档已规定单实例串行访问，直接给每轴各加线程会破坏共享总线的查询配对。
- **建议方案**：一个电机执行线程拥有句柄和完整请求/响应事务；UI 只提交最新目标及显示快照，模式、PI、零点和清理操作串行执行。相机使用独立采集线程和有界最新帧交接。关闭先停止接受目标，再调度有界失能和资源释放。记录实际周期、长尾延迟、超时及反馈年龄，分别预算目标推进和反馈频率；保持无排队积压的目标覆盖策略。
- **兼容性/代价**：优先在上层控制服务实现，不强制将同步低层 SDK 全部异步化；需统一线程归属和取消流程。保持现有立即下发 API。8 ms 间隔如要缩短或配置化，必须单独经过真机总线验证。
- **验证方法**：慢相机、丢包和迟到反馈条件下检查 UI 可交互、总线事务不交错、目标无积压、退出有界，并报告实际频率及分位延迟；硬件验证单独记录负载条件。

### A02 / P2：复用绑定和运动辅助逻辑，缩小 UI 职责

- **位置**：[UI ctypes 绑定](../examples/ui.py#L56)，第 56–108 行；[Python SDK 绑定](../python/f32c/__init__.py#L132)，第 132–146、245–259 行；[工具之间的导入](../examples/autotune.py#L18)，第 18 行；[角度和量化](../examples/tune.py#L32)，第 32–38 行。
- **事实/依据**：UI 再定义相机结构、库加载和电机函数签名，并自行管理句柄；SDK 已提供这些能力。UI 和调参工具分别实现角度推进、角度区间及量化逻辑；`autotune` 又通过导入交互工具 `tune` 获取公共辅助函数。UI 缺少 SDK 级错误文本呈现，策略也已在 pitch 路径上产生分歧。
- **影响**：新 API、字段布局、加载路径和故障处理容易在两套绑定间漂移，改动需要同时理解 UI、FFI 和控制策略。
- **建议方案**：UI 使用公开的 `f32c.Gimbal2Dof`、`ZedMini`、`Frame` 和枚举，通过 A01 的控制服务管理生命周期。将工具共享的量化、路径推进、限位检查和统计提取为不依赖 Qt/命令行的轻量辅助模块；GUI 负责展示、输入及对危险操作的明确交互。
- **兼容性/代价**：保持现有示例入口和 SDK 立即下发语义；不在基础 `set_deg()` 中隐式增加后台线程或平滑。原生相机的借用图像和 Python 自持有字节契约须保持清楚。
- **验证方法**：复用现有模拟对象，验证连接、关闭、使能默认保持、UI 默认平滑回零、相机自动曝光及错误呈现；共享模块覆盖边界和量化。重构时以最终行为为验收，不仅比较函数名称。

### A03 / P2：提供结构化错误及带时序的每轴反馈

- **位置**：[公共 State](../include/f32c/gimbal.hpp#L30)，第 30–34 行；[两轴读取](../src/gimbal.cpp#L195)，第 195–209 行；[C 读取/错误入口](../src/c_api.cpp#L87)，第 87–115 行；[Python State](../python/f32c/__init__.py#L50)，第 50–53、183–193 行。
- **事实/依据**：操作返回 bool/0/1 加字符串；串口错误常被概括为某轴“read failed”。C 桥中的空输出参数等失败不保证更新错误文本。C++ 读取失败时可能留下部分角度，但只有整体 `ok`；C/Python 只交付整体成功。反馈不包含每轴采样时刻。
- **影响**：调用方难区分参数错误、未连接、超时、设备断开和部分完成，无法可靠估算串行两轴快照的时间偏差。没有每轴有效性时，不能用角度 0 推断某轴有效。
- **建议方案**：新增只读错误信息入口，至少表达错误码、操作、轴、底层系统错误和可供人阅读的文本；Python 异常保留这些属性。新增反馈入口，交付每轴有效性、角度和主机单调时钟采样时间，并明确时间戳代表接收/读取完成而非编码器同步采样。相机时间戳的时钟域须单独标识，不能直接当作与主机时间同源。
- **兼容性/代价**：保留现有 `read()` 的全成功契约及 `F32CError` 继承关系；使用新增结构和入口，不直接改变现有 C 结构或 C++ `State` 的二进制布局。结构化错误也不能追溯证明电机接受了无 ACK 的写指令。
- **验证方法**：yaw 成功/pitch 超时、反向故障、空参数、设备断开、迟到反馈；检查旧入口行为与新增字段、错误文本生命周期、时间戳顺序及有效性掩码。

### A04 / P2：明确部分完成与保存副作用，保护 C 边界和 ABI

- **位置**：[多轴目标与运行时 PI](../src/gimbal.cpp#L149)，第 149–175、212–217 行；[单轴四帧 PI](../src/motor.cpp#L41)，第 41–46 行；[C 配置](../include/f32c/c_api.h#L11)，第 11–26 行；[C 创建入口](../src/c_api.cpp#L40)，第 40–68 行；[相机 C 桥](../src/zed_c_api.cpp#L30)，第 30–75 行。
- **事实/依据**：双轴目标、四帧 PI 和置零/保存都是顺序操作。PI 中途失败后，内存配置不更新，但电机可能已收到前几个增益；`set_zero()` 同时调用两轴参数保存。SDK 文档已告知这些非原子语义，并警告置零可能保存当前实验参数，当前协议不提供软件事务回滚。C 桥直接使用 C++ 分配、字符串和对象方法，没有统一异常捕获；当前 `f32c_config` 没有大小或版本字段。
- **影响**：调用方只看到整体失败，难确定操作进度；API 名称不足以提醒保存全部参数的副作用。未来直接在现有配置尾部加字段会破坏旧调用方提供的布局。C++ 异常逃逸 C/ctypes 边界是潜在进程终止路径，本次未注入内存分配或 ZED 异常证明具体终止行为。
- **建议方案**：新增操作结果记录已经发送的轴/步骤，在部分更新后进入需恢复或重新初始化的明确状态；将临时置零与显式保存参数提供为独立的新入口，旧组合入口明确说明副作用。统一 C 入口异常屏障，将异常映射为失败和可查询错误，内存不足时采用不依赖进一步动态分配的兜底文本。增加配置时定义新的版本化结构及新打开入口，含大小/版本标识，保留旧 `f32c_config` 原样。
- **兼容性/代价**：旧 API 的原符号、签名和返回约定保留；新版能力通过新增入口演进。无法回滚的固件操作不能包装成“全部成功或无变化”的事务，也不能添加未在协议中存在的确认帧。
- **验证方法**：在 PI 每一帧、双轴目标第二轴、置零每一步和保存每一轴注入失败，检查结果进度及错误。旧 C 客户端与新库互操作，新 Python 绑定加载旧库时产生清楚的能力错误；模拟 C++/相机异常，确认不逃逸 C ABI。保存的实际范围和掉电结果另作真机验证。

### A05 / P2：完善可消费的构建、打包与无硬件 CI

- **位置**：[CMake 配置](../CMakeLists.txt#L7)，第 7–10、24–46、63–83 行；[打包脚本](../package_sdk.sh#L16)，第 16–21 行；[包配置](../cmake/f32cConfig.cmake.in)；[Python 烟雾测试](../tests/test_python_sdk.py)。
- **事实/依据**：测试和示例无条件加入构建；C++17 通过工程全局变量设定，导出目标未声明公共语言要求。`include/f32c/protocol.hpp` 位于公共 include 树，readme 也将其列入接口，但安装清单缺少它，现有 wheel 中同样没有该头。相机在 CMake 中可选，`package_sdk.sh` 却固定开启 ZED。现有打包检查运行原生、运行时 PI 和调参测试，未执行 Python 安装烟雾或 UI 默认行为测试；仓库没有 `.github` CI 配置。
- **影响**：下游只消费 SDK 时仍承担测试/示例构建；构建目录与安装后接口不一致；仅电机用户打包仍需相机开发依赖。源码检查通过无法替代安装后导入与 CMake 消费验证。
- **建议方案**：使用 `include(CTest)` 和 `BUILD_TESTING` 控制原生测试，新增示例构建开关，并以 `target_compile_features(... PUBLIC cxx_std_17)` 表达要求。按现有公开定位安装 `protocol.hpp`；未来若要收为内部接口，应另行明确迁移。打包脚本提供显式“仅电机”选项，保留当前完整包默认值。CI 核心任务覆盖 Debug/Release、伪终端绑定、调参、Qt offscreen、仅电机 wheel 安装和最小 CMake 消费工程；ZED 构建及真机任务放在具备 SDK/设备的独立运行环境。
- **兼容性/代价**：保留默认完整打包行为及现有 CMake 目标名称，新增选项不强迫老用户迁移。源码版本、wheel 内容与构建元数据要能对应；本次确认 wheel 中 Python 绑定与源码一致，不宣称已有版本漂移。Linux 平台标签本身不承诺任意发行版兼容，需记录原生依赖及支持环境。
- **验证方法**：测试/示例开关四种组合；安装前后头文件一致性；干净环境仅电机 wheel 导入及 CMake 编译；完整包有/无 ZED SDK 场景；旧 wheel/新绑定能力检查。常规 CMake、完整 UI 和 ZED 构建在本环境未验证。

## 4. 整改顺序与验收

| 阶段 | 内容 | 完成条件 |
| --- | --- | --- |
| 1：失败路径与边界 | F02 → F01；F04 可独立处理 | 通信等待有界；失败不能显示成功；清理错误可观察；pitch 起终点及中间目标符合设备策略 |
| 2：反馈与测试保障 | F03、F05、F06 | 特定 ID/回显/分片反馈可读；错误类型一致；Debug/Release 都执行真实验收 |
| 3：架构整理 | A01、A02 | 总线单一归属；UI/相机解耦；共享策略有行为测试；记录实际周期及关闭延迟 |
| 4：接口及发布 | A03、A04、A05 | 增量接口保持 ABI；部分完成可表达；安装消费和无硬件 CI 可重复执行 |

每一阶段应形成可独立评审的变更。先保留现有同步低层 API，避免将机械标定、协议修正、UI 重构和大范围接口升级混在一次提交中。

### 本次已执行的检查

| 检查 | 结果与边界 |
| --- | --- |
| 核心共享库直接编译 | g++ C++17，`-Wall -Wextra -Wpedantic`，成功；不代表 CMake 导出/安装已验证 |
| `test_protocol` | 通过 |
| `test_sdk_api` | 未定义 `NDEBUG` 时通过 |
| `test_serial_log` | 未定义 `NDEBUG` 时通过，伪终端收发及日志检查有效 |
| `test_python_sdk.py` | 通过，仅电机库；相机库缺失被正常处理 |
| `test_runtime_pi.py` | 1 项通过，真实 Python/C/C++ 路径加伪终端 |
| `test_tuning.py` | 14 项通过，运动与时间均为模拟对象 |
| 故障/边界探针 | 复现 F02、F03、F04、F05 和 F01 的 UI 提示；F01 SDK 状态及清理由代码确认 |
| `-DNDEBUG` 原生测试 | 两项仍返回成功；不能算有效验收，作为 F06 证据 |
| 伪终端控制耗时 | 首次十次循环平均 24.87 ms，执行报告附录复测为 24.56 ms；测量对象为同步双轴目标加双轴读取 |
| 已有 wheel 清单 | 未安装 `protocol.hpp`；Python 绑定与当前源码一致 |

当前无可执行的 CMake/CTest、PySide6 或 ZED SDK。Python 能发现名为 `cmake` 的模块是仓库同名目录造成的命名空间，并不表示安装了 CMake 工具。本次未运行真机、完整 UI、相机取流、常规 CMake 构建或 wheel 安装后的消费工程。

### 后续真机验证边界

有界故障恢复不等于保证物理失能；机械限位、固件选择的实际路径、DMA 接收完整性、帧间隔调整、参数保存范围及掉电结果必须单独验证。保持当前已登记的方向和负载条件，在已确认的活动范围内开始实验；UI 配置和主机统计不能替代机械反馈。

## 附录 A：重建现有无硬件基线

在仓库根目录执行。产物写入新的 `/tmp` 目录；测试仅用伪终端、无效设备路径或模拟对象。

```bash
set -e
f32c_review_dir=$(mktemp -d /tmp/f32c-review-XXXXXX)
g++ -std=c++17 -Wall -Wextra -Wpedantic -pthread -fPIC -shared \
  -Iinclude -Isrc src/serial_port.cpp src/motor.cpp src/gimbal.cpp src/c_api.cpp \
  -o "$f32c_review_dir/libf32c.so"
for f32c_test in test_protocol test_sdk_api test_serial_log; do
  g++ -std=c++17 -Wall -Wextra -Wpedantic -pthread -Iinclude -Isrc \
    "tests/$f32c_test.cpp" -L"$f32c_review_dir" -Wl,-rpath,"$f32c_review_dir" \
    -lf32c -lutil -o "$f32c_review_dir/$f32c_test"
  "$f32c_review_dir/$f32c_test"
done
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=python
export F32C_LIBRARY_DIR="$f32c_review_dir"
python3 tests/test_python_sdk.py
python3 tests/test_runtime_pi.py
python3 tests/test_tuning.py
for f32c_test in test_sdk_api test_serial_log; do
  g++ -std=c++17 -DNDEBUG -Iinclude -Isrc "tests/$f32c_test.cpp" \
    -L"$f32c_review_dir" -Wl,-rpath,"$f32c_review_dir" -lf32c -lutil \
    -o "$f32c_review_dir/${f32c_test}_release"
  "$f32c_review_dir/${f32c_test}_release"
done
```

最后两项返回成功只用于复现 F06，不是正常发布验收。附录 D 复用这里导出的 `F32C_LIBRARY_DIR`。

## 附录 B：不依赖 Qt 的 UI 和参数复现

在仓库根目录执行。通过 AST 提取实际函数，不加载 GUI，也不创建原生设备句柄。

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=python python3 - <<'PY'
import ast
from pathlib import Path
from types import SimpleNamespace
from f32c import Config, _CConfig

tree = ast.parse(Path('examples/ui.py').read_text())
functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
             and n.name in ('wrap180', 'advance_angle')]
panel = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Panel')
disable = next(n for n in panel.body if isinstance(n, ast.FunctionDef) and n.name == 'disable')
namespace = {}
module = ast.fix_missing_locations(ast.Module(body=functions + [disable], type_ignores=[]))
exec(compile(module, 'examples/ui.py', 'exec'), namespace)
print('pitch -170 -> 40, max_step 10:', namespace['advance_angle'](-170, 40, 10))
messages = []
fake = SimpleNamespace(lib=SimpleNamespace(f32c_disable=lambda _: 0), handle=1,
                       armed=True, status=SimpleNamespace(setText=messages.append))
namespace['disable'](fake)
print('disable failed; armed/display:', fake.armed, messages[-1])
config = Config(accel=0.5)
config._validate()
print('Config(accel=0.5) passed validation')
try:
    values = [None if name == 'device' else getattr(config, name)
              for name, _ in _CConfig._fields_]
    _CConfig(*values)
except TypeError as exc:
    print('ctypes conversion failed:', exc)
PY
```

基线输出包括 `180.0`、`False 失能`、校验通过后 ctypes 类型错误。

## 附录 C：反馈解析与持续背压复现

在仓库根目录执行。程序只打开伪终端；包装的系统调用固定返回 `EAGAIN`，父进程在 0.5 秒后终止探针，避免无限挂起。

```bash
f32c_probe_dir=$(mktemp -d /tmp/f32c-review-probe-XXXXXX)
g++ -std=c++17 -pthread -Iinclude -Isrc -x c++ - src/serial_port.cpp \
  -lutil -Wl,--wrap=write -o "$f32c_probe_dir/probe" <<'CPP'
#include "f32c/protocol.hpp"
#include "serial_port.hpp"
#include <pty.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <string>
extern "C" ssize_t __wrap_write(int, const void*, size_t) {
    errno = EAGAIN;
    return -1;
}
int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "backpressure") {
        int master, slave;
        char path[128]{};
        if (openpty(&master, &slave, path, nullptr, nullptr)) return 2;
        close(slave);
        f32c::SerialPort port;
        if (!port.open(path)) return 3;
        return port.write_all(f32c::cmd_disable(1)) ? 0 : 1;
    }
    const uint8_t reply[] = {0x7A, 0x78, 2, 0, 0, 0, 0x7B, 0x7B, 0x7B};
    auto frame = f32c::find_frame(reply, sizeof(reply));
    std::printf("ID 120: data bytes=%zu; feedback valid=%d\n",
                frame ? frame->data.size() : 0,
                frame && f32c::feedback_i32(*frame) ? 1 : 0);
    auto combined = f32c::cmd_feedback_mech(1);
    const uint8_t data[] = {0, 0, 0, 100};
    auto expected = f32c::pack(1, 2, data, 4);
    combined.insert(combined.end(), expected.begin(), expected.end());
    frame = f32c::find_frame(combined.data(), combined.size());
    std::printf("Echo + reply: first func=%u; expected feedback func=2\n",
                frame ? frame->func : 0);
}
CPP
"$f32c_probe_dir/probe"
python3 - "$f32c_probe_dir/probe" <<'PY'
import subprocess
import sys
try:
    result = subprocess.run([sys.argv[1], 'backpressure'], timeout=0.5)
    print('returned:', result.returncode)
except subprocess.TimeoutExpired:
    print('persistent EAGAIN: still blocked after 0.5 s; probe terminated')
PY
```

基线解析输出为两字节数据、反馈无效、首帧功能码 14；持续背压输出为超时终止。

## 附录 D：完整绑定路径与周期测量

先在同一 shell 执行附录 A，保留其库目录环境变量，再运行下列脚本。虚拟设备固定回报 12.3°，未连接真实串口。绝对耗时随主机调度变化；失败场景不是吞吐测量。

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=python python3 - <<'PY'
import os
import select
import threading
import time
from f32c import Config, F32CError, Gimbal2Dof

def probe(yaw, pitch, echo=False):
    master, slave = os.openpty()
    stop = threading.Event()
    errors = []
    def emulate():
        buffered = bytearray()
        try:
            while not stop.is_set():
                if not select.select([master], [], [], 0.02)[0]:
                    continue
                buffered.extend(os.read(master, 128))
                while len(buffered) >= 3:
                    size = 6 if buffered[2] == 14 else 5 if buffered[2] in (5, 6, 8, 10) else 7
                    if len(buffered) < size:
                        break
                    frame = bytes(buffered[:size])
                    del buffered[:size]
                    if frame[2] == 14:
                        payload = bytes([0x7A, frame[1], 2, 0, 0, 0, 123])
                        checksum = 0
                        for value in payload:
                            checksum ^= value
                        reply = payload + bytes([checksum, 0x7B])
                        os.write(master, (frame if echo else b'') + reply)
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=emulate)
    worker.start()
    motor = None
    try:
        motor = Gimbal2Dof(Config(device=os.ttyname(slave), id_yaw=yaw, id_pitch=pitch))
        try:
            motor.open()
            print(f'IDs {yaw}/{pitch}, echo={echo}: connected')
            motor.enable()
            started = time.monotonic()
            for _ in range(10):
                motor.set_deg(1, 1)
                motor.read()
            print('set_deg + read mean ms:', round((time.monotonic() - started) * 1000 / 10, 2))
        except F32CError as exc:
            print(f'IDs {yaw}/{pitch}, echo={echo}:', exc)
    finally:
        try:
            if motor is not None:
                motor.close()
        finally:
            stop.set()
            worker.join(timeout=1)
            os.close(master)
            os.close(slave)
        if errors:
            raise RuntimeError(errors)

probe(1, 2)
probe(120, 121)
probe(1, 2, echo=True)
PY
```

基线中 ID 1/2 无回显成功，另外两个场景均报 `yaw read failed`。此脚本是审查探针，打印基线缺陷；后续正式回归测试应改为明确断言正确解析与连接成功。
