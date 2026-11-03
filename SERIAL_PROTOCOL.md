# 六电机串口协议 v1（上位机实现交接）

本文对应 `feat/uart-host-control` 的下位机实现：`UserCode/communication/host_wire.*`、`host_uart.*`、`host_control.*`，控制动作复用 `UserCode/device/dm_position_test.*`。这是二进制协议，不接收文本命令。本文不要求修改或启动任何上位机工程。

## 1. 串口与接线

- USART1，**115200 baud、8 数据位、1 停止位、无校验、无流控**；PA9=MCU TX、PA10=MCU RX。
- 上位机 TX 接 MCU RX，上位机 RX 接 MCU TX，共地，使用与板卡逻辑电平兼容的 USB-UART；不能直接连接 RS-232 电平。
- 下位机 RX：DMA1 Stream0、Circular、字节、Memory Increment；TX：DMA1 Stream1、Normal、字节、Memory Increment。USART1 和两个 DMA IRQ 均为抢占优先级 6，低于电机/CAN 的 5。
- 软件注册 HAL 回调，使用 Receive-to-IDLE DMA，保留半传输/全传输事件；**IDLE、DMA 回调、一次 read() 均不是帧边界**。
- RX DMA 512 B，软件 RX 环 2048 B；TX DMA 64 B，只有发送完成或确认中止后才能复用。两块 DMA 存储共 576 B，32 B 对齐，位于现有 MPU 非缓存的 `0x24000000` AXI 区；不使用 DTCM 做 DMA。
- 工具未烧录或验证本轮串口实板链路。外设初始化、IRQ 开关和 MPU 属性由现有 CubeMX 生成配置提供，不由协议层补配置。

## 2. 电机编号、单位和范围

**motor 字段使用 CAN ID，不是数组索引。** `0xFF` 只用于指定的全局/广播命令。

| 索引 | 部位 | 型号 | motor/CAN ID | 反馈 Master ID | 初始工作范围 |
|---:|---|---|---|---|---|
| 0 | 右前伸缩 | DM-S3519 | 0x0A | 0x2A | −30°～+30° |
| 1 | 中伸缩 | DM-S3519 | 0x0B | 0x2B | −30°～+30° |
| 2 | 后伸缩 | DM-S3519 | 0x0C | 0x2C | −30°～+30° |
| 3 | 左前旋转 | DM-J4340P | 0x0D | 0x2D | −180°～+180° |
| 4 | 左前伸缩 | DM-S3519 | 0x0E | 0x2E | −30°～+30° |
| 5 | 右前旋转 | DM-J4340P | 0x0F | 0x2F | −180°～+180° |

位置和目标：相对当前软件零点的**输出轴度数**；Move 是绝对目标，不是相对增量。速度反馈单位 **rpm**，轨迹速度参考/上限为 **deg/s**，加速度为 **deg/s²**，jerk 为 **deg/s³**，力矩为 **Nm**。不要自行换算 S3519 内部减速比，也没有毫米/丝杠导程换算。CAN ID、Master ID、模式、PMAX/VMAX/TMAX 不可通过本协议修改。

上电保持失能，收到 50 次新的失能、静止、位置稳定反馈后自动建立软件零点并校验默认限值。初始速度/加速度/jerk 为 `10/20/100`。协议可编码范围、软件工作限位和机械安全范围不是同一概念。完整默认区间超出当前绝对协议范围时拒绝应用，不截断。上位机必须查询已应用值，而非只显示本表默认值。

## 3. 固定 64 字节帧

多字节整数均为 **little-endian**；float 为 IEEE-754 binary32，小端；没有结构体对齐填充。必须显式编码，不直接发送语言对象内存布局。每帧永远 64 B，payload_length 不改变传输长度。

| 绝对字节偏移 | 长度 | 类型/内容 |
|---:|---:|---|
| 0 | 2 | 固定 `A5 5A` |
| 2 | 1 | version=`01` |
| 3 | 1 | type：请求=`00`，应答=`01` |
| 4 | 2 | uint16 sequence，由上位机提供，应答原样回显 |
| 6 | 1 | uint8 opcode，应答原样回显 |
| 7 | 1 | uint8 motor；广播应答用具体电机 ID，全局应答用 FF |
| 8 | 2 | uint16 payload_length，0～50 |
| 10 | 2 | reserved，必须为 0 |
| 12 | 50 | payload，未使用尾部必须全部为 0 |
| 62 | 2 | uint16 CRC，小端存储 |

CRC：**CRC-16/CCITT-FALSE**，poly=`0x1021`，init=`0xFFFF`，refin=false，refout=false，xorout=`0x0000`。覆盖完整字节 **0～61**，包括头、长度、保留位和零填充。标准校验向量 ASCII `123456789` → `0x29B1`；在线字节为 `B1 29`。

接收算法：累积字节，寻找 A5 5A，凑齐 64 B 后验 CRC；CRC 错丢一个字节继续同步，不能直接跳过假定的整帧。噪声、CRC 错帧、尚未完整的帧不执行、不应答。通过 CRC 但 version/type/opcode/长度/保留位/填充/电机编号错误则返回协议错误应答。应答统一使用 version=1、type=1，回显原请求 sequence/opcode/motor；没有携带 payload 的错误通知或主动遥测。

## 4. 请求、响应与序号

采用**单请求在途、停等**：发一个请求，收齐所需应答，再发下一请求；心跳、状态查询也走同一发送调度器，不得绕过在途请求并发发送。

- 下位机保留**最近一个语法有效请求**的完整 64 B 及全部应答。相同 sequence 且全帧相同：重放原应答，不再次执行；同 sequence 内容不同：返回 106，不执行。协议非法帧不替换该缓存。
- 缓存仅有一个请求，不是持久命令日志，不保证跨复位或跨其他有效请求的“恰好一次”。不要在发送别的有效请求后再重试旧 Move/SetZero。
- 新请求 sequence 建议加一，uint16 回绕；只禁止与最近缓存请求序号相同但内容不同，不要求全局单调。复位后缓存丢失，应重新握手、查询实际状态，不自动补发旧动作。
- 超时重试必须逐字节重发原帧。建议完整应答等待超时 100 ms，有限重试；重试耗尽标记本地链路故障并停止提交新动作，不将“已发送”显示为成功。运动非幂等，不要用新序号自动重试不确定是否执行的 Move/SetZero。
- 广播 State、Disable、Stop 返回 **6 帧**，同 sequence/opcode，motor 分别为 0A～0F。按 motor 去重收集；全部收到才完成事务，不把第一帧当广播完成。重发广播会重放六帧，重复应答不是新事件。
- 六帧是逐台处理/快照，不是同步采样，也没有全有或全无/回滚保证；某台 Busy 不会撤销其他台的成功。
- 缓存应答内容可能已过时：重发 State/Heartbeat 得到的是旧快照；新鲜状态必须使用新 sequence。重发 Enable 的 Ok 也不能证明当前仍使能。
- 115200/8N1 每方向有效预算为 11,520 B/s；64 B 单帧约 5.56 ms，六状态应答约 33.3 ms，含请求的全状态查询线时约 38.9 ms，另加任务/USB/系统延迟。建议全状态查询 10 Hz、心跳不超过 100 ms 间隔；不以 100/500/1000 Hz 电机内部节拍要求同频串口上报。

每个应答 payload 的前两个字节固定为 **uint16 result**。协议错误和普通动作 ACK 的 payload_length=2；查询成功使用下文专用长度。先检查 result 和长度，不能无条件按 44 B 状态解码错误应答。除广播的逐台 motor 外，请核对 CRC、version、type、sequence、opcode、motor、长度及零填充。

## 5. 命令表

以下长度仅指 payload，不包括帧头、补零及 CRC；`P+n` 表示 payload 相对偏移，绝对偏移为 `12+n`。

| opcode | 名称 | motor | 请求 payload | 成功应答 payload_length |
|---|---|---|---|---:|
| 01 | Heartbeat | FF | 空，长度 0 | 8 |
| 02 | Info | FF | 空，长度 0 | 32 |
| 10 | State | 单台或 FF | 空，长度 0 | 44；FF 为六帧 |
| 11 | Parameters | 仅单台 | 空，长度 0 | 44 |
| 20 | Enable | 仅单台 | 空，长度 0 | 2 |
| 21 | Disable | 单台或 FF | 空，长度 0 | 2；FF 为六帧 |
| 22 | Stop | 单台或 FF | 空，长度 0 | 2；FF 为六帧 |
| 23 | SetZero | 仅单台 | 空，长度 0 | 2 |
| 24 | Move | 仅单台 | P+0 float32 target_deg，长度 4 | 2 |
| 25 | SetLimits | 仅单台 | 五个 float32，长度 20，见下 | 2 |

本版无串口增益修改、直接力矩/速度控制、Flash 保存、清除电机内部错误、运行中改限位、同步六轴运动或主动连续推流命令。未知 opcode 返回 104，不允许自行扩展解释。

### Heartbeat / Info

Heartbeat 成功 payload：P+0 uint16 result=0，P+2 uint16 link_flags，P+4 uint32 uptime_ms（HAL 毫秒计数，uint32 回绕）。

Info 的前 8 B 与 Heartbeat 相同，其余：

| payload 偏移 | 类型 | 字段/当前值 |
|---:|---|---|
| 8 | uint16 | host_timeout_ms=500 |
| 10 | uint16 | trajectory_hz=100 |
| 12 | uint16 | nominal_per_motor_tx_hz=1000 |
| 14 | uint16 | grouped_tx_tick_hz=2000 |
| 16 | uint16 | feedback_check_hz=500；不是已证明的新反馈帧率 |
| 18 | uint8 | motor_count=6 |
| 19 | uint8 | CAN format=1，表示 FD+BRS |
| 20 | uint8[6] | CAN IDs：0A 0B 0C 0D 0E 0F |
| 26 | uint8[6] | Master IDs：2A 2B 2C 2D 2E 2F |

link_flags：bit0=已见有效请求，bit1=host_link_timeout，其余保留为 0。收到任何成功 Heartbeat 时，本次请求已计入 seen；**只有未命中最近请求缓存的新有效 Heartbeat 才能解除已锁存的串口超时**，不会同时清除电机 fault 或自动使能。上位机恢复时必须分配新 sequence；下位机不保存更早的序号历史。

### State

| payload 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | uint16 | result=0，只代表查询成功 |
| 2 | uint16 | flags，见下 |
| 4 | uint32 | motor_state，电机反馈状态值 |
| 8 | uint32 | feedback_count，合格新反馈帧计数，回绕 |
| 12 | float32 | position_deg，相对软件零点的当前反馈 |
| 16 | float32 | velocity_rpm，当前反馈 |
| 20 | float32 | torque_nm，当前反馈 |
| 24 | float32 | accepted_target_deg，最近被接受/保持的位置 |
| 28 | float32 | reference_deg，当前轨迹位置参考 |
| 32 | float32 | reference_velocity_deg_s，当前轨迹速度参考 |
| 36 | float32 | commanded_torque_ff_nm，发送的 MIT 前馈，不是总力矩 |
| 40 | uint16 | last_device_result，设备命令/故障结果，非本次查询 result |
| 42 | uint16 | pending_ozone_command，0 无命令；1 SetZero、2 Enable、3 Disable、4 Move、5 Stop、6 ApplyLimits |

flags：bit0 connected，bit1 zeroed，bit2 limits_configured，bit3 enabled（反馈已确认），bit4 enable_pending，bit5 busy，bit6 motion_complete，bit7 fault，bit8 host_link_timeout；bit9～15=0。

motor_state：0=Disabled，1=Enabled，8=OverVoltage，9=UnderVoltage，10=OverCurrent，11=MosOverheating，12=MotorCoilOverheating，13=Disconnected，14=Overload。connected=false 时不要把其他缓存测量或 enabled=true 当作新鲜状态。真实失能需 connected=true 且 fresh feedback 的 motor_state=0、enabled=false；串口断开、Disable ACK 或 fault 标志都不是物理失能确认。

到位需本次 Move 已 ACK Ok，随后新 State 的 busy=false、motion_complete=true、fault=false、connected=true；默认误差阈值 0.3°。Enable ACK 只表示接受请求，还要等待 enable_pending=false、enabled=true 且无 fault。运动中仅显示“已接受/运行中”，不能收到 Ok 就显示“到位”。

### Parameters

成功长度 44：P+0 uint16 result=0，P+2 uint16 flags（bit0=limits_configured，其余 0），P+4 起依次十个 float32：

`min_deg, max_deg, max_velocity_deg_s, max_acceleration_deg_s2, max_jerk_deg_s3, kp, kd, torque_ff_nm, finish_deg, stationary_rpm`。

返回的是**已应用**限值及构造时复制的控制参数，不是 Ozone 尚未提交的编辑字段。kp/kd 是电机 MIT 刚度/阻尼，不是 MCU 外部 PID；读取不代表可通过本协议修改。RAM 参数没有掉电保存。

### Enable / Disable / Stop / Move

- Enable：要求连接、新反馈电机已失能、已归零、限值与控制参数有效、位置在工作/协议范围内且静止观察满足。先预装保持位置，再在分配时隙请求使能，新反馈确认才设 enabled；使能反馈超时会锁存故障并请求失能。已经请求使能时返回 Busy。串口超时期间返回 107；Ozone 同时尝试也受禁止。
- Disable：取消轨迹/使能请求并请求电机失能，仍在其下一 CAN 时隙发送；等待实际反馈确认。不会取消已有故障锁存，也不是硬件急停。
- Stop：有连接并处于有效使能状态时取消轨迹，保持当时反馈位置；**仍使能且可有保持力矩**。失能/故障时不是可用的失能替代命令，返回 NotEnabled。
- Move：目标为当前软件坐标绝对位置，必须有限、在已应用工作范围和协议编码范围内。要求已使能、无故障、不 busy、当前静止且接近参考；S 曲线求解在任务内进行。运行中收到新 Move 返回 Busy，不抢占。NaN、无穷或越界位置返回 OutsideLimits，不截断成边界。
- 所有新动作沿用现有 MIT 直接参考跟踪，没有在串口任务建立第二个位置环/速度环。TIM7 100 Hz 采样参考，TIM6 2 kHz 轮换三台发送，每台名义 1 kHz；使能、失能、错误处理均不插入额外发送突发。

### SetZero / SetLimits

SetZero：失能、连接、静止且反馈稳定时改变**软件**零点，不写电机硬件零点/Flash。已有运动限位会平移以保持同一物理工作区，不会自动恢复对称默认值。成功后重新查询 State 和 Parameters，作废界面中旧坐标目标；不要自动再次提交旧位置。

SetLimits 请求 P+0/4/8/12/16 依次为 float32：`min_deg, max_deg, max_velocity_deg_s, max_acceleration_deg_s2, max_jerk_deg_s3`。设备要求已归零、反馈状态为 Disabled 且无使能请求；值全部有限，min<max，后三项>0，整个范围满足协议编码边界，速度也须满足协议范围。非法值返回 InvalidParameters 并保留**已应用**配置；不能仅因原来的 limits_configured=true 就判本次成功。现有接口不额外要求区间覆盖当前位置，也不单独检查 isConnected；上位机应先核对连接/新反馈，操作者明确选择工作区，之后 Enable 仍独立检查当前点在区间内。仅 RAM 生效，重启后重新按源码默认值初始化；重新设零仍有前述限位平移语义。成功后读 Parameters 确认，不能把 ACK 写成“已保存到 Flash”。

## 6. result 错误码

| 十进制 | 名称 | 语义/上位机处理 |
|---:|---|---|
| 0 | Ok | 请求已受理/查询成功，不等于动作完成 |
| 1 | Offline | 电机未连接；失联锁存也可能写入 last_device_result |
| 2 | NotZeroed | 尚未建立软件零点 |
| 3 | NotEnabled | 不满足使能状态；超时后 Ozone 使能也被禁止 |
| 4 | Busy | 运动忙、已有待处理 Ozone 命令或使能请求；不覆盖 |
| 5 | InvalidParameters | 控制参数或限值无效 |
| 6 | OutsideLimits | 当前/目标位置或位置协议范围无效，包括非法 Move float |
| 7 | NotStationary | 未满足静止/稳定条件 |
| 8 | MotorFault | 电机状态不允许，或使能确认失败 |
| 9 | PlanningFailed | 既有 S 曲线求解失败，不提交新轨迹 |
| 10 | InvalidCommand | 设备层无效命令；串口未知 opcode 通常为 104 |
| 11 | TransmitFailed | CAN 硬件提交失败锁存结果 |
| 100 | BadVersion | version 不是 1 |
| 101 | BadType | 请求 type 不是 0 |
| 102 | BadLength | 超过 50 或与 opcode 要求长度不同 |
| 103 | BadPadding | reserved 非零或 payload 未用尾部非零 |
| 104 | UnknownOpcode | opcode 不支持 |
| 105 | UnknownMotor | 非 0A～0F，或命令不支持 FF/所给目标 |
| 106 | SequenceConflict | 最近同 sequence 的完整请求不同，不执行 |
| 107 | LinkFault | 串口超时仍锁存时提交串口 Enable/Move，先新 Heartbeat 握手 |

错误检查按版本、方向、长度上界、保留/尾部、opcode、opcode 长度、motor 顺序进行；同一帧同时有多项错误时只返回首先检测到的一项。设备错误是该次请求终态；不要以同序号重试期望 Busy/NotStationary 自动变成 Ok（将重放旧结果），应查询条件后由操作决定是否发新序号。

## 7. 心跳、断线与 Ozone 交互

- 上电尚未收到第一个语法有效串口请求时，不启动串口失联计时，原 Ozone 使用仍可工作。第一个有效请求（包括查询及设备拒绝的动作）建立串口会话。
- 会话健康期间，每个语法有效且非 SequenceConflict 的请求刷新 last_valid，包含缓存重发；坏 CRC、坏版本、未知命令等协议错误不刷新。建议无论是否在运动都由上位机调度器每 100 ms 提交新 Heartbeat。
- 从最后有效请求起 **>=500 ms**，在 TIM3 500 Hz 检查路径锁存 host_link_timeout，取消六台轨迹和 Ozone 待处理命令、锁存各台 fault、请求全部失能；任务轮询也会检查。失能仍通过原 TIM6 交替发送时隙发出；时间判断不依赖阻塞中的串口任务，但仍依赖 MCU 和中断运行。
- 已超时后普通查询、Disable、Stop，以及命中最近请求缓存的 Heartbeat 重发都不会解除锁存；只有**未命中该缓存的新有效 Heartbeat**恢复串口授权，不恢复电机动作/使能/轨迹，不清除 motor fault。缓存只有一项，不追溯更早的 Heartbeat；恢复握手由上位机分配新 sequence。
- 恢复流程：新 Heartbeat → State 全查询 → 确认连通、实际失能、机械安全与静止 → 必要时查询/设置零点限值 → 操作者显式 Enable → 等反馈确认 → 由操作者提交新的 Move。界面不能在重连后自动使能或恢复未完成目标。
- Ozone 与串口由同一任务处理。若某电机已有待处理 Ozone 命令，其串口动作返回 Busy，不覆盖字段；随后处理该 Ozone 命令。失联期间禁止两种入口的 Enable/Move。保留的 Ozone 是并行控制入口，**不是串口连接后自动只读**；调试时应约定操作者所有权，不能同时发互相冲突的动作。
- UART 错误/软件 RX 溢出丢弃不可信片段并重同步；按现有配置恢复接收。TX DMA 出错须确认中止再复用存储，不假装应答已送达。没有串口日志混入二进制流，也不主动重发已丢应答；由上位机停等重试。
- Ozone 可观察 `dm_startup`、`host_link_timeout`、`host_crc_errors`、`host_rx_dropped_bytes`、`host_uart_errors`、`host_tx_errors`。统计不是串口协议字段。初始化串口启动失败时 dm_startup=SerialStartFailed，不启动电机定时器。
- 电机反馈 Watchdog 约 20 ms 与串口 500 ms 是两套独立检查。主控 Halt/断电、IRQ 长时间阻塞、物理线路故障均可能使失能无法发送；软件断线处理不是硬件安全保证。连续接收依赖半/全 DMA IRQ 及时运行；512 B 在当前波特率约 44.4 ms 填满，半缓冲约 22.2 ms，未实测最坏 ISR 延迟。

## 8. 上位机最小参考编解码（Python，无串口副作用）

下面只处理字节，不会自动连接串口或使能电机；可移植到 C#/C++/Rust。实际 read/write、重试及 UI 由上位机实现。一个线程/调度器拥有发送事务和 sequence；UI 只提交用户意图，不直接向串口并发写。

```python
import binascii
import struct


def encode(seq, opcode, motor, payload=b''):
    if not 0 <= seq <= 0xffff or len(payload) > 50:
        raise ValueError('sequence/payload')
    out = bytearray(64)
    struct.pack_into('<2sBBHBBHH', out, 0,
                     b'\xa5\x5a', 1, 0, seq, opcode, motor, len(payload), 0)
    out[12:12 + len(payload)] = payload
    struct.pack_into('<H', out, 62, binascii.crc_hqx(out[:62], 0xffff))
    return bytes(out)


class ResponseParser:
    def __init__(self):
        self.pending = bytearray()

    def feed(self, data):
        self.pending.extend(data)
        frames = []
        while True:
            pos = self.pending.find(b'\xa5\x5a')
            if pos < 0:
                # Keep a trailing A5 in case the next read begins with 5A.
                self.pending[:] = b'\xa5' if self.pending[-1:] == b'\xa5' else b''
                break
            del self.pending[:pos]
            if len(self.pending) < 64:
                break
            raw = bytes(self.pending[:64])
            if struct.unpack_from('<H', raw, 62)[0] != binascii.crc_hqx(raw[:62], 0xffff):
                del self.pending[0]
                continue
            del self.pending[:64]
            _, ver, kind, seq, opcode, motor, size, reserved = struct.unpack_from('<2sBBHBBHH', raw)
            if ver != 1 or kind != 1 or not 2 <= size <= 50 or reserved or any(raw[12 + size:62]):
                continue
            frames.append((seq, opcode, motor, raw[12:12 + size]))
        return frames


# Construction examples only; do not automatically transmit these on reconnect.
heartbeat = encode(1, 0x01, 0xff)
move_two_degrees = encode(3, 0x24, 0x0a, struct.pack('<f', 2.0))
state_all = encode(4, 0x10, 0xff)
limits = encode(6, 0x25, 0x0a, struct.pack('<5f', -30, 30, 10, 20, 100))

# Successful State payload decoding: verify opcode, motor, sequence, result and len==44 first.
# result, flags, motor_state, feedback_count, position, rpm, torque,
# accepted_target, reference, reference_velocity, torque_ff, last_result, ozone_command
# fields = struct.unpack('<HHII7fHH', payload)
# Successful Parameters payload: struct.unpack('<HH10f', payload)
```

上位机接入建议：连接后先 Heartbeat、Info、State(FF)、逐台 Parameters；UI 明确区分“离线/未归零/未应用限位/已失能/待使能/已使能/运行中/到位/故障/上位机失联”，用当前反馈决定按钮和显示，不仅使用最近 ACK。参数与零点修改只允许人工确认失能后的操作。

## 9. 黄金帧与验证边界

以下每块四行，每行 16 B，拼接成一帧；空白不在线上传输。独立 Python `binascii.crc_hqx` 计算，CRC 与本节所列 sequence/opcode/payload 一起固定。

Heartbeat 请求，sequence=1，motor=FF，CRC=0B47：

```text
a5 5a 01 00 01 00 01 ff 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 47 0b
```

Move 请求，sequence=3，motor=0A，target=2.0°，CRC=A5FB：

```text
a5 5a 01 00 03 00 24 0a 04 00 00 00 00 00 00 40
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 fb a5
```

上述 Move 的 Ok ACK，payload_length=2，result=0，CRC=0724：

```text
a5 5a 01 01 03 00 24 0a 02 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 24 07
```

其他构造校验：`encode(2,0x20,0x0A)` 末尾 FE 0A；State(FF,seq=4) 末尾 38 FC；Disable(FF,seq=5) 末尾 B5 CF；上文 limits(seq=6) 末尾 95 5E。

本轮验证：Debug 交叉构建；真实 UART DMA 适配/线协议/Control/DM/SCurve 的主机烟测，HAL/CAN/机构为边界模型，并启用 AddressSanitizer/UndefinedBehaviorSanitizer。覆盖帧错误/拆包/重同步、六状态应答、Ozone 仲裁、序号重放和冲突、Move 到位、500 ms 全失能及手动恢复、RAM 限值与软件零点、RX 溢出/ORE、TX DMA 错误及存储生命周期，另覆盖 499/500 ms 判断和 uint32 回绕。临时烟测不作为仓库永久测试入口。尚未验证本轮真实 USB-UART、板上 DMA/Cache、最坏 IRQ 延迟、电气波形、带载动作或实际失联停机时间；工具不因编译/模拟通过而宣称这些已通过。
