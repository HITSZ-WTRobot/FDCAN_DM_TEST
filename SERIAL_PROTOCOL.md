# 串口设备控制协议 v2

对应 `UserCode/communication/host_wire.*`、`host_control.*`、`host_uart.*`。本版接入六台DM（四伸缩毫米位置、两关节角位置）、双M2006和四轮全向底盘。v1不再接收：Move的伸缩单位及状态布局已改变，不能让旧客户端把角度静默解释成毫米；v1合法CRC请求返回BadVersion=100。本轮不修改/启动上位机工程，旧版“连接反馈不明显”按用户要求留待上位机检查，不宣称该现象已定位或修复。

## 1. 物理接口和通信方式

USART1：**115200 baud、8数据位、1停止位、无校验、无流控**，PA9=MCU TX、PA10=MCU RX。USB-UART交叉连接TX/RX，共地，使用板卡兼容逻辑电平，禁止直接RS-232电平。RX=DMA1 Stream0 Circular，TX=Stream1 Normal；UART和两路DMA IRQ优先级6/0。电机/CAN定时IRQ为5/0。

固定64字节二进制帧；不混入文本日志。RX DMA512字节、软件环2048字节，TX DMA64字节，专用DMA缓冲在现有非缓存AXI区。DMA半传输/全传输/IDLE事件均保留；IDLE、USB read()或一次DMA回调不是帧边界。TX完成或确认中止前不复用发送缓冲。

采用**单请求在途、停等**。客户端拥有唯一发送调度器和sequence，收齐该请求全部应答后才发下一个请求；心跳也通过该调度器。单帧在线约5.56ms，九设备状态广播含请求的纯线时约55.6ms，另有任务/USB延迟。建议全设备状态5～10Hz、空闲时每100ms新心跳，完整事务等待超时150ms；这些是客户端建议，不是固件重试保证。广播收齐期间不要并发插入心跳或运动命令。

## 2. 逻辑设备编号

编号与opcode是不同字段；底盘及M2006的逻辑编号**不是原始CAN ID**。FF只用于全局命令或明确支持的广播，不是单台编号。

| 逻辑编号(hex) | 设备 | kind | unit | CAN总线/实际编号 |
|---|---|---:|---:|---|
| 0A | 右前伸缩DM-S3519 | 1 | 2=mm | FDCAN1/CAN0A，反馈2A |
| 0B | 中伸缩DM-S3519 | 1 | 2=mm | FDCAN1/CAN0B，反馈2B |
| 0C | 后伸缩DM-S3519 | 1 | 2=mm | FDCAN1/CAN0C，反馈2C |
| 0D | 左前关节DM-J4340P | 2 | 1=deg | FDCAN1/CAN0D，反馈2D |
| 0E | 左前伸缩DM-S3519 | 1 | 2=mm | FDCAN1/CAN0E，反馈2E |
| 0F | 右前关节DM-J4340P | 2 | 1=deg | FDCAN1/CAN0F，反馈2F |
| 21 | M2006第一台 | 3 | 1=deg | FDCAN3/C610 ID1，反馈201 |
| 22 | M2006第二台 | 3 | 1=deg | FDCAN3/C610 ID2，反馈202 |
| 40 | 整个全向底盘 | 4 | 3=body | FDCAN2，四轮0B/0A/09/0C |

41～44只作为四轮状态应答的轮位标签，不接受直接单轮动作。轮位顺序固定为右前、左前、左后、右后，反馈Master ID为2B/2A/29/2C。Info返回的九设备顺序就是上表顺序。

位置语义：所有Move都是**相对当前软件零点的绝对目标**，不是增量。DM伸缩按用户标称226.2mm/输出轴圈换算，不再额外乘除内置减速比；不设软件行程区间，但当前PMAX/19.203坐标映射仍只能表达协议原点附近约±23.434mm，任意软件零点下的剩余正负行程不一定对称。不能因226.2mm/圈就发送226.2mm并期待成功，超出可编码范围返回OutsideLimits，不截断或修改PMAX。

DM关节每次软件零点建立后为−180～+180deg；M2006为0～360deg（内置36:1减速后的输出轴），不取最短角或把360取模为0。默认正向均未经本轮实板验证。

## 3. 帧格式和CRC

每帧永远64B，整数little-endian，float为IEEE754 binary32 little-endian；不发送语言结构体的填充内存。

| 绝对偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 2 | A5 5A |
| 2 | 1 | version=02 |
| 3 | 1 | type=00请求、01应答 |
| 4 | 2 | uint16 sequence，应答回显 |
| 6 | 1 | opcode，应答回显 |
| 7 | 1 | 逻辑设备编号/FF；广播应答为具体编号 |
| 8 | 2 | uint16 payload_length，0～50 |
| 10 | 2 | reserved=0 |
| 12 | 50 | payload + 未用尾部全零 |
| 62 | 2 | uint16 CRC，小端 |

**CRC-16/CCITT-FALSE**：poly1021、initFFFF、refin=false、refout=false、xorout0000，覆盖字节0～61（包含零填充）。ASCII `123456789` →29B1，在线B1 29。

接收累计数据，寻找A5 5A，凑满64B验CRC；失败丢一字节继续同步，不能跳过假定整帧。噪声/坏CRC/半帧不执行、不应答。CRC通过但版本、方向、长度、保留位、填充、opcode或目标不合法时返回错误应答。检查顺序是版本→type→长度上界→保留/尾部→opcode/设备适用性→opcode精确长度；多个错误只报先检测到的一个。

应答payload前两字节固定**uint16 result**。动作ACK和所有错误应答长度2；查询成功按下文布局。客户端先验CRC/方向/版本/sequence/opcode/设备，再看result与长度；不能按成功查询格式解析2B错误应答。应答保留位及未用尾部为0。

## 4. 序号、重发和缓存

- 保存最近一个**语法有效**请求的全部64B和全部应答。相同sequence且全帧相同：重放原应答，不再次执行（包括SetZero、Enable、Move、BodyVelocity）；相同sequence内容不同：106，不执行。
- 只缓存最近一项，不保存更早历史，不跨复位持久化。不要先发其他有效请求再重发旧Move/SetZero。新请求sequence建议加一，uint16回绕；不要求全局严格单调。
- 超时重试必须逐字节重发原帧；建议有限重试，耗尽后显示链路故障，不自动用新序号重发不确定是否执行的动作。Busy/NotStationary等是本次终态，重发同帧仍返回原错误；条件满足后需由操作者决定新序号请求。
- FF的State/Disable/Stop返回**9帧**，按九设备编号去重收集；Wheels(40)返回**4帧**，编号41～44。重发会重放全部帧，重复应答不是新事件。九台动作逐台处理，没有同步启动、原子回滚或全有全无保证。
- 重放State/Info/Heartbeat是旧快照，不代表此刻仍运行/连接。读新状态必须新sequence。收到Move/Enable的Ok只表示受理，不是到位/实际使能。

## 5. 命令

下表长度仅payload；`P+n`为payload相对偏移，绝对偏移为12+n。所有float请求必须使用所选设备的单位。

| opcode(hex) | 操作 | 设备 | 请求长度/内容 | 成功应答长度 |
|---|---|---|---|---:|
| 01 | Heartbeat | FF | 0 | 8 |
| 02 | Info | FF | 0 | 42 |
| 10 | State | 单台或FF | 0 | DM48 / M2006 32 / 底盘44；FF九帧 |
| 11 | Parameters | 单台 | 0 | DM48 / M2006 50 / 底盘30 |
| 12 | Wheels | 40 | 0 | 四帧，每帧20 |
| 20 | Enable | 单台 | 0 | 2 |
| 21 | Disable | 单台或FF | 0 | 2；FF九帧 |
| 22 | Stop | 单台或FF | 0 | 2；FF九帧 |
| 23 | SetZero | DM或M2006单台 | 0 | 2 |
| 24 | Move | DM或M2006单台 | 4：P0 float绝对位置，伸缩mm，其余deg | 2 |
| 25 | SetLimits | DM单台 | 伸缩12：v/a/jerk（mm/s、mm/s²、mm/s³）；关节20：min/max/v/a/jerk（deg及相应时间单位） | 2 |
| 26 | BodyVelocity | 40 | 12：P0 vx m/s、P4 vy m/s、P8 wz deg/s | 2 |

不支持广播Enable、底盘SetZero/Move、M2006/底盘SetLimits、直接单轮控制、世界系/整车位置导航、参数Flash保存、运行中自动恢复、速度PID在线改写或新上位机协议扩展。

### 动作规则

- **Enable**：先确认连接、零点/参数、静止/失能等设备条件。DM先预装保持目标再请求使能；底盘由真实库发四台使能请求并等待四个新状态确认，enable_pending时BodyVelocity返回Busy；M2006 enabled是本地控制器授权，C610没有物理使能应答。上电不会自动使能。
- **Disable**：DM下一各自TIM6时隙发送失能，底盘下一TIM3时隙四台失能，M2006清电流缓存并在下一1kHz聚合发送零。ACK不是电气/机械停机确认。
- **Stop**：DM/M2006取消曲线并保持当前位置，仍可产生保持力矩，不是平滑减速或失能。底盘只把车体系速度设零，保持使能，不作位置锁定。未使能设备可返回NotEnabled，FF Stop可出现部分成功。
- **SetZero**：仅连接且失能、静止稳定时接受。上电已有稳定样本自动首次归零；后续软件设零由操作者明确请求。DM关节重新建立新原点两侧各180°，伸缩只改变毫米原点；M2006重新建立0～360°。改变允许运动的实际区域，操作者确认空间。不会写电机零点寄存器或Flash、不会自动运动/使能。DM若新完整关节区间无法编码则拒绝，保留旧零点和有效状态。
- **Move**：有限且满足工作/协议范围，已使能、连接有效、无fault。运动忙返回Busy，保留旧曲线/目标；规划失败也不替换旧轨迹。MM转换和轨迹执行使用已有设备API，不在通信层另建曲线或PID。
- **SetLimits**：失能状态提交；非法值保留已应用配置。伸缩仅调整轨迹动态参数，没有min/max行程字段；关节范围必须在−180～+180°以内且完整可编码。重设关节零点后重新采用完整±180°。仅RAM生效。
- **BodyVelocity**：x向前、y向左、wz逆时针为正；上限分别0.1m/s、0.1m/s、10deg/s，轮速上限30rpm，允许的矩形组合最大包络约29.3406rpm。非有限值/超限拒绝并保留旧参考，不缩放。实际轮向需架空验证。

## 6. Heartbeat / Info

Heartbeat成功：P0 uint16 result，P2 uint16 link_flags，P4 uint32 uptime_ms（HAL时间，uint32回绕）。link_flags bit0=已见有效串口请求，bit1=串口超时锁存。

Info前8B同Heartbeat，其余：

| P偏移 | 类型 | 字段 |
|---:|---|---|
| 8 | u16 | 失联超时ms=500 |
| 10 | u8 | 逻辑设备数=9 |
| 11 | u8 | 协议版本=2 |
| 12 | u8[9] | 编号：0A 0B 0C 0D 0E 0F 21 22 40 |
| 21 | u8[9] | kind：1 1 1 2 1 2 3 3 4 |
| 30 | u16 | 轨迹采样Hz=100 |
| 32 | u16 | 位置/反馈检查Hz=500 |
| 34 | u16 | M2006速度PIDHz=1000 |
| 36 | u16 | 底盘周期参考Hz=500 |
| 38 | u32 | UART波特率=115200 |

这些是固件调度节拍，不是实测反馈带宽、CAN线上完成率或串口上报频率。

## 7. State

所有成功State共享前16B：

| P偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u16 | 本次查询result=0 |
| 2 | u16 | flags |
| 4 | u8 | kind（1伸缩、2关节、3M2006、4底盘） |
| 5 | u8 | unit（1deg、2mm、3body） |
| 6 | u8 | 总线1/2/3 |
| 7 | u8 | reserved=0 |
| 8 | u16 | 统一映射的最近设备结果/故障原因 |
| 10 | u16 | 原始DM motor_state；M2006/底盘FFFF表示无该单一状态 |
| 12 | u32 | DM/M2006合格反馈帧计数；底盘为0（不伪造聚合反馈计数） |

flags：bit0 connected，bit1 zeroed，bit2 limits_configured，bit3 enabled，bit4 enable_pending，bit5 busy，bit6 motion_complete，bit7 fault，bit8 host_link_timeout；其余0。M2006没有limits_configured位，底盘没有zeroed/busy/motion_complete位。底盘connected要求四轮均连接，enabled为软件运行请求，还要pending=false且轮状态全部Enabled才能认为就绪。

### DM，长度48

| P偏移 | 类型 | 字段 |
|---:|---|---|
| 16 | f32 | position，mm或deg |
| 20 | f32 | 实际速度，mm/s或deg/s（不是旧版rpm） |
| 24 | f32 | accepted_target，同位置单位 |
| 28 | f32 | 当前轨迹位置参考，同位置单位 |
| 32 | f32 | 当前轨迹速度参考，mm/s或deg/s |
| 36 | f32 | 反馈力矩Nm |
| 40 | f32 | MIT torque_ff Nm，不是总力矩 |
| 44 | u16 | 待处理设备命令：0无、1设零、2使能、3失能、4关节Move、5停止、6应用限值、7毫米Move |
| 46 | u16 | reserved=0 |

DM原始state：0失能、1使能、8过压、9欠压、10过流、11MOS过热、12线圈过热、13通讯丢失、14过载。原始电机内部故障不能靠串口ACK清除。

### M2006，长度32

P16 f32 position_deg，P20 f32实际deg/s，P24 f32 target_deg，P28 f32最近速度PID使用的总参考rpm（轨迹速度前馈+位置PD）。注意P28的rpm与P20的deg/s不同。库不公开位置参考/纯前馈或物理力矩，此帧不伪造那些字段。

### 底盘，长度44

P16/20/24三个f32=实际vx m/s、vy m/s、wz deg/s；P28/32/36三个f32=请求vx/vy/wz；P40 u8四轮connected位图，P41 u8四轮Enabled位图，P42 u8四轮故障状态位图（原始state>1），P43保留0。位0～3依次右前、左前、左后、右后。断联轮state可为旧值，因此状态位图必须结合connected位图使用。

### Wheels，四帧，每帧长度20

应答编号41/42/43/44与轮位对应；opcode/sequence回显请求。P0 result=0；P2 u16 flags（bit0 connected）；P4 u8实际CAN ID；P5 u8实际Master ID；P6 u16原始DM state；P8 f32输出轴角度deg；P12 f32 rpm；P16 f32 Nm。轮组角度不是整车位置，车体速度反解不能校正打滑。

## 8. Parameters

**DM，长度48**：P0 result；P2 u16 flags（bit0已校验配置，bit1有软件位置区间）；P4 kind；P5 unit；P6 bus=1；P7=0。P8起十个f32依次：

`min, max, max_velocity, max_acceleration, max_jerk, MIT_kp, MIT_kd, torque_ff_nm, finish_threshold, stationary_rpm`。

位置/动态/finish用本设备单位及相应时间单位；最后stationary始终rpm。伸缩min/max为IEEE NaN，表示“无软件行程限位”，不是无限协议编码能力；其他值仍须正常解析。返回已应用配置，不是尚未提交的编辑字段。默认伸缩速度/加速度/jerk约6.28333/12.56667/62.83333（mm时间单位），finish0.1885mm；关节10/20/100（deg时间单位），finish0.3deg。MIT kp5、kd0.5、ff0；stationary伸缩1.1rpm、关节0.1rpm。

**M2006，长度50**：P0 result，P2起连续十二个f32：

`velocity_Kp, velocity_Ki, velocity_Kd, velocity_output_max, position_Kp, position_Kd, position_output_max, profile_max_spd, profile_max_acc, profile_max_jerk, finish_deg, stationary_rpm`。

当前值9/.1/0/8000、5/4000/400、1800/1200/3000、.5/.1。离散PID/PD按已确认周期使用，不额外乘除dt。8000是C610协议指令约8A转矩电流，不是电源输入电流或安全保证。本版只查询，不在线修改这些构造参数。

**底盘，长度30**：P0 result，P2起七个f32：`max_vx=.1, max_vy=.1, max_wz=10, max_wheel_rpm=30, wheel_radius_mm=63.5, wheel_distance_x_mm=435, wheel_distance_y_mm=435`。

## 9. 统一result

协议服务映射不同设备的原始结果为以下统一值；设备类自身枚举不可直接照搬到上位机。

| 值 | 名称 | 说明 |
|---:|---|---|
| 0 | Ok | 已受理/查询成功，不等于到位 |
| 1 | Offline | 缺少有效电机连接/反馈 |
| 2 | NotZeroed | 尚未建立软件零点 |
| 3 | NotEnabled | 不满足运行授权 |
| 4 | Busy | 运动/命令忙、待使能确认或状态不允许 |
| 5 | InvalidParameters | 非有限数值或参数无效（DM非法位置也可返回6） |
| 6 | OutsideLimits | 工作范围/协议编码/速度边界不允许 |
| 7 | NotStationary | 未满足静止稳定要求 |
| 8 | MotorFault | 电机/软件故障条件 |
| 9 | PlanningFailed | 库轨迹求解失败 |
| 10 | InvalidCommand | 设备不支持该内部动作 |
| 11 | TransmitFailed | CAN提交失败 |
| 100 | BadVersion | 非v2 |
| 101 | BadType | 非请求方向 |
| 102 | BadLength | 超过50或与命令精确长度不同 |
| 103 | BadPadding | reserved/未用尾部非零 |
| 104 | UnknownOpcode | 未知命令 |
| 105 | UnknownMotor | 非法逻辑编号/命令不适用于该设备 |
| 106 | SequenceConflict | 最近相同sequence但整帧不同 |
| 107 | LinkFault | 串口超时锁存期间Enable/Move/BodyVelocity被禁止 |

## 10. 链路失联、恢复和客户端显示

首个语法有效请求建立会话；此前原DM调试入口仍可运行，本轮不为底盘/M2006添加Ozone命令。健康会话期间有效请求（包括设备拒绝的动作和最近缓存重发）刷新计时，坏CRC/坏版本/未知命令/SequenceConflict不刷新。

距最后有效请求>=500ms，TIM3检查路径对**全部六DM、两M2006和底盘**锁存停机请求；不依赖正在求解曲线的串口任务返回。DM取消曲线并走原发送时隙失能，底盘在500Hz路径四轮失能，M2006取消授权/零电流并在正常1kHz聚合时隙发送，不立即重试或突发发送。

超时锁存只能由**未命中最近请求缓存的新Heartbeat**解除；旧缓存Heartbeat重发只重放旧应答。缓存只有一项，不追溯更早历史，所以客户端恢复必须主动分配新sequence。解除链路门禁不清电机故障、不自动使能/续跑。M2006锁存故障后需失能稳定并显式SetZero清故障，再Enable；底盘在确认四轮连接/失能/静止后接受新的Enable；DM按设备条件明确Enable恢复。不得修改观察字段绕过条件。

显示建议：连上串口≠电机在线；从Info发现设备后，新State查询逐台显示connected/zeroed/enabled/pending/busy/complete/fault及result。connected=false时测量可能是旧缓存。Move完成须新反馈connected=true、无fault、busy=false、motion_complete=true；底盘是速度模式，没有“目标到位”。响应丢失、发送成功、CRC正确或Disable ACK都不能单独证明物理停机。

原DM保留Ozone邮箱；同台已有待处理命令时串口返回Busy，不覆盖。失联期间两入口的Enable/Move/MoveMm均禁止。UART软件环溢出/错误会丢弃不可信片段并重同步，发送错误确认中止后才可复用缓冲；本轮没有新增主动重连/重试/遥测机制。MCU Halt/断电或总线故障可能使失能帧无法发出，软件保护不是硬件急停。

## 11. Python字节参考（不打开串口）

```python
import binascii
import struct


def encode(seq, opcode, device, payload=b''):
    if not 0 <= seq <= 0xffff or len(payload) > 50:
        raise ValueError('sequence/payload')
    data = bytearray(64)
    struct.pack_into('<2sBBHBBHH', data, 0,
                     b'\xa5\x5a', 2, 0, seq, opcode, device, len(payload), 0)
    data[12:12 + len(payload)] = payload
    struct.pack_into('<H', data, 62, binascii.crc_hqx(data[:62], 0xffff))
    return bytes(data)


class ResponseParser:
    def __init__(self):
        self.pending = bytearray()

    def feed(self, incoming):
        self.pending.extend(incoming)
        replies = []
        while True:
            start = self.pending.find(b'\xa5\x5a')
            if start < 0:
                self.pending[:] = b'\xa5' if self.pending[-1:] == b'\xa5' else b''
                break
            del self.pending[:start]
            if len(self.pending) < 64:
                break
            raw = bytes(self.pending[:64])
            if struct.unpack_from('<H', raw, 62)[0] != binascii.crc_hqx(raw[:62], 0xffff):
                del self.pending[0]
                continue
            del self.pending[:64]
            _, ver, kind, seq, op, dev, size, reserved = struct.unpack_from('<2sBBHBBHH', raw)
            if ver != 2 or kind != 1 or not 2 <= size <= 50 or reserved or any(raw[12 + size:62]):
                continue
            replies.append((seq, op, dev, raw[12:12 + size]))
        return replies


heartbeat = encode(1, 0x01, 0xff)
all_states = encode(2, 0x10, 0xff)  # Expect nine replies, not six.
linear_target = encode(3, 0x24, 0x0a, struct.pack('<f', 2.0))  # Absolute2mm.
joint_target = encode(4, 0x24, 0x0d, struct.pack('<f', 30.0))
m2006_target = encode(5, 0x24, 0x21, struct.pack('<f', 90.0))
body_velocity = encode(6, 0x26, 0x40, struct.pack('<3f', 0.05, 0.0, 0.0))
wheel_states = encode(7, 0x12, 0x40)  # Expect41..44.
disable_all = encode(8, 0x21, 0xff)
```

示例只构造字节，不应在连接/重连时自动发送运动请求。事务层按本次设备kind/长度解码，保留底盘四轮状态分组，不把不同设备的payload当成同一结构。客户端先Heartbeat、Info、State(FF)、Parameters和Wheels确认当前条件，再由操作者明确发Enable/Move。

本轮验证证据见README对应章节；真实USB-UART、前端显示、电机端模式/编号、轮向及带载效果尚须后续实板联调，不由主机模拟或编译结果替代。

## 12. 独立CRC核对向量

以下对应第11节Python变量的完整64B请求，末尾两字节按在线顺序列出；已执行文档原样编码器/解析器，使用Python binascii.crc_hqx独立核对，包含逐字节噪声、坏CRC及连续应答解析。

| 变量 | CRC末尾字节(hex) |
|---|---|
| heartbeat | B7 D6 |
| all_states | 5F 62 |
| linear_target | 0B 78 |
| joint_target | 2A 3B |
| m2006_target | BC 15 |
| body_velocity | E7 41 |
| wheel_states | 5A F4 |
| disable_all | 3D 02 |

例如linear_target的Ok应答：version=2、type=1、sequence=3、opcode=24、device=0A、payload_length=2、result=0，保留/尾部全零，末尾CRC为D4 DA。参数查询中声明的伸缩min/max NaN需按“未配置软件行程区间”显示，不应使整个参数面板报浮点异常。

注意M2006在故障锁存期间disable()仍会释放控制并清零电流，但返回值可保留先前故障原因；故障不因发送Disable而被伪装成已清除。广播逐台结果与状态须分别判断，不能假设九帧全部result=0。

