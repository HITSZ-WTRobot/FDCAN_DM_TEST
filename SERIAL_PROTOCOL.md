# 串口设备控制协议 v2

对应 `UserCode/communication/host_wire.*`、`host_control.*`、`host_uart.*`。本版接入六台DM位置、双M2006直接速度PID和四轮全向底盘。帧版本仍为2，M2006描述符改为kind=5、unit=4（rpm），Info清单必须精确匹配；旧kind=3的位置固件与新上位机互不兼容，禁止跳过握手校验。新增MotorVelocity=0x28，M2006旧Move/SetZero明确拒绝105，不把角度静默解释为转速。v1合法CRC请求仍返回BadVersion=100，串口失联原因107与真正CAN反馈离线1保持区分。

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
| 21 | M2006第一台速度控制 | 5 | 4=rpm | FDCAN3/C610 ID1，反馈201 |
| 22 | M2006第二台速度控制 | 5 | 4=rpm | FDCAN3/C610 ID2，反馈202 |
| 40 | 整个全向底盘 | 4 | 3=body | FDCAN2，四轮0B/0A/09/0C |

41～44只作为四轮状态应答的轮位标签，不接受直接单轮动作。轮位顺序固定为右前、左前、左后、右后，反馈Master ID为2B/2A/29/2C。Info返回的九设备顺序就是上表顺序。

位置语义：所有Move均为相对软件零点的绝对目标，不是增量。伸缩按226.2mm/输出轴圈换算；本版四台位置伸缩PMAX改为125rad，配合既有19.203位置口径可表达协议原点两侧约±234.3439mm，用于用户±200mm需求。电机端必须支持且同步125，否则比例错误10倍。无软件行程min/max，但仍检查绝对编码范围；若软件零点距协议原点超过约34.3439mm，不能保证两侧均有200mm。超出编码范围返回OutsideLimits，不截断。

DM关节每次软件零点建立后为−180～+180deg。M2006目标改为减速后输出轴有符号rpm，范围−300～+300；不设位置终点或0～360°限制，达到转速后持续运行。位置deg只作诊断，不用于速度授权或位置保持；内置36:1只由驱动处理。

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

- 保存最近一个**语法有效**请求的全部64B和全部应答。相同sequence且全帧相同：重放原应答，不再次执行（包括SetZero、Enable、Move、BodyVelocity、MotorVelocity）；相同sequence内容不同：106，不执行。
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
| 11 | Parameters | 单台 | 0 | DM48 / M2006 26 / 底盘30 |
| 12 | Wheels | 40 | 0 | 四帧，每帧20 |
| 20 | Enable | 单台 | 0 | 2 |
| 21 | Disable | 单台或FF | 0 | 2；FF九帧 |
| 22 | Stop | 单台或FF | 0 | 2；FF九帧 |
| 23 | SetZero | 仅DM单台 | 0 | 2 |
| 24 | Move | 仅DM单台 | 4：P0 float绝对位置，伸缩mm、关节deg | 2 |
| 25 | SetLimits | DM单台 | 伸缩12：v/a/jerk（mm/s、mm/s²、mm/s³）；关节20：min/max/v/a/jerk（deg及相应时间单位） | 2 |
| 26 | BodyVelocity | 40 | 12：P0 vx m/s、P4 vy m/s、P8 wz deg/s | 2 |
| 28 | MotorVelocity | 仅21或22 | 4：P0 float输出轴rpm，−300～+300 | 2 |

不支持广播Enable/MotorVelocity、底盘或M2006的SetZero/Move、M2006/底盘SetLimits、直接单轮控制、世界系/整车位置导航、参数Flash保存、运行中自动恢复或串口速度PID在线改写。0x27仍未实现，不用其承载Ozone调参。

### 动作规则

- **Enable**：先确认连接、参数及设备条件。DM保留零点/静止门禁；底盘请求四台使能并等待四个新状态确认。M2006不需软件零点，但须失能且收到50个新的低速样本（|rpm|≤stationary_rpm）；显式Enable可在原因排除后清除锁存故障，重置PID与旧目标并以0rpm授权，绝不恢复旧转速。M2006 enabled仅是本地控制器授权，C610没有物理使能应答。上电不自动使能。
- **Disable**：DM下一各自TIM6时隙发送失能，底盘下一TIM3时隙四台失能，M2006清电流缓存并在下一1kHz聚合发送零。ACK不是电气/机械停机确认。
- **Stop**：DM取消曲线并保持当前位置，仍可产生保持力矩；M2006直接把速度参考设为0rpm且保持软件使能，不锁位置、不规划减速；底盘把车体系速度设零。速度突变可能产生制动电流，不等于失能或硬件急停。未使能设备可返回NotEnabled，FF Stop可出现部分成功。
- **SetZero**：仅六台DM在连接、失能、静止稳定时接受，保留首次自动归零。关节重新建立新原点两侧各180°，伸缩只改变毫米原点；不写电机零点寄存器或Flash、不自动使能。新关节完整范围无法编码则拒绝并保留旧配置。M2006不支持设零。
- **Move**：有限且满足工作/协议范围，已使能、连接有效、无fault。运动忙返回Busy，保留旧曲线/目标；规划失败也不替换旧轨迹。MM转换和轨迹执行使用已有设备API，不在通信层另建曲线或PID。
- **SetLimits**：失能状态提交；非法值保留已应用配置。伸缩仅调整轨迹动态参数，没有min/max行程字段；关节范围必须在−180～+180°以内且完整可编码。重设关节零点后重新采用完整±180°。仅RAM生效。
- **BodyVelocity**：x向前、y向左、wz逆时针为正。合成平移hypot(vx,vy)≤1m/s、|wz|≤10deg/s；vx/vy各字段上限1，单轮速度包络160rpm。非法、非有限或合成超限拒绝且保留旧目标，不缩放，1e-6仅为float32量化容差。Parameters布局不变，返回max_vx=1/max_vy=1/max_wz=10/max_wheel_rpm=160；旧上位机须读取新上限。轮向和提速必须先安全验证。
- **MotorVelocity**：仅M2006。已使能、连接有效、无故障/主机门禁时，有限且绝对值不超过查询max_rpm的目标直接交给速度PID，可在旋转中更新或换向，不因持续运行返回位置Busy；非法值拒绝且保留旧目标。没有位置终点、加减速规划或motion_complete判据。Stop为0rpm，Disable为取消授权/零电流；两者语义不同。

## 6. Heartbeat / Info

Heartbeat成功：P0 uint16 result，P2 uint16 link_flags，P4 uint32 uptime_ms（HAL时间，uint32回绕）。link_flags bit0=已见有效串口请求，bit1=串口超时锁存。

Info前8B同Heartbeat，其余：

| P偏移 | 类型 | 字段 |
|---:|---|---|
| 8 | u16 | 失联超时ms=500 |
| 10 | u8 | 逻辑设备数=9 |
| 11 | u8 | 协议版本=2 |
| 12 | u8[9] | 编号：0A 0B 0C 0D 0E 0F 21 22 40 |
| 21 | u8[9] | kind：1 1 1 2 1 2 5 5 4；旧3位置模式不兼容 |
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
| 4 | u8 | kind（1伸缩、2关节、4底盘、5 M2006速度） |
| 5 | u8 | unit（1deg、2mm、3body、4rpm） |
| 6 | u8 | 总线1/2/3 |
| 7 | u8 | reserved=0 |
| 8 | u16 | 统一映射的最近设备结果/故障原因 |
| 10 | u16 | 原始DM motor_state；M2006/底盘FFFF表示无该单一状态 |
| 12 | u32 | DM/M2006合格反馈帧计数；底盘为0（不伪造聚合反馈计数） |

flags：bit0 connected，bit1 zeroed，bit2 limits_configured，bit3 enabled，bit4 enable_pending，bit5 busy，bit6 motion_complete，bit7 fault，bit8 host_link_timeout；其余0。M2006速度模式不设zeroed/limits_configured/enable_pending/busy/motion_complete位，不用到位表示达速；底盘同样没有位置到位。底盘connected要求四轮均连接，enabled还须pending=false且轮状态全部Enabled才就绪。

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

P16 f32 position_deg（只诊断），P20 f32实际velocity_rpm，P24 f32已受理target_rpm，P28 f32最近速度PID使用的velocity_reference_rpm。后三项统一输出轴rpm，不乘除6；无位置PD或轨迹前馈。未使能、故障或停止时旧目标不会续跑；位置跨过360°不会触发位置越界。不公开电流/力矩字段，不伪造零测量。

### 底盘，长度44

P16/20/24三个f32=实际vx m/s、vy m/s、wz deg/s；P28/32/36三个f32=请求vx/vy/wz；P40 u8四轮connected位图，P41 u8四轮Enabled位图，P42 u8四轮故障状态位图（原始state>1），P43保留0。位0～3依次右前、左前、左后、右后。断联轮state可为旧值，因此状态位图必须结合connected位图使用。

### Wheels，四帧，每帧长度20

应答编号41/42/43/44与轮位对应；opcode/sequence回显请求。P0 result=0；P2 u16 flags（bit0 connected）；P4 u8实际CAN ID；P5 u8实际Master ID；P6 u16原始DM state；P8 f32输出轴角度deg；P12 f32 rpm；P16 f32 Nm。轮组角度不是整车位置，车体速度反解不能校正打滑。

## 8. Parameters

**DM，长度48**：P0 result；P2 u16 flags（bit0已校验配置，bit1有软件位置区间）；P4 kind；P5 unit；P6 bus=1；P7=0。P8起十个f32依次：

`min, max, max_velocity, max_acceleration, max_jerk, MIT_kp, MIT_kd, torque_ff_nm, finish_threshold, stationary_rpm`。

位置/动态/finish用本设备单位及时间单位；stationary始终rpm。伸缩min/max为NaN表示无软件行程区间，不代表无限编码能力。返回已应用配置：伸缩轨迹6.28333/12.56667/62.83333（mm时间单位），finish0.1885mm，MIT20/0.5/0；关节轨迹570/1080/3500（deg时间单位），finish0.3deg，MIT保留已有用户增益；stationary伸缩1.1rpm、关节0.1rpm。启动值以config.hpp为准，运行态以实际回读为准。

**M2006，长度26**：P0 result；P2起连续六个f32：

`velocity_Kp, velocity_Ki, velocity_Kd, velocity_output_max, max_rpm, stationary_rpm`。

默认18/.1/0/8000、300/.1。PID离散增益不额外乘除dt，输出上限8000为约8A转矩电流指令，不是电源输入电流或机械安全保证。位置PD、轨迹和finish字段已移除；旧50B参数包不能按新布局解码。串口只查询配置，不在线改PID。

**底盘，长度30**：P0 result，P2起七个f32：`max_vx=1, max_vy=1, max_wz=10, max_wheel_rpm=160, wheel_radius_mm=63.5, wheel_distance_x_mm=435, wheel_distance_y_mm=435`。vx/vy还共同受hypot≤1约束，并非两个分量同时可取1。

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
| 107 | LinkFault | 串口超时门禁，或设备保留的串口失联故障原因；新心跳后仍可作为设备最近结果返回 |

## 10. 链路失联、恢复和客户端显示

首个语法有效请求建立会话；此前原DM调试入口仍可运行，本轮不为底盘/M2006添加Ozone命令。健康会话期间有效请求（包括设备拒绝的动作和最近缓存重发）刷新计时，坏CRC/坏版本/未知命令/SequenceConflict不刷新。

距最后有效请求>=500ms，TIM3检查路径对**全部六DM、两M2006和底盘**锁存停机请求；不依赖正在求解曲线的串口任务返回。DM取消曲线并走原发送时隙失能，底盘在500Hz路径四轮失能，M2006取消授权/零电流并在正常1kHz聚合时隙发送，不立即重试或突发发送。

超时锁存只能由**未命中最近请求缓存的新Heartbeat**解除；旧缓存Heartbeat重发只重放旧应答。解除链路门禁不清设备故障、不自动使能或续跑。M2006须失能并累积50个新的低速反馈，排除原因后显式Enable清故障且以0rpm授权；底盘及DM按各自条件明确Enable恢复，不修改观察字段绕过检查。

State的P8是统一协议结果：设备保留串口故障时返回107，真正CAN反馈离线返回1；它与P0查询是否成功、当前host_link_timeout标志独立。M2006内部枚举与协议结果不可直接混用。

显示建议：连上串口≠电机在线；Info须核对全部编号及kind，再读新State。connected=false时测量可为旧缓存。仅DM Move使用motion_complete判定到位；M2006与底盘是持续速度模式，没有“目标到位”。速度目标ACK不证明已达到目标转速，Disable ACK不证明物理停机。

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
m2006_target = encode(5, 0x28, 0x21, struct.pack('<f', 90.0))  # Output-shaft90rpm, not90deg.
body_velocity = encode(6, 0x26, 0x40, struct.pack('<3f', 0.05, 0.0, 0.0))
wheel_states = encode(7, 0x12, 0x40)  # Expect41..44.
disable_all = encode(8, 0x21, 0xff)
```

示例只构造字节，不应在连接/重连时自动发送运动请求。客户端先Heartbeat、Info（含新kind5）、State(FF)、Parameters和Wheels确认条件，再由操作者明确Enable并发Move或MotorVelocity。不同kind按自己的单位和长度解析；不能向M2006发旧角度目标。

本轮验证证据见README对应章节；真实USB-UART、前端显示、电机端模式/编号、轮向及带载效果尚须后续实板联调，不由主机模拟或编译结果替代。

## 12. 独立CRC核对向量

以下对应第11节Python变量的完整64B请求，末尾两字节按在线顺序列出；已执行文档原样编码器/解析器，使用Python binascii.crc_hqx独立核对，包含逐字节噪声、坏CRC及连续应答解析。

| 变量 | CRC末尾字节(hex) |
|---|---|
| heartbeat | B7 D6 |
| all_states | 5F 62 |
| linear_target | 0B 78 |
| joint_target | 2A 3B |
| m2006_target（0x28，90rpm） | FB 2C |
| body_velocity | E7 41 |
| wheel_states | 5A F4 |
| disable_all | 3D 02 |

例如linear_target的Ok应答：version=2、type=1、sequence=3、opcode=24、device=0A、payload_length=2、result=0，保留/尾部全零，末尾CRC为D4 DA。参数查询中声明的伸缩min/max NaN需按“未配置软件行程区间”显示，不应使整个参数面板报浮点异常。

注意M2006在故障锁存期间disable()仍会释放控制并清零电流，但返回值可保留先前故障原因；故障不因发送Disable而被伪装成已清除。广播逐台结果与状态须分别判断，不能假设九帧全部result=0。

