# 操作、安全与验证边界

上位机逐项操作、通信通过判据和按反馈调参的步骤见[上位机电机通信检查与调参步骤](MOTOR_TEST_GUIDE.md)；设备配置与接口见[设备说明](DEVICES.md)。

## 历史交付背景

三总线串口 v2 功能与配套 `.ioc` 已通过云端 [PR #2](https://github.com/HITSZ-WTRobot/FDCAN_DM_TEST/pull/2) 合并。其后曾在从合并后 main 建立的 `fix/serial-session-timeout` 分支修复串口间歇超时和故障误标；当时用户反馈其他功能正常、控制参数尚未整定。保留 500ms 失联保护，不修改 PID、轨迹、外设参数，不自动恢复运动。下文各阶段验证记录须按其配置和时间边界理解，不能扩展为当前全部功能的实板证明。

## 串口上位机控制

完整交接文档：[SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md)，明确为**协议v2**，包括设备编号/单位、固定64B帧、CRC、查询布局、序号缓存、断线恢复、Python编解码和独立CRC向量。旧v1客户端必须更新，不保留把角度误当毫米的兼容分支。

USART1为PA9 TX/PA10 RX、115200/8N1，RX DMA1 Stream0 Circular、TX Stream1 Normal，IRQ均6/0。停等请求支持九个逻辑设备（六DM、双M2006、整个底盘），另有四轮状态查询。DM Move按设备使用mm/deg，M2006 MotorVelocity使用输出轴rpm（位置deg仅作诊断），底盘BodyVelocity用m/s和deg/s；可查询当前参数，DM可在失能时修改运动约束。广播State/Disable/Stop须收齐九帧，Wheels须收齐四帧。原DM邮箱保留但不会覆盖串口，底盘/M2006无Ozone运动入口。

首个有效请求建立会话，建议每100ms新心跳；>=500ms无有效请求取消全部六DM、双M2006和底盘的运行授权，并按各自周期请求失能/零电流，不立即重试。新Heartbeat只恢复链路门禁，不自动使能/恢复目标；M2006故障后须排除原因、保持失能并收到50个新的低速反馈，再显式Enable以0rpm授权，不支持SetZero。Stop不是失能，底盘Stop为零速度而非位置锁定。实际停机需新反馈确认，软件断线处理不是硬件急停；没有Flash参数保存。

HC-05类SPP模块可作为现有USART1的双向二进制透传链路，模块TX→PA10/RX、RX←PA9/TX并共地，通信模式需与115200/8N1一致；具体供电/逻辑电平按实际板卡确认，不将AT配置速率当作透传速率。Windows配对后上位机选择模块虚拟COM。手柄开关首次仅请求底盘使能，关闭对FF九设备请求失能，软件零速/失能不代表物理急停。蓝牙硬件/延迟未由工具实测，本轮不改.ioc或生成代码。


## Ozone 测试步骤

### 1. 运行与观察

用实际板卡的 J-Link/SWD 配置打开 `build/Debug/DMDebug.elf`，由操作者在动力安全隔离条件下下载并运行。之后保持主控运行，使用 Ozone 支持的运行态变量读写功能；不要在使能状态暂停/单步来改变量。

添加观察项：

```cpp
dm_startup
dm_rx_frames
dm_tx_submitted_frames
dm_tx_failures
dm_motors[0]
```

`dm_startup=Ready`（1）只证明初始化流程完成，不代表全部电机在线。按[设备配置表](DEVICES.md#电机配置)选择要测试的设备索引，确认指针非空再展开。以下以 `[0]` 为例：

```text
connected=true，feedback_count持续增加
zeroed=true，limits_configured=true
motor_state=0，enabled=false，fault=false
command=0，result=0
```

默认限位自动应用，不用首次再手填五个值。就绪依赖有效反馈，不用固定延时假定完成。

### 2. 单台使能与小目标

1. 以下角度示例只用于关节：机构支撑可靠、硬件急停可用后，在 Watch 中写 `dm_motors[3]->command=2`；伸缩使用[毫米软件接口](DEVICES.md#伸缩毫米位置与软件设零接口)，不用 target_deg/Move(4)。
2. 等命令自动回到 0，确认 `result=0`、`connected=true`、`enabled=true`、`enable_pending=false`、`fault=false`。使能前会预装当前位置保持目标。
3. 经确认动作空间允许后，先写 `target_deg=1.0`，再写 `command=4`，执行相对本次软件零点的 **+1° 绝对目标**。这只是测试示例，不代表该动作对所有机构都安全。
4. 观察 `accepted_target_deg`、`reference_deg`、`reference_velocity_deg_s`、`position_deg`、`velocity_rpm`、`torque_nm`。
5. 正常到位应有 `motion_complete=true`、`busy=false`，同时连接、使能和实际反馈正常。`busy=false` 也可能来自停止、失能或故障，不能单独作为到位证明。
6. 确认稳定后可先写 `target_deg=0.0`，再写 `command=4`，回到已有软件零点。

每次都先修改数据，最后写 command；等待 command 清零后才提交下一条命令。运动中再次 Move 返回 Busy，不覆盖原轨迹。`result=0` 表示命令接受，不等于已使能或已到位。

### 3. 停止、失能及重新设零

| command | 操作 | 注意 |
|---:|---|---|
| 0 | None | 无待处理命令 |
| 1 | SetZero | 仅失能、静止且有效反馈时接受；关节以新原点重建±180°，伸缩重定义毫米原点，不写电机 Flash |
| 2 | Enable | 人工请求使能，不自动执行旧目标 |
| 3 | Disable | 请求失能，在本电机下一发送时隙提交 |
| 4 | Move | 仅关节：规划并执行绝对 `target_deg`；伸缩拒绝此角度命令 |
| 5 | Stop | 取消轨迹并保持当前位置，仍可能有力矩；不是平滑减速或硬件急停 |
| 6 | ApplyLimits | 失能时提交动态约束；关节另检查±180°以内的工作区，伸缩不启用min/max行程检查 |
| 7 | MoveMm | 仅伸缩：绝对 `target_mm`，由 moveToMm() 复用该命令路径；关节拒绝 |

失能后必须确认新的 `connected=true`、`motor_state=0`、`enabled=false`，且反馈计数继续增加，不能将旧缓存或断线当作已经停机。重新设零后若要运动，等待静止反馈满足条件，再人工使能。复位会重新归零和应用默认值，不保存运行时覆盖参数。

首次测试逐台完成，随后才在同一总线上分别使能六台、提交经过确认安全的小目标。各台独立规划，不保证同时启动、同时到达；没有全体使能广播命令。

### 4. 覆盖限位与故障定位

失能、归零且 command=0 时，可以编辑 `max_velocity_deg_s`、`max_acceleration_deg_s2`、`max_jerk_deg_s3`，再提交 `command=6`；规划内核仍用角度单位，伸缩等效毫米值按[设备说明中的单位换算](DEVICES.md#伸缩毫米位置与软件设零接口)计算。关节的 min_deg/max_deg 必须在−180°～+180°内、min<max且整个区间可编码；伸缩 min/max 不构成软件行程边界。只改字段不会应用，非法动态参数不覆盖有效配置；limits_configured=true 不代表刚写的值被接受。下一次关节软件归零会重新建立完整±180°，不沿用此前缩小的区间。

| result | 含义 |
|---:|---|
| 0 | Ok |
| 1 | Offline |
| 2 | NotZeroed |
| 3 | NotEnabled |
| 4 | Busy |
| 5 | InvalidParameters |
| 6 | OutsideLimits |
| 7 | NotStationary |
| 8 | MotorFault（也可能是使能反馈超时） |
| 9 | PlanningFailed |
| 10 | InvalidCommand |
| 11 | TransmitFailed |
| 12 | LinkFault（串口超时，串口协议统一映射为107） |

先区分串口与CAN：DM的 `result=12/LinkFault` 表示串口500ms超时，真正CAN离线仍为 `result=1/Offline`。串口恢复后仍保留设备故障，不自动恢复运动；`host_link_timeout=false` 不能否定之前发生过超时。CAN故障再检查电机端 Master ID、编号、总线时序、接线、供电和终端。全局RX计数不能替代每台 `feedback_count`；`dm_tx_submitted_frames` 只表示硬件FIFO提交，不证明线上发送成功或动作完成。

软件看门狗由 TIM3 推进，名义失联窗口约 20 ms。故障会取消轨迹、锁存并请求失能，不自动重启，也不自动清除电机内部错误。排除原因、确认失能和静止后，再人工 Enable。不要改内部状态标志绕过检查。

## 安全与验证边界

- 参数图片的 CAN Timeout 为 0；主控暂停、断电或停止发包时，电机未必自动失能。
- 先支撑机构，再失能并核对反馈，按现场流程隔离动力后才下载、复位、暂停或断开调试器。Halt、断点和单步不是急停。
- `torque_ff_nm=0` 不代表总力矩为 0，Kp/Kd 仍会产生保持/阻尼力矩；前馈也不是总力矩上限。默认增益是调试初值，用户确认可运行不等于全部负载工况已完成整定。
- 本轮验证使用未修改的上游 DM 驱动、真实应用与 SCurve，CAN/HAL/RTOS 和机构使用主机模型：覆盖独立反馈 ID、交叉/异常帧拒收、旧 Master ID 拒绝连接、六台独立到位和故障处理。
- TIM6 2 kHz 的主机时序验证覆盖 32,820 个发送时隙：每次三台、两组交替；健康的一秒窗口每台提交 1,000 帧。不模拟真实位级仲裁、重发或电气故障，不能代替实板波形与负载检查。
- 实板结果：用户反馈“代码已经烧录并测试过可行了”；工具未直接操作目标，也未为重复确认该结果重跑硬件测试。当前 Debug 构建及主机烟测通过，烟测为本轮临时验证，不是仓库提供的独立测试命令。
- 旧串口v2位置模式烟测只证明其当时的软件路径；本次M2006速度协议使用新kind5/0x28/参数26B，须以本轮构建和新行为验证为准，不能复用旧30°/90°到位结论。广播失能仍逐台返回结果，清零电流时可能保留故障原因；ACK不证明物理停止。
- 此前串口超时分类烟测验证499/500ms边界、协议107与CAN离线1的区分；当前仍保留500ms门禁。M2006恢复已改为失能低速50新帧后显式Enable，不再设零，旧位置恢复步骤作废。未为本次切换重跑真实电机或通信故障场景。

