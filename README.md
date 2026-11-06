# FDCAN_DM_TEST

STM32H723VET6 三 FDCAN 串口控制工程：FDCAN1为六台DM位置控制，FDCAN2为四轮全向底盘速度控制，FDCAN3为两台独立M2006直接速度PID（输出轴±300rpm）。设备层负责控制，communication负责串口分发，app只组装与调度；不新增Ozone控制邮箱。

> 三总线串口v2功能与配套 `.ioc` 已通过云端 [PR #2](https://github.com/HITSZ-WTRobot/FDCAN_DM_TEST/pull/2) 合并。本轮在从合并后main建立的 `fix/serial-session-timeout` 分支修复串口间歇超时和故障误标；用户反馈其他功能正常、控制参数尚未整定。保留500ms失联保护，不修改PID、轨迹、外设参数，不自动恢复运动。

上位机逐项操作、通信通过判据和按反馈调参的步骤见 [上位机电机通信检查与调参步骤](MOTOR_TEST_GUIDE.md)。

## 电机配置

电机端必须预先设置为 **MIT 模式**，Master ID 按上游驱动要求为 **`0x20 + CAN ID`**。固件不会自动写入模式、编号、波特率或电机 Flash。

| Ozone 索引 | 部位 | 型号 | CAN ID | Master ID | PMAX（rad） | VMAX（rad/s） | TMAX（Nm） |
|---|---|---|---|---|---:|---:|---:|
| `dm_motors[0]` | 右前伸缩 | DM-S3519 | `0x0A` | `0x2A` | 125 | 200 | 10 |
| `dm_motors[1]` | 中伸缩 | DM-S3519 | `0x0B` | `0x2B` | 125 | 200 | 10 |
| `dm_motors[2]` | 后伸缩 | DM-S3519 | `0x0C` | `0x2C` | 125 | 200 | 10 |
| `dm_motors[3]` | 左前旋转 | DM-J4340P | `0x0D` | `0x2D` | 12.5 | 10 | 28 |
| `dm_motors[4]` | 左前伸缩 | DM-S3519 | `0x0E` | `0x2E` | 125 | 200 | 10 |
| `dm_motors[5]` | 右前旋转 | DM-J4340P | `0x0F` | `0x2F` | 12.5 | 10 | 28 |

PMAX/VMAX/TMAX是协议映射量程，不是机械运动限制，两端必须一致。本版将四台位置伸缩的PMAX由12.5改为125rad；必须先确认电机端支持并逐台设为125，再运行本固件。电机仍为12.5时禁止按此配置运动，比例会错10倍；未由工具改写或验证电机寄存器。两关节仍12.5/10/28，FDCAN2四轮不随此修改。Master ID仍为0x20+CAN ID。

S3519 位置口径保留用户此前实测及上游驱动的处理：反馈位置为减速前角度，驱动除以 `19.203` 换算输出轴角度；应用发送 MIT 位置时反向转换。规划器内部仍使用输出轴度数，四个伸缩对外提供毫米接口，按用户给定的标称 **226.2 mm/输出轴圈**换算；操作者不额外乘除内置减速比。S3519 速度沿用驱动的输出轴口径，毫米速度反馈从 rpm 独立换算。

## 硬件和定时分工

- STM32H723VET6；FDCAN1 的 RX 为 PB8，TX 为 PB9；通过板载收发器接 CAN_H/CAN_L。
- FDCAN 内核时钟 100 MHz；前两路仲裁段 1 Mbps、数据段 5 Mbps、FD+BRS；第三路 Classic CAN 1 Mbps、不启用 BRS。电机端需匹配实际时序，不仅是界面中的速率标签。
- 上游 `DMMotor::CAN_FilterInit(can, index)` 使用标准 Mask 过滤器：`id1=0x028`、`id2=0x7F8`，覆盖合法反馈 ID。每实例及应用再次同时检查反馈 ID 与载荷电机 ID。
- 三条总线各由一个 Can 对象管理；全部十二台电机完成构造、接收回调注册后才启动总线，各设备组不共享或重复管理同一 HAL 句柄。

| 执行位置 | 频率 | 用途 |
|---|---:|---|
| 命令任务 | 约 1 ms 服务 | 串口/原DM命令、底盘/M2006状态服务；不自动使能 |
| TIM7 | 100 Hz | 仅六台DM轨迹采样；M2006不再运行位置规划 |
| TIM3 | 500 Hz | DM反馈/故障、底盘速度或失能输出、M2006低速样本/故障观察、500ms串口门禁；唯一全局Watchdog推进者 |
| TIM6 | 2 kHz | 原 DM 两组三台交替发送；每第二次回调更新两台独立 M2006 的速度PID，再统一聚合发送，即1 kHz |
| TIM23 | HAL 系统时基 | 保留生成配置，不用于电机控制 |

发送组 A 为 `[0,1,2]`（`0x0A/0x0B/0x0C`），组 B 为 `[3,4,5]`（`0x0D/0x0E/0x0F`）。组间软件提交间隔 0.5 ms，同一台周期 1 ms。MIT、使能、失能均走该唯一发送入口；发送失败不在当前时隙立即重试。CAN 仲裁、排队和错误重发仍可能改变实际线上时刻；分组不等于降低平均负载。

基础外设配置由操作者使用 CubeMX 管理。程序只注册回调、配置运行期过滤/通知及发送延迟补偿、启动已初始化外设，不重新配置时钟或位时序。

## 获取依赖和构建

需要 STM32CubeMX（对应 `.ioc` 的固件包）、Arm GNU Toolchain、CMake ≥ 3.22、Ninja、cpkg。固件使用 C++20。本轮构建使用 Arm GNU 14.3 Rel1。

```sh
git clone --recurse-submodules https://github.com/HITSZ-WTRobot/FDCAN_DM_TEST.git
cd FDCAN_DM_TEST
```

`.gitmodules` 的依赖 URL 使用 GitHub HTTPS。也可先普通克隆，再通过 cpkg 同步：

```sh
git clone https://github.com/HITSZ-WTRobot/FDCAN_DM_TEST.git
cd FDCAN_DM_TEST
cpkg sync --submodule-protocol https
```

依赖由 [wtrproject.toml](wtrproject.toml) 声明，`Trajectory::MotorTrajectory` 传递提供当前使用的 `VelocityProfile::SCurve`；`Chassis::Omni4` 提供四全向轮运动学并传递引入轮速控制器；`MotorDrivers::DJI` 提供新增 M2006/C610 的反馈解码和电流聚合发送。不手工编辑模块源码或生成的包 CMake 文件。同步会拉取配置的依赖分支，请留意子模块版本变更；已提交的子模块引用记录交付时的版本。

本仓库按现有 `.gitignore` 不提交 `Core/`、`Drivers/`、`Middlewares/`、启动汇编、CubeMX 生成的 CMake 子目录以及构建产物，**新克隆不能跳过代码生成**：

1. 使用 CubeMX 打开 [DMDebug.ioc](DMDebug.ioc)，安装/选择它要求的 STM32H7 固件包，保持目标为 CMake 并生成代码。
2. 保留根 CMake 的 C++20、UserCode 和 cpkg 接入；不要用其他工程的生成文件替代。
3. 执行 cpkg 同步，生成被忽略的 `cmake/wtr_modules.cmake` 和包构建文件。
4. 在工具链、CMake、Ninja 可用的终端执行：

   ```sh
   cpkg sync
   cmake --preset Debug
   cmake --build --preset Debug
   ```

产物：`build/Debug/DMDebug.elf`。下载前先隔离动力并支撑机构；本工程的构建命令不会自动烧录。

## 默认运动参数

| 电机类别 | 相对软件零点的位置范围 | 速度 | 加速度 | jerk |
|---|---|---:|---:|---:|
| 四个伸缩 | 不设软件行程区间；保留绝对协议编码边界 | 6.2833 mm/s | 12.5667 mm/s² | 62.8333 mm/s³ |
| 两个旋转 | −180°～+180° | 570°/s | 1080°/s² | 3500°/s³ |

当前MIT启动值：四伸缩Kp=20、Kd=0.5；两关节保留用户已有增益，力矩前馈均为0。到位阈值0.3°（伸缩0.1885mm），静止阈值伸缩1.1rpm、关节0.1rpm。关节原图1440°/s超过VMAX10rad/s，用户确认改用570°/s，其余按图。伸缩无软件行程区间，选择PMAX125以覆盖协议原点两侧±200mm需求；不是新增物理限位或实测可用行程。

上电不会自动使能。六台DM收到50次新的失能、静止且位置稳定反馈后尝试建立软件零点。两个关节初次及后续软件设零都重新建立−180°～+180°；若完整范围不能编码则拒绝并保留旧状态。四伸缩不设软件行程区间，但零点和轨迹仍须可编码。M2006无需零点，改为失能后50个新的低速反馈授权条件；不会自动归零或恢复转速。

设备参数统一入口为 [UserCode/config.hpp](UserCode/config.hpp)：`config::dm_motor_parameters` 为六台 DM 的编号、型号、量程及方向，`config::dm_control` 为 MIT 参数，`config::joint_motion_limits` / `config::linear_motion_dynamics` 为轨迹约束；底盘方向为 `config::chassis.reverse`，M2006 编号/方向和控制参数分别为 `config::m2006_motor_parameters` / `config::m2006_control`。参数在初始化时被复制，初始化后只改配置变量不会在线生效；修改代码、构建并安全重启，或在初始化前设置。关节限位可在 ±180° 内缩小，动态约束支持失能时显式提交；伸缩 min_deg/max_deg 不作为行程限位。

### 伸缩毫米位置与软件设零接口

四个伸缩为 `dm_motors[0]`（右前）、`[1]`（中）、`[2]`（后）、`[4]`（左前）；关节为 `[3]`、`[5]`。构造时显式指定PositionKind避免单位猜测。以下软件接口已被串口v2消费，不会在固件启动时自动执行：

```cpp
// 在既有命令任务中调用；先按既有流程确认已归零、已使能、静止且不busy。
device::Result accepted = dm_motors[0]->moveToMm(10.0f);
// 目标是“当前软件零点外10mm”，不是从当前点再增加10mm。
auto feedback_mm = dm_motors[0]->linearFeedback();
```

重新设零是另一个独立操作：先失能并确认稳定新反馈，再调用；使能状态下会返回 Busy，不会边运动边改变坐标。

```cpp
device::Result zero_result = dm_motors[0]->setSoftwareZero();
```

`moveToMm()` 返回本次受理结果，不是到位结果；Ok 后仍需 connected=true、fault=false、motion_complete=true 且 busy=false，并核对新反馈。运动中/已有待处理命令时返回 Busy，不覆盖已接受轨迹；NaN/Inf 或无法编码的目标返回 OutsideLimits。关节调用毫米接口返回 InvalidCommand，原始 Move(4)/target_deg 对伸缩也返回 InvalidCommand，避免把毫米误当度。接口共用既有命令仲裁和 S 曲线求解，只能由同一个命令任务调用，不可在 CAN/UART ISR 或其他任务并发调用；所有发送仍走原 TIM6 分组时隙。

`linearFeedback()` 对关节返回 nullopt，对伸缩返回无堆分配的快照，包含 position_mm、velocity_mm_s、accepted_target_mm、reference_mm、reference_velocity_mm_s。其定义分别为角位置/目标/参考乘 `226.2/360`，实际反馈速度 rpm 乘 `226.2/60`，参考速度 deg/s 乘 `226.2/360`；不要混用两种速度单位。快照一致性由短临界区保证，但 connected=false 时仍可能包含旧值；运动状态/连接/故障仍查看原设备字段。

“无伸缩行程限位”保留协议编码检查，不增加机械行程区间。现按用户±200mm目标将四台位置伸缩PMAX设为125rad，既有19.203位置比例下对应协议原点附近±234.3439mm；理论最低PMAX约106.68082rad。电机端须支持且同步为125；软件零点若偏离协议原点超过约34.3439mm，至少一侧不足200mm，不能凭重新设零保证任意位置两侧各200mm。保持226.2mm/圈换算和原坐标口径，不伪造无限编码范围或删除越界拒绝。

此前12.5rad量程阶段的主机验证覆盖毫米轨迹、单位/零点/故障及约±23.434mm编码边界，不可当作本次125rad配置的实板证据。本轮验证以当前配置与未修改的真实驱动/SCurve运行结果为准；尚未烧录或验证电机端PMAX支持、实际接线、机械行程与带载安全。

此前位置模式验证记录不用于证明当前M2006速度模式。六DM仍使用既有SCurve，四伸缩和双关节的毫米/角度接口与限位逻辑不随本次改变；当前速度模式的实际验证见后文，模型不替代实板或机械安全。


## 四轮全向底盘结构（FDCAN2）

底盘现在提供设备层 enable/disable/stop/setVelocity 与反馈接口，复用既有 Master、Omni4 和 JustEncoder。只开放车体系速度，不开放世界系或整车位姿目标。编码器后端用于库的真实反馈契约，不把积分里程宣称为绝对定位；不新增Ozone控制。原六DM和双M2006保持各自设备接口。

### 几何与电机参数

用户确认：四轮安装角 45°，轮径 127 mm（库传入半径 **63.5 mm**）；四轮中心为正方形，边长 **435 mm**，所以库的完整前后轮距和左右轮距均为 435 mm，不传半轮距。电机输出轴到车轮直连，`reduction_rate=1`；不重复计入 DM-S3519 内置减速。所有 `reverse` 初值为 false，是用户指定的待实板确认方向，不代表已校验装配。

| 库轮位顺序 | 部位 | CAN ID | 要求的 Master ID | 速度模式指令 ID |
|---:|---|---|---|---|
| 0 | 右前 | 0x0B | 0x2B | 0x20B |
| 1 | 左前 | 0x0A | 0x2A | 0x20A |
| 2 | 左后 | 0x09 | 0x29 | 0x209 |
| 3 | 右后 | 0x0C | 0x2C | 0x20C |

四台协议映射量程均为 PMAX=12.5 rad、VMAX=200 rad/s、TMAX=10 Nm。参数图片中前轮是 MIT、后轮是速度模式，Master ID 均为旧 0x20；本轮用户已选择**统一内部速度模式**，须由操作者用电机工具完成模式=3 和上表 Master ID 配置，固件不会修改电机寄存器或 Flash，尚未取得全部四台已调整完成的确认。电机内部速度 PI 暂保留图片 KP=0.539579、KI=0.002，不等同带载整定；应用 InternalVel 路径不使用外部速度 PID。

### 外设与执行边界

- FDCAN2 RX=PB5、TX=PB6，使用第二路独立隔离收发器及 FDCAN2H/FDCAN2L/FDCAN2_GND；不与第一路物理总线合并。两路可以使用重复 CAN ID，因为总线和回调对象独立。
- 公共内核时钟 100 MHz；Normal、FD+BRS、自动重传 Enable。Nominal Prescaler/Seg1/Seg2/SJW=5/15/4/4（1 Mbps）；Data=1/15/4/4（5 Mbps）。FDCAN 回调注册保持启用，FDCAN2 IT0/IT1 IRQ 均启用且为 5/0，由生成的 IRQ 调用 HAL_FDCAN_IRQHandler(&hfdcan2)。
- FDCAN2 MessageRAMOffset=512（32 位字）；StdFiltersNbr=1、ExtFiltersNbr=0，RX FIFO0=16×64 B，TX FIFO=16×8 B、FIFO 模式，其他 RX FIFO/Buffer、TX Buffer/Event 数量为 0。当前 FDCAN1 占字区间 [0,353)，FDCAN2 占 [512,865)，不得重叠。应用使用上游标准过滤器及实测环回延迟补偿，offset 由实际数据段采样点算得 16 mtq。
- 以上配置由用户在 CubeMX 完成，本轮已核对实际生成结果，未私改生成层；用户 `.ioc` 改动不随应用提交擅自暂存。其他工作区/新克隆必须先在 CubeMX 配齐 FDCAN2 并生成，不能只取应用提交就跳过此步骤。
- 四个 DMMotor、四个内部速度控制器、Omni4、JustEncoder、Master按依赖顺序构造，永久存活。构造/start不使能；启动返回滤波、延迟补偿和CAN启动结果。
- TIM3的update()每2ms执行一次：健康运行时调用Master.controllerUpdate()，由真实Omni4分解并每轮发送一个速度参考；未运行/故障时每轮发送一个失能包。不再同时调用motion.update()造成重复发送。失败不立即重试。
- enable()是任务接口，现有库会在此发送四台使能请求；随后等待新反馈确认四台都Enabled，才接受非零速度；没有自动使能。disable及链路故障在下一TIM3时隙发失能。上游mutex接口不从ISR调用；停机IRQ只取消运行授权，周期路径释放控制并发送失能。
- 命令上限为合成平移 `hypot(vx,vy)≤1m/s`、`|wz|≤360deg/s`、保守单轮包络≤360rpm。按实际几何检查 `hypot(vx,vy)+hypot(0.435,0.435)/2×|wz|×π/180≤360×2π×0.0635/60`；这是方向无关上界，可能拒绝实际某个方向仍可行的组合。平移与旋转不分别截断，超限直接拒绝并保留旧参考，不默默缩放；1e-6m/s仅为float32量化容差。实际轮速仍由原Omni4库分解。
- 纯1m/s平移、纯±360deg/s旋转都可用；手柄0.4m/s与±360deg/s可同时使用，最坏约350.79rpm；手动1m/s与360deg/s组合被拒绝。360rpm为用户确认的软件上限，不是电机带载能力或整车安全速度保证；需更新固件和上位机，未自动烧录。

### 代码与反馈

`UserCode/device/dm_omni_chassis.*` 封装总线、对象、车体速度命令和500Hz更新；app只调用。setVelocity缓存真实库车体系速度目标，stop发送零速度参考而非位置锁定，disable请求停止输出。linkLost锁存故障和禁止使能，linkRestored只解除链路门禁，不自动使能。新enable要求四轮连接、失能且静止，随后清除软件故障并等待反馈确认；不改电机端模式或Flash。

可观察 `chassis_startup`、`chassis_poll_ok` 和 `chassis_feedback`，它们不是控制命令：

- `chassis_startup`：NotStarted=0、Ready=1、FilterFailed=2、DelayCompensationFailed=3、CanStartFailed=4。
- `chassis_poll_ok` 仅表示最近一轮四帧全部提交硬件成功，不证明物理线上发送、收到应答或已经失能。
- `chassis_feedback.wheels[0..3]` 按上表轮位顺序记录connected/state/输出轴deg/rpm/Nm；完整快照另外包含target、enabled、enable_pending、fault、result。connected=false时其他测量可能是旧缓存，不能作新鲜反馈；enabled只是软件授权，实际使能需轮状态确认。
- `velocity_in_body` 实际调用 Omni4::forwardGetVelocity()，单位为 vx/vy=m/s、wz=deg/s；x 向前、y 向左、逆时针为正。只有四轮均有合格新反馈且轮向正确时才有物理意义；它不是整车定位，也不校正打滑。
- `config::chassis.reverse`（`UserCode/config.hpp`）在构造时复制；运行后仅改该数组不会在线改变轮向。改代码或在初始化前设置，安全重启后生效；当前不提供在线调参/保存接口。电机端实际模式、反馈编号和轮向仍需在架空、动力安全的条件下确认。

### 本轮验证

本轮360deg/s、360rpm配置的Debug构建通过；临时ASan/UBSan烟测运行实际DMOmniChassis、Master、Omni4、JustEncoder、DM驱动及轮速控制器，仅CAN/HAL/CMSIS使用边界模型。725个合法目标通过，含360个平移方向各叠加±360deg/s；9个超限或非有限目标均拒绝且保留旧目标；纯360deg/s打包轮速290.638rpm，包络边界附近峰值359.985rpm，零速停止及失能帧也已观察。

此前30rpm及1m/s/160rpm阶段的UART、广播失能和500ms门禁结果属于旧阶段证据，不代替本轮提速实板验证。本轮未烧录或操作机构；主机模型不证明实物轮向、无线延迟、波形或带载安全。


## 双 M2006/C610（FDCAN3）

### 直接速度闭环

两台分别使用未修改的 `DJIMotor` 和 `MotorVelController(ExternalPID)`，设备为 `UserCode/device/m2006_velocity.*` 的 `M2006Velocity`，实例仍为 `m2006_motors[0]` / `[1]`。目标为减速后输出轴有符号rpm，默认−300～+300，内置36:1只由库处理。两台目标和PID独立；没有位置终点、0～360°限制、位置PD、位置保持或S形起停。速度阶跃和直接换向可能产生电流冲击，必须按机械条件选择安全目标。

| 参数 | 当前值 | 含义 |
|---|---:|---|
| velocity_pid.Kp / Ki / Kd | 18 / 0.1 / 0 | 既有离散速度PID，1kHz，不额外乘除dt |
| velocity_pid.abs_output_max | 8000 | C610电流协议单位，约8A转矩电流指令，不是电源电流测量或安全保证 |
| max_rpm | 300 | 输出轴绝对速度目标上限，超限拒绝、不截断 |
| stationary_rpm | 0.1 | 启用/故障恢复的低速门限，须50个新的合格反馈 |

参数在构造时从 `config::m2006_control` 复制；运行后仅改启动数组不会在线生效。上位机Ozone页只保留实际速度PID、max_rpm和stationary_rpm，删除位置PD/轨迹/到位字段；写RAM仍需匹配ELF，不保存Flash。max_rpm是命令和周期执行的约束，不是电机硬件保护。

### 生命周期与软件接口

- 上电零电流、未使能，不自动设零。位置deg仅作反馈诊断，越过360°不因位置越界停机。
- `enable()`：连接有效、参数合法、主机门禁已解除，失能并有50个新的低速样本后才允许；显式请求可清除已经排除原因的锁存故障，重置PID与旧目标，以0rpm授权，不恢复旧转速。enabled不等于C610物理使能应答。
- `setVelocityRpm(target)`：有限且在±max_rpm内，直接调用现有速度控制器参考；可在旋转中更新，非法值不覆盖旧有效目标。无位置Busy或motion_complete判据。
- `stop()`：直接设0rpm并保持软件使能，可产生制动电流，不锁位置；`disable()`及故障取消授权、清PID/旧目标并缓存零电流。它们都不直接插入CAN帧。
- 反馈断开、发送失败或500ms串口超时锁存故障，正常1kHz聚合时隙发零；恢复反馈/新心跳不会自动使能。低速样本仅按新反馈累计，不以固定延时或重复缓存当稳定。

```cpp
auto& a = *m2006_motors[0]; // M2006Velocity，C610 ID1
auto& b = *m2006_motors[1]; // 独立实例，C610 ID2
// 示例不是自动执行流程；先确认机械条件，检查每次返回结果。
auto enabled = a.enable();
// 仅在 enabled == device::M2006Velocity::Result::Ok 后，
// 由操作者明确选择安全转速并调用 a.setVelocityRpm(target_rpm)。
auto feedback_a = a.feedback();
auto feedback_b = b.feedback();
```

串口逻辑ID仍0x21/0x22。新 `MotorVelocity=0x28` 携带一个float32 rpm；旧Move/SetZero对M2006返回105，不能把旧角度当速度。Info kind从3改为5，State unit=4，位置仅P16诊断deg，其余速度/目标/参考统一rpm；Parameters改为26B六字段。新旧上下位机须配套更新，严格握手拒绝旧清单，详见[SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md)。设备内部Result枚举不得直接解释为串口码；协议统一Offline=1、TransmitFailed=11、LinkFault=107。

Feedback提供connected、enabled、fault、result、position_deg、velocity_rpm、target_rpm、velocity_reference_rpm和feedback_count；速度没有到位。断线测量可能是旧缓存。`m2006_tx_failures`仅表示提交失败，不证明线上发送或物理停止。


### CAN3 与定时交接

FDCAN3 RX=PD12、TX=PD13，第三路独立收发器和FDCAN3H/L/GND。100MHz内核，Classic/Normal、自动重传Enable；当前用户生成Nominal Prescaler=5、Seg1=14、Seg2=5、SJW=4，速率1Mbps、采样点75%；Data字段在Classic模式无效，不开BRS或TDC。StdFilters=1、ExtFilters=0；库掩码0x200/0x7F0，实例严格接受自己的Classic标准数据帧DLC8。

MessageRAMOffset=1024（32位字），RX FIFO0=16×64bytes，TX FIFO=16×8bytes，其他分配数量0；占字区间[1024,1377)，不与前两路重叠。**当前CAN库即便Classic模式也要求已分配RX元素至少64bytes**，因此不能将RX元素改成8bytes，亦不要把元素数量16改成64。FDCAN3 IT0/IT1均5/0，HAL回调注册已启用。外设由用户在CubeMX配置生成，代码仅启动/注册；现有 `.ioc` 已随功能提交，后续配套配置同样随对应功能核对提交，不擅自改写或重新生成。

TIM7只更新DM轨迹；TIM3逐台调用M2006 `feedbackUpdate()`观察新样本、连接和故障。TIM6仍2kHz驱动DM分组，每第二次回调逐台 `velocityUpdate()` 调用实际速度PID，再用 `DJIMotor::SendIqCommand(group1_4)`统一发送一帧0x200，即每台1kHz。停止/失能/故障不从其他位置突发发送，不立即重试；槽3/4保持零。Watchdog只在TIM3推进，名义反馈失联约20ms，不在速度环重复推进。

速度模式无软件位置边界或硬限位保护。零电流不是切电或机械停止；MCU停机、CAN故障、重力负载与制动能力仍须硬件急停和机械支撑保障。外设、接线、ID1/2及默认方向不改。

### 验证证据与边界

Debug固件构建通过。WSL g++15的ASan/UBSan临时烟测链接实际M2006Velocity、DJI解码/电流打包、MotorVelController、PIDMotor和Watchdog，仅CAN/CMSIS采用边界替身：49/50个新低速帧门禁、正负rpm与电流方向、超限/NaN保留目标、跨360°持续运行、0rpm制动、两实例独立失能、主机/发送/离线故障锁存及显式0目标恢复均通过，共捕获1408次共享0x200提交。它证明软件路径，不证明实际电流、带载响应或机械安全；未烧录或驱动实物。


## 串口上位机控制

完整交接文档：[SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md)，明确为**协议v2**，包括设备编号/单位、固定64B帧、CRC、查询布局、序号缓存、断线恢复、Python编解码和独立CRC向量。旧v1客户端必须更新，不保留把角度误当毫米的兼容分支。

USART1为PA9 TX/PA10 RX、115200/8N1，RX DMA1 Stream0 Circular、TX Stream1 Normal，IRQ均6/0。停等请求支持九个逻辑设备（六DM、双M2006、整个底盘），另有四轮状态查询。DM Move按设备使用mm/deg，M2006使用deg，底盘BodyVelocity用m/s和deg/s；可查询当前参数，DM可在失能时修改运动约束。广播State/Disable/Stop须收齐九帧，Wheels须收齐四帧。原DM邮箱保留但不会覆盖串口，底盘/M2006无Ozone运动入口。

首个有效请求建立会话，建议每100ms新心跳；>=500ms无有效请求取消全部六DM、双M2006和底盘的运行授权，并按各自周期请求失能/零电流，不立即重试。新Heartbeat只恢复链路门禁，不自动使能/恢复目标；M2006故障后需失能静止显式SetZero再Enable。Stop不是失能，底盘Stop为零速度而非位置锁定。实际停机需新反馈确认，软件断线处理不是硬件急停；没有Flash参数保存。

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

`dm_startup=Ready`（1）只证明初始化流程完成，不代表全部电机在线。按前表选择要测试的设备索引，确认指针非空再展开。以下以 `[0]` 为例：

```text
connected=true，feedback_count持续增加
zeroed=true，limits_configured=true
motor_state=0，enabled=false，fault=false
command=0，result=0
```

默认限位自动应用，不用首次再手填五个值。就绪依赖有效反馈，不用固定延时假定完成。

### 2. 单台使能与小目标

1. 以下角度示例只用于关节：机构支撑可靠、硬件急停可用后，在 Watch 中写 `dm_motors[3]->command=2`；伸缩使用上文毫米软件接口，不用 target_deg/Move(4)。
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

失能、归零且 command=0 时，可以编辑 `max_velocity_deg_s`、`max_acceleration_deg_s2`、`max_jerk_deg_s3`，再提交 `command=6`；规划内核仍用角度单位，伸缩等效毫米值按上述比例换算。关节的 min_deg/max_deg 必须在−180°～+180°内、min<max且整个区间可编码；伸缩 min/max 不构成软件行程边界。只改字段不会应用，非法动态参数不覆盖有效配置；limits_configured=true 不代表刚写的值被接受。下一次关节软件归零会重新建立完整±180°，不沿用此前缩小的区间。

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

## 目录和维护

- [UserCode/app.cpp](UserCode/app.cpp)：独立设备实例组装、初始化、任务及薄定时调度。
- [UserCode/config.hpp](UserCode/config.hpp)：原 app 中的 DM、底盘方向、M2006 编号/控制和轨迹参数统一入口；构造时复制，不是在线调参接口。
- [UserCode/device/dm_position_test.hpp](UserCode/device/dm_position_test.hpp)、[实现](UserCode/device/dm_position_test.cpp)：命令、软件零点、运动限值、轨迹与 MIT 输出。
- [UserCode/communication/](UserCode/communication/)：固定帧/CRC 编解码、UART DMA 生命周期、上位机命令仲裁及状态应答；协议见 [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md)。
- [UserCode/device/dm_omni_chassis.hpp](UserCode/device/dm_omni_chassis.hpp)、[实现](UserCode/device/dm_omni_chassis.cpp)：FDCAN2 四轮电机、内部轮速控制器与 Omni4 组合，以及失能通信/反馈。
- [UserCode/device/m2006_position.hpp](UserCode/device/m2006_position.hpp)、[实现](UserCode/device/m2006_position.cpp)：单台 M2006 的外部轨迹前馈、位置PD/速度PID、零点/启停/故障和反馈快照。
- [DMDebug.ioc](DMDebug.ioc)：CubeMX 配置唯一来源。
- `Modules/`：cpkg 管理的 Git 子模块；本工程只调用上游接口，不维护私改驱动副本。
- `cmake/`：已跟踪工具链和构建接入；生成内容按忽略规则保留在本地。

开发在功能分支逐模块提交，先推送远端，再通过 GitHub PR 在云端合并；不在本地直接合并到 main。参考参数图片和板卡 PDF 不随此仓库发布。
