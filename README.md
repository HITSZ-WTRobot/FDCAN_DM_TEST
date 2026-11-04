# FDCAN_DM_TEST

STM32H723VET6 双 FDCAN 工程：FDCAN1 保留六个达妙电机的位置测试，使用已有 SCurve 生成位置、速度参考并发送 MIT 五元组，不在 MCU 中建立外部位置 PD 或速度 PID；FDCAN2 新增四个 DM-S3519 的全向轮底盘结构，目前仅失能轮询和反馈观察，不接入底盘运动命令。

> 用户此前确认可烧录运行的是原六电机版本，不是本轮新增底盘或上轮串口实现。本分支从 origin/main 单独开发，未带入上轮未达到可用程度的串口测试。底盘本轮通过 Debug 构建和主机边界烟测，尚无实板运动验证；不宣称带载稳定或机械安全。

## 电机配置

电机端必须预先设置为 **MIT 模式**，Master ID 按上游驱动要求为 **`0x20 + CAN ID`**。固件不会自动写入模式、编号、波特率或电机 Flash。

| Ozone 索引 | 部位 | 型号 | CAN ID | Master ID | PMAX（rad） | VMAX（rad/s） | TMAX（Nm） |
|---|---|---|---|---|---:|---:|---:|
| `dm_motors[0]` | 右前伸缩 | DM-S3519 | `0x0A` | `0x2A` | 12.5 | 200 | 10 |
| `dm_motors[1]` | 中伸缩 | DM-S3519 | `0x0B` | `0x2B` | 12.5 | 200 | 10 |
| `dm_motors[2]` | 后伸缩 | DM-S3519 | `0x0C` | `0x2C` | 12.5 | 200 | 10 |
| `dm_motors[3]` | 左前旋转 | DM-J4340P | `0x0D` | `0x2D` | 12.5 | 10 | 28 |
| `dm_motors[4]` | 左前伸缩 | DM-S3519 | `0x0E` | `0x2E` | 12.5 | 200 | 10 |
| `dm_motors[5]` | 右前旋转 | DM-J4340P | `0x0F` | `0x2F` | 12.5 | 10 | 28 |

PMAX/VMAX/TMAX 是协议映射量程，不是机械运动限制，必须与电机端一致。旧参数图片中的共用 Master ID `0x20` 已不适用于本版固件。

S3519 位置口径保留用户此前实测及上游驱动的处理：反馈位置为减速前角度，驱动除以 `19.203` 换算输出轴角度；应用发送 MIT 位置时反向转换。规划器内部仍使用输出轴度数，四个伸缩对外提供毫米接口，按用户给定的标称 **226.2 mm/输出轴圈**换算；操作者不额外乘除内置减速比。S3519 速度沿用驱动的输出轴口径，毫米速度反馈从 rpm 独立换算。

## 硬件和定时分工

- STM32H723VET6；FDCAN1 的 RX 为 PB8，TX 为 PB9；通过板载收发器接 CAN_H/CAN_L。
- FDCAN 内核时钟 100 MHz；仲裁段 1 Mbps，数据段 5 Mbps，FD+BRS。电机端需匹配实际时序，不仅是界面中的“5M”标签。
- 上游 `DMMotor::CAN_FilterInit(can, index)` 使用标准 Mask 过滤器：`id1=0x028`、`id2=0x7F8`，覆盖合法反馈 ID。每实例及应用再次同时检查反馈 ID 与载荷电机 ID。
- 两条总线各由一个 Can 对象管理；全部十台电机完成构造、接收回调注册后才启动总线，底盘不复用 FDCAN1 实例。

| 执行位置 | 频率 | 用途 |
|---|---:|---|
| 命令任务 | 约 10 ms 轮询 | 原六电机 Ozone 命令/曲线求解，以及四轮底盘反馈快照；不提供底盘命令 |
| TIM7 | 100 Hz | 推进轨迹位置和速度参考 |
| TIM3 | 500 Hz | 原六电机反馈/故障处理、FDCAN2 四轮失能轮询；唯一软件 Watchdog 推进者 |
| TIM6 | 2 kHz | 每次交替发送三台；每台名义更新 1 kHz |
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
| 两个旋转 | −180°～+180° | 10°/s | 20°/s² | 100°/s³ |

MIT 初始参数为 `kp=5.0`、`kd=0.5`、`torque_ff_nm=0.0`，到位误差阈值仍为 `0.3°`（伸缩等效 `0.1885 mm`）。伸缩动态参数为原 `10°/s、20°/s²、100°/s³` 的等效毫米值，未擅自加快运动。静止观察阈值仍为伸缩 `1.1 rpm`、关节 `0.1 rpm`，考虑不同协议量程下的速度分辨率。这些都不是实物安全边界或已整定参数。

上电不会自动使能。每台收到 50 次新的失能、静止且位置稳定反馈后尝试建立软件零点。两个关节初次及后续软件设零都以新原点重新建立 **−180°～+180°**，不再平移旧限位；这会改变所允许的实际运动区域，操作者负责确认空间。若新原点的完整 ±180° 范围不能编码，则拒绝此次设零，不截断范围、不修改旧零点/目标/已应用参数；首次失败保持 zeroed/limits_configured=false。四个伸缩不检查软件 min/max 行程，但当前零点和每个轨迹参考仍必须可编码；limits_configured 对伸缩只表示轨迹动态参数已校验。

固定 MIT 参数入口为 [UserCode/app.cpp](UserCode/app.cpp) 的 `dm_control`，在设备构造时复制；初始化后只改数组不会在线生效。修改代码、构建并安全重启，或在初始化前设置。关节限位可在 ±180° 内缩小，动态约束支持失能时显式提交；伸缩 min_deg/max_deg 不作为行程限位。

### 伸缩毫米位置与软件设零接口

四个伸缩为 `dm_motors[0]`（右前）、`[1]`（中）、`[2]`（后）、`[4]`（左前）；关节为 `[3]`、`[5]`。构造时显式指定 PositionKind，避免仅凭调用数值猜单位。以下是软件调用示例，不会在固件启动时自动执行，也未接入串口：

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

“无伸缩行程限位”仅取消原 ±30° 的软件区间，不修改 PMAX、驱动坐标映射或机械保护。当前 PMAX=12.5 rad、既有 S3519 位置比例 19.203，对应协议原点附近约 **±37.296°输出轴角度，即 ±23.434 mm**。这不是以任意软件零点为中心的固定区间：重新设零后可编码的正负剩余距离通常不对称；不能因为 226.2 mm/圈就接受 226.2 mm 目标。软件设零不重置驱动累计角度，也不会让绝对 MIT 保持目标跳变；不写 Flash、重启后重新建立原点。若机构需要超出当前编码边界的行程，须先确认协议/电机配置方案，不能靠去掉校验或静默限幅实现。

本轮验证使用真实 DMPositionTest、DM 驱动与 SCurve，CAN/机构为理想参考跟随及真实协议量化的边界模型，ASan/UBSan 烟测通过：六台上电稳定归零、关节±180°端点及重新定中心、无法编码的新零点拒绝且旧状态保持、四台20mm绝对轨迹（超过旧软件行程）、重复绝对目标不累加、伸缩重归零与保持参考无跳变、错误单位/NaN/Inf/PMAX/Busy拒绝，以及原每台1kHz分组发送节拍。毫米动态参数及约±23.434mm绝对编码边界另经独立计算核对。未烧录或验证本轮实板运动；226.2mm/圈为用户提供的标称值，不是工具完成的实物标定。


## 四轮全向底盘结构（FDCAN2）

本轮只完成真实设备与运动学库的组装、独立总线启动、失能轮询和反馈，不提供底盘串口收发、Ozone 控制、整车 Controller、定位或位置跟踪。没有空串口处理函数或自写运动学。原六电机仍按后文测试步骤工作，不能用其 `dm_motors` 变量控制底盘。

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
- 启动构造四个 DMMotor，再构造四个 MotorVelController（显式 InternalVel）、最后构造库 Omni4。固定存储显式构造，永久存活，无新增堆分配。构造和 start() 均不请求使能；启动检查滤波、延迟补偿与 CAN start 的返回值，失败不启动定时轮询、不自动恢复。
- 当前 TIM3 每 2 ms 在全局唯一 Watchdog::EatAll() 后发送四帧**失能**请求；每台 500 次/s、一秒四台共 2000 次提交，失败不立即重试。与 FDCAN1 的 TIM6 分组发送相互独立。该节拍只用于失能状态通信，不宣称为未来运行时底盘控制周期。
- 本轮没有可使能底盘的操作入口；Omni4 的内部轮速 update() 也不在启动路径调用。内部速度模式 setRef() 会立即发送，且上游控制权释放不保证硬件失能，因此后续接入真实 Controller 时须明确替换失能轮询调度，不能另起串口回调直接写四轮目标。

### 代码与反馈

`UserCode/device/dm_omni_chassis.*` 的 DMOmniChassis 封装上述组合；`UserCode/app.cpp` 仅负责固定存储、FDCAN2 组装、启动及薄调度。`motion()` 返回真实 Omni4 对象，供后续既有 Controller 绑定，不绕过其受保护的速度分解接口。当前没有虚假定位对象或串口协议占位；后续通信层消费真正控制接口时再确定命令契约。

可观察 `chassis_startup`、`chassis_poll_ok` 和 `chassis_feedback`，它们不是控制命令：

- `chassis_startup`：NotStarted=0、Ready=1、FilterFailed=2、DelayCompensationFailed=3、CanStartFailed=4。
- `chassis_poll_ok` 仅表示最近一轮四帧全部提交硬件成功，不证明物理线上发送、收到应答或已经失能。
- `chassis_feedback.wheels[0..3]` 按上表轮位顺序记录 connected、motor state、输出轴 angle_deg、velocity_rpm、torque_nm。约每 10 ms 在短临界区取得一次一致快照，电机连接 Watchdog 由 TIM3 推进，名义约 20 ms。connected=false 时其他值可能是旧缓存，不能作新鲜反馈。
- `velocity_in_body` 实际调用 Omni4::forwardGetVelocity()，单位为 vx/vy=m/s、wz=deg/s；x 向前、y 向左、逆时针为正。只有四轮均有合格新反馈且轮向正确时才有物理意义；它不是整车定位，也不校正打滑。
- `chassis_config.reverse` 在构造时复制；运行后仅改该数组不会在线改变轮向。改代码或在初始化前设置，安全重启后生效；当前不提供在线调参/保存接口。电机端实际模式、反馈编号和轮向仍需在架空、动力安全的条件下确认。

### 本轮验证

Debug 固件构建通过。临时 ASan/UBSan 原生烟测使用真实 DMOmniChassis、DM、MotorVelController、PID 依赖和 Omni4，CAN/HAL 采用边界模型：验证启动失败与重复启动、独立总线、500 次轮询共 2000 帧且全部为失能包、发送失败不重试、提交计数回绕、旧 Master ID/交叉 ID 拒收、Watchdog 断联、前进/横移/旋转/复合速度的 127/435 mm 分解与反解、方向配置。运动分解由临时测试刺激器通过库 Controller 控制权接口触发，不是固件新增的操作入口。临时测试遵守对象永久登记生命周期，未改上游代码；完成后移除临时文件。未烧录、驱动实物或测试真实总线仲裁/电气波形、实际反馈频率、轮向和带载稳定性。


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

通信故障优先检查电机端 Master ID 是否已从旧 `0x20` 改为对应的 `0x2A～0x2F`，再检查编号、总线时序、接线、供电和终端电阻。全局 RX 计数不能替代每台 `feedback_count`。`dm_tx_submitted_frames` 只表示硬件 FIFO 提交，不证明线上发送成功或动作完成。

软件看门狗由 TIM3 推进，名义失联窗口约 20 ms。故障会取消轨迹、锁存并请求失能，不自动重启，也不自动清除电机内部错误。排除原因、确认失能和静止后，再人工 Enable。不要改内部状态标志绕过检查。

## 安全与验证边界

- 参数图片的 CAN Timeout 为 0；主控暂停、断电或停止发包时，电机未必自动失能。
- 先支撑机构，再失能并核对反馈，按现场流程隔离动力后才下载、复位、暂停或断开调试器。Halt、断点和单步不是急停。
- `torque_ff_nm=0` 不代表总力矩为 0，Kp/Kd 仍会产生保持/阻尼力矩；前馈也不是总力矩上限。默认增益是调试初值，用户确认可运行不等于全部负载工况已完成整定。
- 本轮验证使用未修改的上游 DM 驱动、真实应用与 SCurve，CAN/HAL/RTOS 和机构使用主机模型：覆盖独立反馈 ID、交叉/异常帧拒收、旧 Master ID 拒绝连接、六台独立到位和故障处理。
- TIM6 2 kHz 的主机时序验证覆盖 32,820 个发送时隙：每次三台、两组交替；健康的一秒窗口每台提交 1,000 帧。不模拟真实位级仲裁、重发或电气故障，不能代替实板波形与负载检查。
- 实板结果：用户反馈“代码已经烧录并测试过可行了”；工具未直接操作目标，也未为重复确认该结果重跑硬件测试。当前 Debug 构建及主机烟测通过，烟测为本轮临时验证，不是仓库提供的独立测试命令。

## 目录和维护

- [UserCode/app.cpp](UserCode/app.cpp)：参数、设备部位映射、初始化、任务及薄定时调度。
- [UserCode/device/dm_position_test.hpp](UserCode/device/dm_position_test.hpp)、[实现](UserCode/device/dm_position_test.cpp)：命令、软件零点、运动限值、轨迹与 MIT 输出。
- [UserCode/device/dm_omni_chassis.hpp](UserCode/device/dm_omni_chassis.hpp)、[实现](UserCode/device/dm_omni_chassis.cpp)：FDCAN2 四轮电机、内部轮速控制器与 Omni4 组合，以及失能通信/反馈。
- [DMDebug.ioc](DMDebug.ioc)：CubeMX 配置唯一来源。
- `Modules/`：cpkg 管理的 Git 子模块；本工程只调用上游接口，不维护私改驱动副本。
- `cmake/`：已跟踪工具链和构建接入；生成内容按忽略规则保留在本地。

开发在功能分支逐模块提交，先推送远端，再通过 GitHub PR 在云端合并；不在本地直接合并到 main。参考参数图片和板卡 PDF 不随此仓库发布。
