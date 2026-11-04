# FDCAN_DM_TEST

STM32H723VET6 单条 FDCAN 总线驱动六个达妙电机的位置测试工程。主控使用已有 SCurve 算法生成位置、速度参考，通过 MIT 五元组发送，不在 MCU 中建立外部位置 PD 或速度 PID。

> 用户已确认代码烧录并测试可行；本轮工具侧也通过固件构建和主机边界模拟。实测负载、持续时间及详细工况未提供，不据此宣称全工况稳定或机械安全；默认增益仍需按具体机构评估。

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

S3519 位置口径保留用户此前实测及上游驱动的处理：反馈位置为减速前角度，驱动除以 `19.203` 换算输出轴角度；应用发送 MIT 位置时反向转换。软件目标和反馈以**输出轴度数**表示，操作者不要额外乘除减速比。S3519 速度沿用驱动的输出轴口径。伸缩电机没有毫米/丝杠导程换算。

## 硬件和定时分工

- STM32H723VET6；FDCAN1 的 RX 为 PB8，TX 为 PB9；通过板载收发器接 CAN_H/CAN_L。
- FDCAN 内核时钟 100 MHz；仲裁段 1 Mbps，数据段 5 Mbps，FD+BRS。电机端需匹配实际时序，不仅是界面中的“5M”标签。
- 上游 `DMMotor::CAN_FilterInit(can, index)` 使用标准 Mask 过滤器：`id1=0x028`、`id2=0x7F8`，覆盖合法反馈 ID。每实例及应用再次同时检查反馈 ID 与载荷电机 ID。
- 所有电机完成构造、接收回调注册后才启动共享总线。

| 执行位置 | 频率 | 用途 |
|---|---:|---|
| 命令任务 | 约 10 ms 轮询 | 处理 Ozone 命令、求解新曲线 |
| TIM7 | 100 Hz | 推进轨迹位置和速度参考 |
| TIM3 | 500 Hz | 更新反馈、软件零点、到位和故障状态；唯一软件 Watchdog 推进者 |
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

依赖由 [wtrproject.toml](wtrproject.toml) 声明，`Trajectory::MotorTrajectory` 传递提供当前使用的 `VelocityProfile::SCurve`；`Chassis::Omni4` 提供四全向轮运动学并传递引入轮速控制器。不手工编辑模块源码或生成的包 CMake 文件。同步会拉取配置的依赖分支，请留意子模块版本变更；已提交的子模块引用记录交付时的版本。

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
| 四个伸缩 | −30°～+30° | 10°/s | 20°/s² | 100°/s³ |
| 两个旋转 | −180°～+180° | 10°/s | 20°/s² | 100°/s³ |

MIT 初始参数为 `kp=5.0`、`kd=0.5`、`torque_ff_nm=0.0`，到位误差阈值 `0.3°`。静止观察阈值：伸缩 `1.1 rpm`、旋转 `0.1 rpm`，考虑不同协议量程下的速度分辨率。这些都不是实物安全边界或已整定参数。

上电不会自动使能。每台在收到 50 次新的失能、静止且位置稳定的反馈后建立软件零点，自动检查并应用默认运动限值。如果整个默认范围在当前绝对零点下超出 PMAX，保留 `limits_configured=false` 并拒绝使能，不静默截断或改写 PMAX。S3519 当前输出轴绝对可编码范围约 ±37.3°，不因机械可旋转 360° 而扩大。

固定 MIT 参数入口为 [UserCode/app.cpp](UserCode/app.cpp) 的 `dm_control`，在设备构造时复制；初始化后只改数组不会在线生效。修改代码、构建并安全重启，或在初始化前设置。机械运动限值则支持失能状态下通过 Ozone 覆盖。

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

1. 机构支撑可靠、硬件急停可用后，在 Watch 中写 `dm_motors[0]->command=2`。
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
| 1 | SetZero | 仅失能、静止且有效反馈时接受；不写电机 Flash，已应用限位同步平移以保留物理范围 |
| 2 | Enable | 人工请求使能，不自动执行旧目标 |
| 3 | Disable | 请求失能，在本电机下一发送时隙提交 |
| 4 | Move | 规划并执行 `target_deg` |
| 5 | Stop | 取消轨迹并保持当前位置，仍可能有力矩；不是平滑减速或硬件急停 |
| 6 | ApplyLimits | 在失能状态提交五个暂存运动限制，非法值不覆盖有效配置 |

失能后必须确认新的 `connected=true`、`motor_state=0`、`enabled=false`，且反馈计数继续增加，不能将旧缓存或断线当作已经停机。重新设零后若要运动，等待静止反馈满足条件，再人工使能。复位会重新归零和应用默认值，不保存运行时覆盖参数。

首次测试逐台完成，随后才在同一总线上分别使能六台、提交经过确认安全的小目标。各台独立规划，不保证同时启动、同时到达；没有全体使能广播命令。

### 4. 覆盖限位与故障定位

失能、归零且 command=0 时，可以编辑设备字段：`min_deg`、`max_deg`、`max_velocity_deg_s`、`max_acceleration_deg_s2`、`max_jerk_deg_s3`，再提交 `command=6`。只改字段不会应用，必须检查本次 `result`；`limits_configured=true` 不代表刚写的非法值被接受。

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
- [DMDebug.ioc](DMDebug.ioc)：CubeMX 配置唯一来源。
- `Modules/`：cpkg 管理的 Git 子模块；本工程只调用上游接口，不维护私改驱动副本。
- `cmake/`：已跟踪工具链和构建接入；生成内容按忽略规则保留在本地。

开发在功能分支逐模块提交，先推送远端，再通过 GitHub PR 在云端合并；不在本地直接合并到 main。参考参数图片和板卡 PDF 不随此仓库发布。
