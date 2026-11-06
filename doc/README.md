# 固件文档

- [构建说明](BUILD.md)：工具链、依赖同步、CubeMX 生成与构建产物。
- [设备、硬件与执行参数](DEVICES.md)：六 DM、四轮底盘、双 M2006 的接线、时序、参数、接口和分阶段验证证据。
- [操作、安全与验证边界](OPERATION.md)：串口入口、Ozone 操作、故障恢复、安全要求和历史证据边界。
- [串口协议 v2](SERIAL_PROTOCOL.md)：设备清单、单位、帧布局、CRC、命令、结果码与 Python 参考。
- [上位机通信检查与调参步骤](MOTOR_TEST_GUIDE.md)：逐台通信核对、小运动验证、记录与调参流程。

配套上位机：[dmdebug-station](https://github.com/HITSZ-WTRobot/dmdebug-station)，可从其[发布页](https://github.com/HITSZ-WTRobot/dmdebug-station/releases)获取发布包；使用方法见[上位机文档](https://github.com/HITSZ-WTRobot/dmdebug-station/blob/main/doc/USAGE.md)。

本文档中的 `UserCode/`、`build/`、`.ioc` 等路径，除链接外均相对于固件仓库根目录。当前启动值以源码为准，运行态以新鲜回读为准；任何主机模型或历史构建记录都不代替硬件、机械和带载安全验证。

## 目录和维护

- [UserCode/app.cpp](../UserCode/app.cpp)：独立设备实例组装、初始化、任务及薄定时调度。
- [UserCode/config.hpp](../UserCode/config.hpp)：原 app 中的 DM、底盘方向、M2006 编号/控制和轨迹参数统一入口；构造时复制，不是在线调参接口。
- [UserCode/device/dm_position_test.hpp](../UserCode/device/dm_position_test.hpp)、[实现](../UserCode/device/dm_position_test.cpp)：命令、软件零点、运动限值、轨迹与 MIT 输出。
- [UserCode/communication/](../UserCode/communication/)：固定帧/CRC 编解码、UART DMA 生命周期、上位机命令仲裁及状态应答；协议见 [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md)。
- [UserCode/device/dm_omni_chassis.hpp](../UserCode/device/dm_omni_chassis.hpp)、[实现](../UserCode/device/dm_omni_chassis.cpp)：FDCAN2 四轮电机、内部轮速控制器与 Omni4 组合，以及失能通信/反馈。
- [UserCode/device/m2006_velocity.hpp](../UserCode/device/m2006_velocity.hpp)、[实现](../UserCode/device/m2006_velocity.cpp)：单台 M2006 的直接速度 PID、低速授权、启停/故障和反馈快照；位置仅作诊断。
- [DMDebug.ioc](../DMDebug.ioc)：CubeMX 配置唯一来源。
- `Modules/`：cpkg 管理的 Git 子模块；本工程只调用上游接口，不维护私改驱动副本。
- `cmake/`：已跟踪工具链和构建接入；生成内容按忽略规则保留在本地。

开发在功能分支逐模块提交，先推送远端，再通过 GitHub PR 在云端合并；不在本地直接合并到 main。参考参数图片和板卡 PDF 不随此仓库发布。
