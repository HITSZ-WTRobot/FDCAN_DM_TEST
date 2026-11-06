# FDCAN_DM_TEST

STM32H723VET6 三 FDCAN 电机控制固件：FDCAN1 控制六台 DM 位置设备，FDCAN2 控制四轮全向底盘，FDCAN3 控制两台独立 M2006/C610 输出轴速度（±300 rpm）。通过串口协议 v2 与[配套上位机](https://github.com/HITSZ-WTRobot/dmdebug-station)通信。

上电不自动使能。运动前必须核对电机端模式、编号、量程和机械安全条件；软件停机不能替代硬件急停。

## 获取与构建

依赖：STM32CubeMX 及 `.ioc` 对应 STM32H7 固件包、Arm GNU Toolchain（`arm-none-eabi-` 在 PATH 中）、CMake ≥ 3.22、Ninja、cpkg；固件使用 C++20。

```sh
git clone --recurse-submodules https://github.com/HITSZ-WTRobot/FDCAN_DM_TEST.git
cd FDCAN_DM_TEST
```

新克隆先用 CubeMX 打开 `DMDebug.ioc`，保持 CMake 目标并生成代码；保留根 CMake 的 C++20、UserCode 和 cpkg 接入，再执行：

```sh
cpkg sync --submodule-protocol https
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/DMDebug.elf`；上述命令不自动烧录。详细前提见[构建说明](doc/BUILD.md)。

## 文档

- [文档索引与代码地图](doc/README.md)
- [设备、接线、时序与参数](doc/DEVICES.md)
- [串口操作、Ozone、安全与验证边界](doc/OPERATION.md)
- [串口协议 v2 与编解码示例](doc/SERIAL_PROTOCOL.md)
- [上位机通信检查与调参步骤](doc/MOTOR_TEST_GUIDE.md)
