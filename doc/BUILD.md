# 构建说明

以下命令均从固件仓库根目录执行；首次克隆后的 CubeMX 代码生成不可省略。

## 获取依赖和构建

需要 STM32CubeMX（对应 `.ioc` 的固件包）、Arm GNU Toolchain（`arm-none-eabi-` 工具须在 PATH 中）、CMake ≥ 3.22、Ninja、cpkg。固件使用 C++20。已有构建记录使用 Arm GNU 14.3 Rel1。

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

依赖由 [wtrproject.toml](../wtrproject.toml) 声明，`Trajectory::MotorTrajectory` 传递提供当前使用的 `VelocityProfile::SCurve`；`Chassis::Omni4` 提供四全向轮运动学并传递引入轮速控制器；`MotorDrivers::DJI` 提供新增 M2006/C610 的反馈解码和电流聚合发送。不手工编辑模块源码或生成的包 CMake 文件。同步会拉取配置的依赖分支，请留意子模块版本变更；已提交的子模块引用记录交付时的版本。

本仓库按现有 `.gitignore` 不提交 `Core/`、`Drivers/`、`Middlewares/`、启动汇编、CubeMX 生成的 CMake 子目录以及构建产物，**新克隆不能跳过代码生成**：

1. 使用 CubeMX 打开 [DMDebug.ioc](../DMDebug.ioc)，安装/选择它要求的 STM32H7 固件包，保持目标为 CMake 并生成代码。
2. 保留根 CMake 的 C++20、UserCode 和 cpkg 接入；不要用其他工程的生成文件替代。
3. 执行 cpkg 同步，生成被忽略的 `cmake/wtr_modules.cmake` 和包构建文件。
4. 在工具链、CMake、Ninja 可用的终端执行：

   ```sh
   cpkg sync
   cmake --preset Debug
   cmake --build --preset Debug
   ```

产物：`build/Debug/DMDebug.elf`。下载前先隔离动力并支撑机构；本工程的构建命令不会自动烧录。

