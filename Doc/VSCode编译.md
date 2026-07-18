# 在 VSCode 中编译 FanController

## STM32 扩展

如果要使用 VSCode STM32 扩展自带的配置、编译和调试功能，应在 STM32CubeMX 的 Project Manager 中把 Toolchain/IDE 设为 CMake，再重新生成工程。这样不需要手动维护很长的 `c_cpp_properties.json`、`tasks.json` 或 `launch.json`。

当前 `FanController.ioc` 仍为 STM32CubeIDE 工具链配置，因此本仓库先保留独立 Makefile。后续切换为 CMake 并重新生成代码后，应以 CubeMX 生成的 CMake 配置为准。

## 编译

在 VSCode 终端执行：

```bash
make -j4
```

清理后重新编译：

```bash
make clean
make -j4
```

生成文件：

- `Debug/FanController.elf`
- `Debug/FanController.hex`
- `Debug/FanController.bin`
- `Debug/FanController.map`

Makefile 会优先使用 macOS 上的 `/Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin`；该路径不存在时使用环境变量 `PATH` 中的 `arm-none-eabi-*` 工具。

## 修改 CubeMX 配置

用 STM32CubeMX 打开 `FanController.ioc`，修改后重新生成代码。需要 VSCode STM32 扩展完整接管构建时，同时把 Toolchain/IDE 改为 CMake。业务代码放在 `fan_controller.c/.h` 中；生成文件中的自定义代码只放在 `USER CODE BEGIN/END` 区域内。
