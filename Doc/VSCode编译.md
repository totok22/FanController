# 在 VSCode 中编译 FanController

## STM32 扩展

`FanController.ioc` 已使用 CMake 工具链并生成 `CMakeLists.txt`、`CMakePresets.json` 和 `cmake/`。VSCode STM32 扩展可直接使用这些配置，不需要手动维护很长的 `c_cpp_properties.json`、`tasks.json` 或 `launch.json`。

## 编译

确保 `cmake`、`ninja` 和 `arm-none-eabi-*` 工具位于 `PATH`，然后执行：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

清理后重新编译：

```bash
cmake --build --preset Debug --target clean
cmake --build --preset Debug
```

生成文件：

- `build/Debug/FanController.elf`
- `build/Debug/FanController.hex`
- `build/Debug/FanController.bin`
- `build/Debug/FanController.map`

## 修改 CubeMX 配置

用 STM32CubeMX 打开 `FanController.ioc`，保持 Toolchain/IDE 为 CMake，修改后重新生成代码。业务代码放在 `fan_controller.c/.h` 中；生成文件中的自定义代码只放在 `USER CODE BEGIN/END` 区域内。重新生成后还要确认根目录 `CMakeLists.txt` 中仍包含 `fan_controller.c`。
