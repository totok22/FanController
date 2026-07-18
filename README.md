# FanController - FSAE 风扇控制器

基于 STM32F103C8T6 的独立风扇控制板固件。控制器从 CANB 接收电机、逆变器和 IGBT 温度，控制两路 PWM，并测量三台风扇的实际转速。

## 硬件

| 功能 | 引脚与配置 |
| --- | --- |
| CAN | PA11/PA12，500kbps，自动重发和总线关闭自动恢复 |
| PWM1 | PA8 / TIM1_CH1，2kHz，控制两台电机水套风扇 |
| PWM2 | PB8 / TIM4_CH3，2kHz，控制逆变器散热风扇 |
| TACH1 | PA0 / TIM2_CH1，下降沿输入捕获 |
| TACH2 | PA6 / TIM3_CH1，下降沿输入捕获 |
| TACH3 | PA7 / TIM3_CH2，下降沿输入捕获 |
| UART1 | PA9/PA10，115200bps 调试输出 |
| LED | PB5，低电平点亮 |

PWM 信号经过 2N7002 反相。代码和 CAN 报文中的占空比均按风扇侧定义：0%停止，100%全速。

## 主要功能

- 默认温控曲线：40℃启动、30%最低运行占空比、60℃全速、35℃以下停止；
- 两路 PWM 默认以20%/s上升、50%/s下降，两组风扇启动间隔至少2.5秒；
- 温度报文超时后先保持最后目标5秒，再进入50%保底策略；
- 支持带有效时间的手动和关闭模式，超时后自动恢复温控；
- 通过三个输入捕获通道测量转速，并上报停转、温度超时和外设启动故障。
- 每500ms翻转一次 LED，作为主循环运行指示。

详细策略、命令格式和台架检查步骤见 [Doc/风扇控制.md](Doc/风扇控制.md)。DBC 文件为 [Doc/FanController_CANB.dbc](Doc/FanController_CANB.dbc)。

## CAN 报文

| ID | 方向 | 内容 |
| --- | --- | --- |
| `0x506` | 接收 | 四个电机温度 |
| `0x507` | 接收 | 四个逆变器温度 |
| `0x508` | 接收 | 四个 IGBT 温度 |
| `0x5A2` | 发送 | 三个转速和两路当前占空比，500ms周期 |
| `0x5A3` | 发送 | 温度、模式、目标占空比和故障，500ms周期 |
| `0x5A4` | 接收 | 风扇控制命令 |
| `0x5A5` | 发送 | 命令应答 |
| `0x5A6/0x5A7` | 发送 | 当前温控曲线和失联策略 |

## 目录

```text
Core/Inc/fan_controller.h     控制器接口和 CAN ID
Core/Src/fan_controller.c     温控、PWM、测速、诊断和 CAN 命令
Core/Src/main.c               外设启动、CAN 过滤器和回调
FanController.ioc             CubeMX 配置
Doc/风扇控制.md               完整控制说明
```

## 编译

确保 `cmake`、`ninja` 和 `arm-none-eabi-*` 工具位于 `PATH`，然后执行：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

输出文件位于 `build/Debug/FanController.elf`、`build/Debug/FanController.hex` 和 `build/Debug/FanController.bin`。VSCode 使用方式见 [Doc/VSCode编译.md](Doc/VSCode编译.md)。
