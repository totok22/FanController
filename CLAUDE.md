# FanController - FSAE 风扇控制器

## 项目概述
STM32F103 风扇控制器，通过 CANB 温度报文控制三台散热风扇。

## 硬件配置
- MCU：STM32F103C8T6，72MHz HSE+PLL
- CAN：500kbps，PA11/PA12
- PWM1：PA8 / TIM1_CH1，2kHz，控制两台电机水套风扇
- PWM2：PB8 / TIM4_CH3，2kHz，控制逆变器散热风扇
- TACH1：PA0 / TIM2_CH1
- TACH2：PA6 / TIM3_CH1
- TACH3：PA7 / TIM3_CH2
- UART1：PA9/PA10，调试输出
- LED：PB5，低电平点亮

## 关键文件
- `Core/Src/fan_controller.c` / `Core/Inc/fan_controller.h`：温控、PWM、转速、失联处理和 CAN 协议
- `Core/Src/main.c`：CAN 过滤器、控制器初始化和主循环调用
- `Core/Src/stm32f1xx_it.c`：CAN RX0、TIM2 和 TIM3 中断
- `FanController.ioc`：CubeMX 配置
- `Doc/风扇控制.md`：控制策略和报文定义

## CAN 报文
- 接收：`0x506` 电机温度、`0x507` 逆变器温度、`0x508` IGBT 温度、`0x5A4` 风扇命令
- 发送：`0x5A2` 风扇状态、`0x5A3` 诊断、`0x5A5` 命令应答、`0x5A6/0x5A7` 策略状态
- `0x5A2/0x5A3` 周期为500ms，其余报文按命令返回

## 编码规范
- 只在 `USER CODE BEGIN/END` 块内修改 CubeMX 生成的文件
- 独立业务代码放在 `fan_controller.c/.h` 中
- 使用清楚、直接的中文注释
- 文档和注释避免难懂或含义不明确的词语
