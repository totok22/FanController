#ifndef FAN_CONTROLLER_H
#define FAN_CONTROLLER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* CANB 温度输入，Intel 字节序，温度单位为 0.1℃/LSB。 */
#define FAN_CAN_ID_MOTOR_TEMP          0x506U
#define FAN_CAN_ID_INVERTER_TEMP       0x507U
#define FAN_CAN_ID_IGBT_TEMP           0x508U

/* 风扇控制器报文，均使用 CAN 标准帧。 */
#define FAN_CAN_ID_STATUS               0x5A2U
#define FAN_CAN_ID_DIAGNOSTIC           0x5A3U
#define FAN_CAN_ID_COMMAND              0x5A4U
#define FAN_CAN_ID_COMMAND_ACK          0x5A5U
#define FAN_CAN_ID_CURVE_STATUS         0x5A6U
#define FAN_CAN_ID_FAILSAFE_STATUS      0x5A7U

void FanController_Init(void);
void FanController_Update(void);
void FanController_OnCanFrame(const CAN_RxHeaderTypeDef *header,
                              const uint8_t data[8]);
void FanController_OnInputCapture(TIM_HandleTypeDef *htim);

#endif /* FAN_CONTROLLER_H */
