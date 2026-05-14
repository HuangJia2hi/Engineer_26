#ifndef CHASSIS_YAW_CTRL_H
#define CHASSIS_YAW_CTRL_H

#include "arm_math_types.h"

#include <stdint.h>

void Chassis_YawCtrl_Init(void);
void Chassis_YawCtrl_HoldCurrentAngle(void);
void Chassis_YawCtrl_UpdateTargetFromDbus(int16_t ch3);
void Chassis_YawCtrl_UpdateTargetFromMouse(int16_t mouse_x, uint8_t enable_input, float32_t rate_scale);
float32_t Chassis_YawCtrl_GetClosedLoopWz(void);
float32_t Chassis_YawCtrl_GetCurrentAngle(void);
float32_t Chassis_YawCtrl_GetTargetAngle(void);

#endif /* CHASSIS_YAW_CTRL_H */
