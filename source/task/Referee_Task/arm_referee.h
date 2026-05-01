#ifndef ARM_REFEREE_H
#define ARM_REFEREE_H

#include "DBusSys.h"
#include "arm_state_machine.h"
#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "auto_keyboard.h"

void Arm_Keyboard_Manager(uint8_t key);

void Arm_Keyboard_ctrl_Manager(uint8_t key);

extern bool ctrl_q_isPressed;

#endif
