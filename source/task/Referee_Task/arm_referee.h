#ifndef ARM_REFEREE_H
#define ARM_REFEREE_H

#include "DBusSys.h"
#include "arm_state_machine.h"
#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "auto_keyboard.h"

typedef enum { RESET_SAVE_ZERO_NONE, RESET_SAVE_ZERO_OK } RESET_SAVEZERO_STATUS;
typedef enum { RESET_NONE, RESET_EXIT, RESET_LOGIC_ERROR } EXIT_RESET_STATUS;

void Arm_Keyboard_Manager(uint8_t key);

void Arm_Keyboard_ctrl_Manager(uint8_t key);

void Arm_Keyboard_shift_Manager(uint8_t key);

extern uint8_t Emerency_flag;

#endif
