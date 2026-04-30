#include "DBusSys.h"
#include "arm_state_machine.h"
#include <stdint.h>
#include "cmsis_os2.h"
#include "auto_keyboard.h"

extern rc_info_t remoter;
#define COMMAND_TABLE_SIZE 2

void Arm_Keyboard_Manager(uint8_t key) {

  if (key == (uint8_t)'F') {
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
  }
  if (key == (uint8_t)'Q') {
    Arm_Current_Control_Mode = Arm_Auto_Mode;
    auto_key_cmd_exec(CMD_AUTO_GET_A_POS);
  }
  if (key == (uint8_t)'V') {
    Arm_Current_Control_Mode = Arm_Auto_Mode;
    auto_key_cmd_exec(CMD_AUTO_GET_A_SET);
  }

}
