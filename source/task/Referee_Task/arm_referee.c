#include "DBusSys.h"
#include "arm_state_machine.h"
#include <stdint.h>
#include "cmsis_os2.h"
#include "auto_keyboard.h"

extern rc_info_t remoter;

static const auto_key_cmd_t pos_cmds[3] = {
    CMD_AUTO_GET_A_POS,
    CMD_AUTO_GET_B_POS,
    CMD_AUTO_GET_C_POS,
};
static const auto_key_cmd_t set_cmds[3] = {
    CMD_AUTO_GET_A_SET,
    CMD_AUTO_GET_B_SET,
    CMD_AUTO_GET_C_SET,
};

void Arm_Keyboard_Manager(uint8_t key) {
  static uint8_t pos_idx = 0;
  static uint8_t set_idx = 0;

  if (key == (uint8_t)'F') {
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
  }
  if (key == (uint8_t)'Q') {
    Arm_Current_Control_Mode = Arm_Auto_Mode;
    auto_key_cmd_exec(pos_cmds[pos_idx]);
    pos_idx = (pos_idx + 1) % 3;
  }
  if (key == (uint8_t)'V') {
    Arm_Current_Control_Mode = Arm_Auto_Mode;
    auto_key_cmd_exec(set_cmds[set_idx]);
    set_idx = (set_idx + 1) % 3;
  }

}
