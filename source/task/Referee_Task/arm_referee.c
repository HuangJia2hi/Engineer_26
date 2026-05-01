#include "arm_referee.h"
extern rc_info_t remoter;
#include "auto_keyboard.h"
static const auto_key_cmd_t pos_cmds[3] = {
    CMD_AUTO_GET_A_POS,
    CMD_AUTO_GET_B_POS,
    CMD_AUTO_GET_C_POS,
};

void Arm_Keyboard_Manager(uint8_t key) {
  static uint8_t pos_idx = 0;

  if (key == (uint8_t)'F') {
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
  }
  if (key == (uint8_t)'Q') {
    Arm_Current_Control_Mode = Arm_Auto_Mode;
    auto_key_cmd_exec(pos_cmds[pos_idx]);
    auto_key_cmd = pos_cmds[pos_idx];
    pos_idx = (pos_idx + 1) % 3;
  }

}

void Arm_Keyboard_ctrl_Manager(uint8_t key) {
  if (key == 'Q') {
    Arm_Current_Control_Mode = Arm_Auto_Mode;

    if (auto_key_cmd == CMD_AUTO_GET_A_POS) {
      auto_key_cmd_exec(CMD_AUTO_GET_A_SET);
    }

    if (auto_key_cmd == CMD_AUTO_GET_B_POS) {
      auto_key_cmd_exec(CMD_AUTO_GET_B_SET);
    }

    if (auto_key_cmd == CMD_AUTO_GET_C_POS) {
      auto_key_cmd_exec(CMD_AUTO_GET_C_SET);
    }
  }
}
