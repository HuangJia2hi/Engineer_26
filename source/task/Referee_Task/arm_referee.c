#include "arm_referee.h"
#include "arm_handle.h"
#include "arm_state_machine.h"
#include "servo_drv.h"
extern rc_info_t remoter;
#include "auto_keyboard.h"
#include "arm_debug.h"
static const auto_key_cmd_t pos_cmds[3] = {
    CMD_AUTO_GET_A_POS,
    CMD_AUTO_GET_B_POS,
    CMD_AUTO_GET_C_POS,
};

static uint8_t get_idx = 0;

static const auto_key_get_cmd_t get_cmds[4] = {
    CMD_AUTO_GET_RIGHT_BACK,
    CMD_AUTO_GET_RIGHT_MID,
    CMD_AUTO_GET_RIGHT_FRONT,
    CMD_AUTO_GET_LEFT_FORNT,
};

void Arm_Keyboard_E_Exec(void) {
  Arm_Current_Control_Mode = Arm_Auto_Mode;
  auto_key_get_cmd_exec(auto_key_get_cmd);

}

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
   if (key == (uint8_t)'E') {
       Arm_Keyboard_E_Exec();
       get_idx = (get_idx + 1) % 4;
       auto_key_get_cmd = get_cmds[get_idx];
   }   
   if (key == (uint8_t)'V') {
    Arm_Current_Control_Mode = Arm_Rising_Mode;
   }
}

void Arm_Keyboard_ctrl_Manager(uint8_t key) {

    // if (key == (uint8_t)'W')
    // {
    //   servo_addPos(&view_gimbal_pitch , 1);uint8_t yaw_motion = 0;

    // else if (key == (uint8_t)'S') {
    //   servo_addPos(&view_gimbal_pitch , -1);
    // }
    // else if (key == (uint8_t)'A') {

    //   servo_addPos(&view_gimbal_yaw , 1);
    // }
    // else if (key == (uint8_t)'D') {

    //   servo_addPos(&view_gimbal_yaw , -1);
    // }
    // else {
    // }

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

  if (key == 'E') {
    get_idx = (get_idx + 1) % 4;
    auto_key_get_cmd = get_cmds[get_idx];
  }
}
void Arm_Keyboard_shift_Manager(uint8_t key){
    
}
