#include "DBusSys.h"
#include "arm_state_machine.h"
#include <stdint.h>
#include "cmsis_os2.h"

extern rc_info_t remoter;
#define COMMAND_TABLE_SIZE 2

void Arm_Keyboard_Manager(uint8_t key) {

  if (key == (uint8_t)'F') {
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
  }
  if (key == (uint8_t)'R') {
    Arm_Current_Control_Mode = Arm_IDLE_Mode;
  }

   
/*   if (key == (uint8_t)'E') {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    cmd_place_get = Command_getRight;
  }

  if (key == (uint8_t)'R') {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    cmd_place_get = Command_placeLeft;
  }

  if (key == (uint8_t)'T') {
    Arm_Current_Control_Mode = Arm_Traj_Mode;
    cmd_place_get = Command_placeRight;
  } */
}
