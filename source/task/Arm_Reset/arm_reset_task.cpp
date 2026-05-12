extern "C" {
#include "jointFollowAngle.h"
#include "DBusSys.h"
#include "arm_state_machine.h"
#include "arm_debug.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "joint_control_drv.h"
}

static inline bool Motor_Disable_Detect(Joint_t *Joint)
{
  return (Joint->joint_motor->error_code == Motor_DM_DISABLE);
}
void Arm_Reset(Joint_t* Joint)
{
  for (int joint_index = 0; joint_index<JOINT_NUM; joint_index++) {
  if (Motor_Disable_Detect(&Joint[joint_index])) {
    Joint_Motor_Enable(Joint); 
    osDelay(1);
    EndEffector_Motor_Enable(&EndEffector);
  }
  }
}

extern "C" void arm_reset_task(void *argument)
{
    UNUSED(argument);    
    while (1) {
    
    Arm_Reset(Joint);
    osDelay(100);
    }
}
