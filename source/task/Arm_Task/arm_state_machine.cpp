#include "arm_state_machine.h"
#include "arm_handle.h"

#if TRAJ_DEBUG
arm_control_mode_t Arm_Current_Control_Mode = Arm_Auto_Mode;
#else 
arm_control_mode_t Arm_Current_Control_Mode = Arm_IDLE_Mode;
#endif


float Rising_Joint_Radian[6] = {0,1.4,1.3,0,0.4,0};

const float Rising_Velcoity[6] = {
  0.1,0.4,0.8,0,0.4,0
};

static float Zero_Joint_Radian[6] = { 0 ,0.4, 0.5 ,0, 0, 0};
static float Zero_Velocity[6] ={
  0.1,0.4,0.8,0.1,0.1,0.1
};


GripperStateMachine gripperSM;

void GripperStateMachine::update(endEffector_t *ee) {
  switch (mode_) {
  case GRIPPER_IDLE_MODE:
    mode_ = GRIPPER_OPEN_MODE;
    break;
  case GRIPPER_OPEN_MODE:
    Gripper_Open(ee);
    break;
  case GRIPPER_CLOSE_MODE:
    Gripper_Close(ee);
    break;
  case GRIPPER_SPECI_MODE:
    Gripper_Speci(ee);
    break;
  }
}

extern "C" void gripper_set_mode(gripper_control_mode_t mode) {
    gripperSM.setMode(mode);
}

extern "C" gripper_control_mode_t gripper_get_mode(void) {
    return gripperSM.getMode();
}

/**
 * @brief 关节状态机
 *
 * @param Joint
 */
void Joint_Control_Mode_Manager(Joint_t *Joint) {
  switch (Arm_Current_Control_Mode) {
  case ARM_START_MODE:
    ARM_STATRT_UP_HANDLE();
    break;
  case Arm_Rising_Mode:
    Point_Publisher(Target_Point, Rising_Joint_Radian, Rising_Velcoity);
    break;
  case Arm_IDLE_Mode:
    // Arm_Current_Control_Mode = ARM_START_MODE;
#if TRAJ_DEBUG
    Arm_Current_Control_Mode = Arm_Auto_Mode;
#else
    Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;
#endif

    break;

  case Arm_Custom_Controller_Follow_Mode:

    Arm_Custom_Controller_Follow_Handle();

    break;

  case Arm_Frozen_Mode:
    Arm_Frozen_Handle();
    break;

  case Arm_Set_Radian:
    break;
  case Arm_Traj_Mode:
    Arm_Traj_Handle();
    break;
  case Arm_Zero_Mode:
    Point_Publisher(Target_Point, Zero_Joint_Radian, Zero_Velocity);
    break;
  case Arm_Auto_Mode:
    Arm_Auto_Mode_Handle();
    break;
  case ARM_RESET_ZERO_MODE:
    ARM_RESET_ZERO_HANDLE();
    break;
  case ARM_FULL_RESET_MODE:
    break;
case ARM_SAFE_MODE:
    ARM_SAFE_MODE_HANDLE();
    break;
  }
}
