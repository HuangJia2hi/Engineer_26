#ifndef ARM_STATE_MACHINE_H
#define  ARM_STATE_MACHINE_H

#include "DBusSys.h"
#include "joint_control_drv.h"
#include "ee_control_drv.h"
#include "arm_debug.h"

typedef enum {
  Arm_IDLE_Mode = 0,
  Arm_Custom_Controller_Follow_Mode,
  Arm_Frozen_Mode,
  Arm_Set_Radian,
  Arm_Traj_Mode,
  Arm_Rising_Mode,
  Arm_Zero_Mode,
  Arm_Auto_Mode,
  ARM_FULL_RESET_MODE,
  ARM_RESET_ZERO_MODE,
  ARM_START_MODE,
  ARM_SAFE_MODE,
} arm_control_mode_t;

typedef enum{
    GRIPPER_IDLE_MODE = 0,
    GRIPPER_OPEN_MODE,
    GRIPPER_CLOSE_MODE,
    GRIPPER_SPECI_MODE
}gripper_control_mode_t ;

#ifdef __cplusplus

class GripperStateMachine {
public:
    void update(endEffector_t *ee);
    void setMode(gripper_control_mode_t mode) { mode_ = mode; }
    gripper_control_mode_t getMode() const { return mode_; }
private:
    gripper_control_mode_t mode_ = GRIPPER_IDLE_MODE;
};

extern GripperStateMachine gripperSM;

#endif

extern float Ctrller_Joint_Radian[6];
extern DM_motor_t *Joint_Motor[JOINT_NUM];
extern target_point_t Target_Point[6];
extern arm_control_mode_t Arm_Current_Control_Mode;

#ifdef __cplusplus
extern "C" {
#endif

void gripper_set_mode(gripper_control_mode_t mode);
gripper_control_mode_t gripper_get_mode(void);

/** @brief 关节状态机管理器 */
void Joint_Control_Mode_Manager(Joint_t *Joint);
#ifdef __cplusplus
}
#endif
#endif
