#ifndef ARM_HANDLE_H
#define ARM_HANDLE_H

#include "arm_state_machine.h"
#include "trajectory_publisher_drv.h"

extern osThreadId_t Trajectory_PublisherHandle;
extern target_point_t Target_Point[6];
extern float Target_Joint_Radian[6];

#pragma pack(1)
typedef struct {
  float radian[6];
  uint8_t botton;
  uint8_t gimbal_cmd[2];
} custom_controller_parsed_data_t;
#pragma pack()

void Arm_Auto_Mode_Handle(void);
void Arm_Traj_Handle(void);
void Arm_Transition_Handle(Joint_t*, const float*);
void Arm_Frozen_Handle(void);
void Arm_Custom_Controller_Follow_Handle(void);
#endif
