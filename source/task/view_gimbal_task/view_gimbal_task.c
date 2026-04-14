#include "cmsis_os2.h"
#include "arm_handle.h"
#include "servo_drv.h"
#include <stdint.h>


extern uint8_t yaw_motion;
extern uint8_t pitch_motion;
typedef enum {
  GIMBAL_STAY= 0,
  GIMBAL_LEFT,
  GIMBAL_RIGHT
} gimbal_motion_t;

void view_gimbal_motion_handle(servo_t *servo, uint8_t motion) {
  switch (motion) {
  case GIMBAL_LEFT:
    servo_addPos(servo, -1);
    break;
  case GIMBAL_RIGHT:
    servo_addPos(servo, 1);
    break;
  case GIMBAL_STAY:
    break;
  }
}

servo_t view_gimbal_yaw,view_gimbal_pitch;
extern custom_controller_parsed_data_t custom_controller_parsed_data;
void temp_handle(void){
  if (custom_controller_parsed_data.gimbal_cmd[0] == 1) {
    servo_addPos(&view_gimbal_pitch, 1);
  }
}
void View_Gimbal_Task(void *argument){
  UNUSED(argument);
  
  servo_init(&view_gimbal_yaw, &htim1, TIM_CHANNEL_1);
  servo_init(&view_gimbal_pitch, &htim1, TIM_CHANNEL_3);

  servo_setPos(&view_gimbal_pitch, 90);
  servo_setPos(&view_gimbal_yaw, 90);
  while(1)
  {
    // temp_handle();
    view_gimbal_motion_handle(&view_gimbal_pitch, custom_controller_parsed_data.gimbal_cmd[0]);
    view_gimbal_motion_handle(&view_gimbal_yaw, custom_controller_parsed_data.gimbal_cmd[1]);
    servo_drive(&view_gimbal_pitch);
    servo_drive(&view_gimbal_yaw);
    osDelay(10);
  }
}
