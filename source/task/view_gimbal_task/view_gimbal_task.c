#include "referee_api.h"
#include "cmsis_os2.h"
#include "arm_handle.h"
#include "servo_drv.h"
#include <stdbool.h>
#include <stdint.h>
#include "arm_debug.h"

extern keyboard_t kb_info;
uint16_t if_ctrl;
uint16_t if_a ; 
uint16_t if_s ; 
uint16_t if_d ; 
uint16_t if_w;

static inline void key_bits_update(void )
{    
 if_ctrl = kb_info.key_code.bit.CTRL;
 if_a = kb_info.key_code.bit.A;
 if_s = kb_info.key_code.bit.S;
 if_d = kb_info.key_code.bit.D;
 if_w = kb_info.key_code.bit.W;
  }
static inline void key_to_motion(void)
  {
    if (if_ctrl && if_a ) {
      yaw_motion = GIMBAL_RIGHT;
    }
    else if (if_ctrl && if_d ) {
      yaw_motion = GIMBAL_LEFT;
    }
    else if (if_ctrl && if_w ) {
      pitch_motion = GIMBAL_LEFT;
    }
    else if (if_ctrl && if_s ) {
      pitch_motion = GIMBAL_RIGHT;
    }
    else{
      pitch_motion = GIMBAL_STAY;
      yaw_motion = GIMBAL_STAY;
    }
  }



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
static bool gimbal_init = false;

void View_Gimbal_Task(void *argument){

  UNUSED(argument);
  
  if (!gimbal_init) {
    gimbal_init = true;
  servo_init(&view_gimbal_yaw, &htim1, TIM_CHANNEL_1);
  servo_init(&view_gimbal_pitch, &htim1, TIM_CHANNEL_3);

  servo_setPos(&view_gimbal_pitch, 0);
  servo_setPos(&view_gimbal_yaw,90);
  }
  while(1)
  {
    key_bits_update();
    key_to_motion();
    // temp_handle();
    #if !SERVO_DEBUG
      view_gimbal_motion_handle(&view_gimbal_yaw, yaw_motion);
      view_gimbal_motion_handle(&view_gimbal_pitch, pitch_motion);
    #endif
    servo_drive(&view_gimbal_pitch);
    servo_drive(&view_gimbal_yaw);
    servo_limit(&view_gimbal_pitch, 0,180);
    servo_limit(&view_gimbal_yaw, 0, 180);
    osDelay(10);
  }
}
