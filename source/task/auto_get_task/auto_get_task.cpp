#include "arm_state_machine.h"
#include "auto_get_timer_init.h"
#include "cmsis_os2.h"
#include "joint_control_drv.h"
#include "stm32h7xx_hal_def.h"
#include "tool.h"
#include <stdint.h>

#define TRAJ_GROUP_SIZE (30)
extern target_point_t Target_Point[6];
struct traj_group_point_t {
  target_point_t q[6];
  uint32_t time;
  gripper_control_mode_t gripper_ctrl;
};
class TrajectoryExecutor {
public:
  void init(traj_group_point_t *traj, uint32_t size, uint32_t period_ms);
  void build_time_acc();
  void update(uint32_t idx, target_point_t *Target_Point);
  void reset();
  uint32_t get_seq(void);

private:
  traj_group_point_t *traj_ = nullptr;

  uint32_t size_ = 0;
  uint32_t period_ms_ = 5;

  uint32_t seg_ = 0;
  int32_t last_seg_ = -1;

  uint32_t time_acc_[30];
};

void TrajectoryExecutor::init(traj_group_point_t *traj, uint32_t size,
                              uint32_t period_ms) {
  this->traj_ = traj;
  this->size_ = size;
  this->period_ms_ = period_ms;

  this->seg_ = 0;
  this->last_seg_ = -1;
}
uint32_t TrajectoryExecutor::get_seq() { return this->seg_; }
void TrajectoryExecutor::reset() {
  seg_ = 0;
  last_seg_ = -1;
}
void TrajectoryExecutor::build_time_acc() {
  uint32_t sum = 0;

  for (uint32_t i = 0; i < size_; i++) {
    sum += traj_[i].time;
    time_acc_[i] = sum;
  }
}

void TrajectoryExecutor::update(uint32_t idx, target_point_t *Target_Point) {
  uint32_t time_ms = idx * period_ms_;

  while (seg_ < size_ && time_ms >= time_acc_[seg_]) {
    seg_++;
  }

  if (seg_ >= size_)
    return;

  if (seg_ == last_seg_)
    return;

  last_seg_ = seg_;

  for (int i = 0; i < 6; i++) {
    Target_Point[i] = traj_[seg_].q[i];
  }
    Gripper_Current_Control_Mode = traj_[seg_].gripper_ctrl;
}
traj_group_point_t traj_group_A[] = {
    {{
         {0, 0.5},
         {0, 0.5},
         {0.2, 0.5},
         {0, 0.5},
         {0, 0.5},
         {0, 1.0},
     },
     5000,
     GRIPPER_OPEN_MODE
    },
    {

        {
            {0, 0.5},
            {0.7, 0.8},
            {0.85, 1.0},
            {0, 0.5},
            {0, 0.5},
            {Pi, 1.0},
        },
        5000,
     GRIPPER_OPEN_MODE,
    },
    {{
         {0, 0.5},
         {0.7, 0.5},
         {0.7, 1.0},
         {0, 0.5},
         {-0.5f, 1.0f},
         {Pi, 0.5},
     },
     2000,
     GRIPPER_CLOSE_MODE
    },
    {{
         {0, 0.5},
         {0, 0.5},
         {0.7, 1.0},
         {0, 0.5},
         {0, 0.5},
         {0, 1.5},
     },
     1000,
     GRIPPER_CLOSE_MODE
    },
    {{
         {0, 0.5},
         {0, 1.0},
         {0.2, 1.0},
         {0, 0.5},
         {0, 0.5},
         {0, 1.0},
     },
     1000,
     GRIPPER_CLOSE_MODE,
    },
};
traj_group_point_t  traj_group_B [] =  {
  {
    {
      {}
    }
  }
}
TrajectoryExecutor traj_exec;

uint32_t test_seq;
extern "C" void auto_get_task(void *argument) {
  UNUSED(argument);
  traj_exec.init(traj_group_A, TRAJ_GROUP_SIZE, 5);
  traj_exec.build_time_acc();
  traj_exec.reset();
  while (true) {
    test_seq = traj_exec.get_seq();
    traj_exec.update(auto_traj_idx, Target_Point);
    osDelay(5);
  }
}
