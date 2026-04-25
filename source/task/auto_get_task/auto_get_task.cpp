#include "auto.h"
#include "auto_traj_data.h"


huangjiazhi::TrajectoryExecutor traj_exec;

uint32_t test_seq;
extern "C" void auto_get_task(void *argument) {
  UNUSED(argument);
  traj_exec.init(traj_group_D, huangjiazhi::TRAJ_GROUP_SIZE, 5);
  traj_exec.build_time_acc();
  traj_exec.reset();
  while (true) {
    test_seq = traj_exec.get_seq();
    traj_exec.update(auto_traj_idx, Target_Point);
    osDelay(5);
  }
}
