#include "auto.h"
#include "auto_traj_data.h"
#include "auto_keyboard.h"

huangjiazhi::TrajectoryExecutor traj_exec;
static volatile auto_key_cmd_t cmd_req = CMD_NONE;
static auto_key_cmd_t last_cmd = CMD_NONE;
void Auto_Switch_Group(huangjiazhi::traj_group_point_t *group, uint32_t size)
{
    traj_exec.init(group, size, 5);
    traj_exec.build_time_acc();
    traj_exec.reset();
    auto_traj_idx = 0;
    osTimerStart(auto_traj_timer_id, 5);
}

extern "C" void auto_key_cmd_exec(auto_key_cmd_t cmd){
    cmd_req = cmd;
}

extern "C" void auto_get_task(void *argument) {
  UNUSED(argument);
  traj_exec.init(traj_group_auto_get_A, traj_group_auto_get_A_size, 5);
  traj_exec.build_time_acc();
  traj_exec.reset();
  while (true) {
    if (cmd_req != CMD_NONE && cmd_req != last_cmd) {

        switch (cmd_req) {

            case CMD_AUTO_GET_A_POS:
                Auto_Switch_Group(traj_group_auto_get_A,
                                  traj_group_auto_get_A_size);
                break;

            case CMD_AUTO_GET_A_SET:
                Auto_Switch_Group(traj_group_auto_get_B,
                                  traj_group_auto_get_B_size);
                break;

            default:
                break;
        }

        last_cmd = cmd_req;
        cmd_req = CMD_NONE; 
    }
    traj_exec.update(auto_traj_idx, Target_Point);

    if (traj_exec.is_finished()) {
        osTimerStop(auto_traj_timer_id);
        auto_traj_idx = 0;
    }

    osDelay(5);
  }
}
