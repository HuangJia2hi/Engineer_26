#include <auto.h>


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
     GRIPPER_OPEN_MODE},
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
     GRIPPER_CLOSE_MODE},
    {{
         {0, 0.5},
         {0, 0.5},
         {0.7, 1.0},
         {0, 0.5},
         {0, 0.5},
         {0, 1.5},
     },
     1000,
     GRIPPER_CLOSE_MODE},
    {
        {
            {0, 0.5},
            {0, 1.0},
            {0.2, 1.0},
            {0, 0.5},
            {0, 0.5},
            {0, 1.0},
        },
        2000,
        GRIPPER_CLOSE_MODE,
    },
};
traj_group_point_t traj_group_B[] = {

    {
        {
            {0, 0.5},
            {0, 1.0},
            {0.2, 1.0},
            {0, 0.5},
            {0, 0.5},
            {0, 1.0},
        },
        2000,
        GRIPPER_OPEN_MODE,
    },
    {
        {
            {0, 0.5},
            {0.7, 0.8},
            {1.0, 1.0},
            {0, 0.5},
            {0, 0.5},
            {Pi - 0.90, 0.8},
        },
        5000,
        GRIPPER_OPEN_MODE,
    },
    {
        {
            {0.6, 0.5},
            {0.7, 1.0},
            {0.8, 1.0},
            {0, 0.5},
            {0, 0.5},
            {Pi - 0.9, 1.0},
        },
        1500,
        GRIPPER_CLOSE_MODE,
    },
    {
        {
            {0, 1.0},
            {0, 1.0},
            {0.2, 1.0},
            {0, 0.5},
            {0, 0.5},
            {0, 1.5},
        },
        2000,
        GRIPPER_CLOSE_MODE,
    },
};
traj_group_point_t traj_group_C[] = {
    {
        {
            {0, 0},
            {0, 0},
            {1.2, 1.0},
            {0, 0},
            {0, 0},
            {0, 0},
        },
        2000,
        GRIPPER_OPEN_MODE,
    },
    {
        {
            {0, 0},
            {0, 0},
            {1.2, 1.0},
            {0.870796, 1.0},
            {0, 0},
            {0, 0},
        },
        2000,
        GRIPPER_OPEN_MODE,
    },

    {
        {
            {0, 0.5},
            {0.7, 0.6},
            {1.4, 1.0},
            {0.9, 1.0},
            {-0.2, 0.5},
            {0, 0},
        },
        5000,
        GRIPPER_OPEN_MODE,
    },
    {
        {
            {0.6, 0.8},
            {0.7, 0.6},
            {1.5, 1.0},
            // {0.870796,1.0},
            {0.9, 1.0},
            {-0.2, 0.5},
            {0, 0},
        },
        2000,
        GRIPPER_CLOSE_MODE,
    },
    {
        {
            {0, 0.8},
            {0, 0.6},
            {0.5, 1.0},
            {0, 1.0},
            {0, 0},
            {0, 0},
        },
        2000,
        GRIPPER_CLOSE_MODE,
    },
};

traj_group_point_t traj_group_D[] = {
    {
        {
            {0, 0},
            {1.0, 0.8},
            {1.65, 1.5},
            {0.9, 0.8},
            {-0.2, 0.5},
            {0, 0},
        },
        5000,
        GRIPPER_OPEN_MODE,
    },
    {
        {
            {0.6, 1.0},
            {1.0, 0.8},
            {1.9, 2.0},
            {0.9, 0.8},
            {-0.2, 0.5},
            {0, 0},
        },
        2000,
        GRIPPER_CLOSE_MODE,
    },

    {
        {
            {0, 0.6},
            {0, 0.8},
            {0.2, 1.5},
            {0, 0.8},
            {0, 0.5},
            {0, 0},
        },
        2000,
        GRIPPER_CLOSE_MODE,
    },
};
TrajectoryExecutor traj_exec;

uint32_t test_seq;
extern "C" void auto_get_task(void *argument) {
  UNUSED(argument);
  traj_exec.init(traj_group_D, TRAJ_GROUP_SIZE, 5);
  traj_exec.build_time_acc();
  traj_exec.reset();
  while (true) {
    test_seq = traj_exec.get_seq();
    traj_exec.update(auto_traj_idx, Target_Point);
    osDelay(5);
  }
}
