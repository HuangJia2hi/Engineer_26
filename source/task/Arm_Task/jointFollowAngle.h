#ifndef JOINTFOLLOWANGLE
#define JOINTFOLLOWANGLE

#include "arm_state_machine.h"
#include "DBusSys.h"

extern endEffector_t EndEffector;
extern Joint_t Joint[JOINT_NUM];
/** @brief 任务jointFollowAngle */
void jointFollowAngle(void *argument);

void target_point_init(target_point_t *Target_Point);
#endif
