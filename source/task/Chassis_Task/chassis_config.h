#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* Chassis task timing */
#define Chassis_Task_Loop_Delay_MS 2U                         /* 底盘任务主循环延时，单位 ms */
#define Chassis_Task_Loop_Period_S 0.002f                    /* 底盘任务主循环周期，单位 s */

/* Chassis planar motion */
#define Max_Velocity 3.0f                                     /* 底盘平移控制的最大线速度 */
#define Chassis_Keyboard_Shift_Speed_Ratio 0.3f              /* 键盘按下 Shift 时的平移速度倍率 */
#define Chassis_Lateral_Forward_Compensation_Ratio 0.08f     /* 左右平移时补一点前向量，抵消底盘轻微后溜 */

/* Chassis yaw closed-loop input shaping */
#define Chassis_Yaw_Remoter_Deadzone 50                      /* 遥控器 ch3 改目标角时的输入死区 */
#define Chassis_Yaw_Remoter_TargetRate_Max 3.0f             /* 遥控器满量程时对应的目标 yaw 角速度，单位 rad/s */
#define Chassis_Yaw_Remoter_Polarity -1.0f                  /* 遥控器 ch3 改目标角时的方向极性 */
#define Chassis_Yaw_Mouse_Deadzone Chassis_Yaw_Remoter_Deadzone   /* 鼠标 X 改目标角时的输入死区，按 DBUS 同配置 */
#define Chassis_Yaw_Mouse_Input_Limit Remoter_CHMAX               /* 鼠标 X 参与目标角映射前的限幅范围，按 DBUS 同配置 */
#define Chassis_Yaw_Mouse_TargetRate_Max Chassis_Yaw_Remoter_TargetRate_Max /* 鼠标满量程时对应的目标 yaw 角速度，按 DBUS 同配置 */
#define Chassis_Yaw_Mouse_Polarity -1.0f                    /* 鼠标 X 改目标角时的方向极性 */
#define Chassis_Yaw_InputRate_Accel_Max 200.0f              /* yaw 输入角速度上升斜率上限，先大幅放开，排除输入斜坡导致的滞后 */
#define Chassis_Yaw_InputRate_Decel_Max 400.0f              /* yaw 输入角速度下降斜率上限，先大幅放开，排除输入斜坡导致的滞后 */
#define Chassis_Yaw_InputRate_Active_Threshold 0.08f        /* yaw 输入角速度超过该值时视为“主动持续旋转” */

/* Chassis yaw closed-loop PID */
#define Chassis_Yaw_IMU_Update_Period_S 0.001f              /* IMU yaw 速度链路目标更新周期，单位 s */
#define Chassis_Yaw_Angle_Deadzone 0.2f                    /* yaw 位置环软静区，沿用当前车上较稳的设置 */
#define Chassis_Yaw_Speed_Deadzone 0.11f                   /* yaw 速度环软静区，再放宽一点，继续压静止附近的小噪声抖动 */
#define Chassis_Yaw_Speed_Feedback_Max 8.0f                 /* yaw 角速度反馈限幅，单位 rad/s */
#define Chassis_Yaw_IMU_Speed_Polarity 1.0f                 /* IMU yaw 角速度反馈方向极性 */
#define Chassis_Yaw_InputRate_Feedforward_Gain 1.0f         /* 遥控器/鼠标给出的目标角速度前馈增益 */
#define Chassis_Yaw_Pos_PID_kp 2.2f                         /* yaw 位置环近端比例系数，控制收敛末端的平顺性 */
#define Chassis_Yaw_Pos_PID_ki 0.01f                        /* yaw 位置环积分系数 */
#define Chassis_Yaw_Pos_PID_kd 0.02f                        /* yaw 位置环微分系数 */
#define Chassis_Yaw_Pos_PID_Maxout 8.0f                     /* yaw 位置环输出目标角速度上限，单位 rad/s */
#define Chassis_Yaw_Pos_PID_Maxiout 1.0f                    /* yaw 位置环积分项上限 */
#define Chassis_Yaw_Pos_Fast_Error_Threshold 0.00f          /* yaw 位置误差超过该值后开启远端加速，单位 rad */
#define Chassis_Yaw_Pos_Fast_Extra_kp 0.0f                  /* yaw 远离目标时的额外比例系数，先关闭这条实验性加速链路 */
#define Chassis_Yaw_Pos_SpeedDamping_Gain 0.0f             /* yaw 收尾保持时的位置环速度阻尼增益，抑制回正时跑过头 */
#define Chassis_Yaw_Spd_PID_kp 1.5f                        /* yaw 速度环比例系数，主要提供阻尼 */
#define Chassis_Yaw_Spd_PID_ki 0.0f                         /* yaw 速度环积分系数 */
#define Chassis_Yaw_Spd_PID_kd 0.0f                        /* yaw 速度环微分系数，先关闭，避免速度误差差分项引入抖动和相位滞后 */
#define Chassis_Yaw_Spd_PID_Maxout 2.0f                     /* yaw 速度环输出修正量上限，单位 rad/s */
#define Chassis_Yaw_Spd_PID_Maxiout 0.5f                    /* yaw 速度环积分项上限 */
#define Chassis_Yaw_Wz_Output_Max 8.0f                      /* yaw 最终输出到底盘解算的角速度上限，单位 rad/s */
#define Chassis_Yaw_FrontWheel_Correction_Ratio 0.5f       /* 闭环 yaw 在前轮上的额外纠偏比例 */

/* Rising mechanism motion */
#define Max_Rising_Motor_Velocity 2.8f                      /* 抬升 3508 电机的最大目标速度 */
#define Max_Rising_DM_angle 0.9f                            /* 抬升 DM 电机允许的最大目标角 */
#define Rising_DM_ZeroPoint 0.05f                           /* 抬升 DM 电机零位参考角 */
#define Rising_DM_Velocity 2.1f                             /* 抬升 DM 电机速度给定 */
#define Rising_DM_ImuTarget_Blend_Start_Ratio 0.15f         /* 抬升角从零点到最大值的 20% 位置开始逐渐附加 IMU 额外目标 */
#define Rising_DM_ImuTarget_Blend_End_Ratio 0.30f           /* 抬升角到达零点到最大值的 60% 位置时，IMU 额外目标附加到最大 */
#define Rising_DM_ImuTarget_Fallback 0.18f                  /* IMU 额外目标的最大附加值 */

/* Chassis kinematics */
#define Track_R 0.05                                        /* 麦轮底盘角速度到轮速换算使用的等效半径 */
#define Steel_R 0.15                                        /* 轮子等效半径 */
#define Reduction_Ratio (3519.0f / 187.0f)                  /* 3508 到轮子的减速比 */
#define Motor_Wheel_Trans (6.677f * 0.0001f)                /* 电机转速反馈换算到底盘轮速的系数 */

/* Rising behavior defaults */
#define CHASSIS_RISING_BEHAVIOR_DEFAULT 0U                  /* 上电默认的抬升行为：0 一级，1 二级 */
#define CHASSIS_RISING_KEYBOARD_RC_CH2 200                  /* 键盘触发 rising 时，喂给抬升控制的等效 ch2 */

/* Single-lift timing */
#define CHASSIS_RISING_SINGLE_LIFT_DURATION_MS 1500U        /* 一级抬升阶段持续时间 */
#define CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_NUM 100  /* 一级抬升阶段底盘前进速度比例分子 */
#define CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_DEN 100 /* 一级抬升阶段底盘前进速度比例分母 */
#define CHASSIS_RISING_SINGLE_LIFT_RISING_RC_CH2 Remoter_CHMAX /* 一级抬升阶段抬升机构等效 ch2 */
#define CHASSIS_RISING_SINGLE_TRANSITION_DURATION_MS 0U     /* 一级抬升到前冲之间的停顿时间 */
#define CHASSIS_RISING_SINGLE_DRIVE_DURATION_MS 1400U        /* 一级抬升后前冲阶段持续时间 */
#define CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_NUM 70     /* 一级抬升后前冲速度比例分子 */
#define CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_DEN 100     /* 一级抬升后前冲速度比例分母 */

/* Double-lift timing */
#define CHASSIS_RISING_DOUBLE_LIFT_DURATION_MS 1600U        /* 二级抬升阶段持续时间 */
#define CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_NUM 100  /* 二级抬升阶段底盘前进速度比例分子 */
#define CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_DEN 100  /* 二级抬升阶段底盘前进速度比例分母 */
#define CHASSIS_RISING_DOUBLE_LIFT_RISING_RC_CH2 Remoter_CHMAX /* 二级抬升阶段抬升机构等效 ch2 */
#define CHASSIS_RISING_DOUBLE_TRANSITION_DURATION_MS 0U     /* 二级抬升到前冲之间的停顿时间 */
#define CHASSIS_RISING_DOUBLE_DRIVE_DURATION_MS 600U        /* 二级抬升后前冲阶段持续时间 */
#define CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_NUM 70      /* 二级抬升后前冲速度比例分子 */
#define CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_DEN 100     /* 二级抬升后前冲速度比例分母 */
#define CHASSIS_RISING_DOUBLE_RISING_HOLD_DURATION_MS 1800U /* 二级抬升完成后保持 rising 的持续时间 */
#define CHASSIS_RISING_DOUBLE_NORMAL_HOLD_DURATION_MS 600U  /* 二级抬升完成后恢复 normal 的保持时间 */

/* Chassis 3508 index mapping */
#define Chassis_Motor_3508_ZQ 0                             /* 底盘左前轮在数组中的索引 */
#define Chassis_Motor_3508_ZH 1                             /* 底盘左后轮在数组中的索引 */
#define Chassis_Motor_3508_YH 2                             /* 底盘右后轮在数组中的索引 */
#define Chassis_Motor_3508_YQ 3                             /* 底盘右前轮在数组中的索引 */

/* Rising 3508 index mapping */
#define Rising_Motor_3508_Left 0                            /* 抬升左 3508 在数组中的索引 */
#define Rising_Motor_3508_Right 1                           /* 抬升右 3508 在数组中的索引 */

/* Chassis CAN IDs */
#define Chassis_Motor_3508_ZQ_id 0x201                      /* 底盘左前轮 3508 的 CAN 反馈 ID */
#define Chassis_Motor_3508_ZH_id 0x202                      /* 底盘左后轮 3508 的 CAN 反馈 ID */
#define Chassis_Motor_3508_YH_id 0x203                      /* 底盘右后轮 3508 的 CAN 反馈 ID */
#define Chassis_Motor_3508_YQ_id 0x204                      /* 底盘右前轮 3508 的 CAN 反馈 ID */

/* Rising CAN IDs */
#define Rising_Motor_3508_Left_id 0x205                     /* 抬升左 3508 的 CAN 反馈 ID */
#define Rising_Motor_3508_Right_id 0x206                    /* 抬升右 3508 的 CAN 反馈 ID */

/* Chassis command CAN IDs */
#define Chassis_Motor_ALL_id 0x200                          /* 底盘四个 3508 的统一发送 ID */
#define Rising_Motor_ALL_id 0x1FF                           /* 抬升两个 3508 的统一发送 ID */

/* Chassis wheel speed PID */
#define Chassis_3508_PID_kp 10000                            /* 底盘轮速环比例系数 */
#define Chassis_3508_PID_ki 0.0f                            /* 底盘轮速环积分系数 */
#define Chassis_3508_PID_kd 0.0f                            /* 底盘轮速环微分系数 */
#define Chassis_3508_PID_Maxout 16384                       /* 底盘轮速环输出上限 */
#define Chassis_3508_PID_Maxiout 8192                       /* 底盘轮速环积分上限 */

/* Chassis power limit defaults */
#define Chassis_PowerLimit_Enable_Default 1U                 /* 上电默认开启底盘功率限制 */
#define Chassis_PowerLimit_UserMax_Default 120.0f           /* 用户侧配置的底盘功率上限默认值 */
#define Chassis_PowerModel_TorqueCoeff_Default 1.99688994e-6f /* 功率模型的力矩项系数默认值 */
#define Chassis_PowerModel_K1_Default 1.23e-07f            /* 功率模型 K1 默认值 */
#define Chassis_PowerModel_K2_Default 1.453e-07f           /* 功率模型 K2 默认值 */
#define Chassis_PowerModel_K3_Default 4.081f              /* 功率模型 K3 默认值 */
#define Chassis_PowerModel_GlobalScale_Default 1.7f        /* 功率模型总缩放默认值 */
#define Chassis_PowerScale_Attack_Default 1.0f             /* 功率缩放收紧时的滤波系数 */
#define Chassis_PowerScale_Release_Default 0.05f           /* 功率缩放放开时的滤波系数 */
#define Chassis_PowerLimit_SafetyRatio_Default 0.95f       /* 功率限制安全系数，给模型误差和瞬时峰值留余量 */
#define Chassis_PowerLimit_SafetyMargin_W_Default 5.0f     /* 在安全系数之外再额外预留的功率余量，单位 W */

/* Chassis power calc groups */
#define Chassis_PowerCalc_Group_Chassis 0U                  /* 底盘轮组功率估算分组索引 */
#define Chassis_PowerCalc_Group_Rising 1U                   /* 抬升轮组功率估算分组索引 */
#define Chassis_PowerCalc_Group_Count 2U                    /* 功率估算分组总数 */
#define Chassis_PowerCalc_Enable_Default 1U                 /* 各功率估算分组默认开启 */

/* Rising mode power allocation */
#define Chassis_Rising_PowerAlloc_Front_W 50.0f             /* Rising 模式下分配给底盘前轮组的目标功率 */
#define Chassis_Rising_PowerAlloc_Rear_W 50.0f              /* Rising 模式下分配给底盘后轮组的目标功率 */
#define Chassis_Rising_PowerAlloc_Tracks_W 50.0f            /* Rising 模式下分配给抬升 3508 轮组的目标功率 */

/* Rising 3508 speed PID */
#define Rising_3508_PID_kp 9000                             /* 抬升 3508 轮速环比例系数 */
#define Rising_3508_PID_ki 0.0001f                          /* 抬升 3508 轮速环积分系数 */
#define Rising_3508_PID_kd 0.0f                             /* 抬升 3508 轮速环微分系数 */
#define Rising_3508_PID_Maxout 16384                        /* 抬升 3508 轮速环输出上限 */
#define Rising_3508_PID_Maxiout 8192                        /* 抬升 3508 轮速环积分上限 */

/* Rising DM angle PID */
#define Rising_DM_PID_kp 1.2f                               /* 抬升 DM 角度环比例系数 */
#define Rising_DM_PID_ki 0.004f                             /* 抬升 DM 角度环积分系数 */
#define Rising_DM_PID_kd 0.02f                              /* 抬升 DM 角度环微分系数 */
#define Rising_DM_PID_Maxout 0.85f                          /* 抬升 DM 角度环输出上限 */
#define Rising_DM_PID_Maxiout 0.8f                          /* 抬升 DM 角度环积分上限 */

/* DM motor identifiers */
#define DM_l0010l_Master_ID_Left 0x13                       /* 左侧 DM 电机主控 ID */
#define DM_l0010l_Master_ID_Right 0x14                      /* 右侧 DM 电机主控 ID */
#define DM_l0010l_CAN_ID_Left 0x03                          /* 左侧 DM 电机从机 ID */
#define DM_l0010l_CAN_ID_Right 0x04                         /* 右侧 DM 电机从机 ID */

#endif /* CHASSIS_CONFIG_H */
