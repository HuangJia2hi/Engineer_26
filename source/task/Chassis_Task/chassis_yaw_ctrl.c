#include "chassis_yaw_ctrl.h"

#include "DBusSys.h"
#include "IMU_Task.h"
#include "PIDtool.h"
#include "chassis_config.h"
#include "chassis_debug.h"
#include "tool.h"

#include <math.h>

static pid_type_def s_chassis_yaw_pos_pid;
static pid_type_def s_chassis_yaw_spd_pid;
static float32_t s_chassis_target_yaw_angle = 0.0f;
static float32_t s_chassis_current_yaw_angle = 0.0f;
static float32_t s_chassis_input_yaw_rate = 0.0f;
static float32_t s_chassis_target_yaw_speed = 0.0f;
static float32_t s_chassis_current_yaw_speed = 0.0f;
static float32_t s_chassis_last_raw_yaw_angle = 0.0f;
static uint8_t s_chassis_yaw_initialized = 0U;

static float32_t Chassis_YawCtrl_ApplySoftDeadzone(float32_t value, float32_t deadzone)
{
    if (deadzone <= 0.0f) {
        return value;
    }

    if (value > deadzone) {
        return value - deadzone;
    }

    if (value < -deadzone) {
        return value + deadzone;
    }

    return 0.0f;
}

static float32_t Chassis_YawCtrl_ApplySlewRate(float32_t current_value,
                                               float32_t target_value,
                                               float32_t accel_limit,
                                               float32_t decel_limit)
{
    float32_t delta = target_value - current_value;
    float32_t max_delta = accel_limit * Chassis_Task_Loop_Period_S;

    if (fabsf(target_value) < fabsf(current_value)) {
        max_delta = decel_limit * Chassis_Task_Loop_Period_S;
    }

    delta = limit(delta, -max_delta, max_delta);
    return current_value + delta;
}

static void Chassis_YawCtrl_ResetPosPid(void)
{
    PID_Init(&s_chassis_yaw_pos_pid,
             Chassis_Yaw_Pos_PID_kp,
             Chassis_Yaw_Pos_PID_ki,
             Chassis_Yaw_Pos_PID_kd,
             Chassis_Yaw_Pos_PID_Maxout,
             Chassis_Yaw_Pos_PID_Maxiout);
}

static void Chassis_YawCtrl_ResetSpdPid(void)
{
    PID_Init(&s_chassis_yaw_spd_pid,
             Chassis_Yaw_Spd_PID_kp,
             Chassis_Yaw_Spd_PID_ki,
             Chassis_Yaw_Spd_PID_kd,
             Chassis_Yaw_Spd_PID_Maxout,
             Chassis_Yaw_Spd_PID_Maxiout);
}

static float32_t Chassis_YawCtrl_NormalizeDelta(float32_t delta_angle)
{
    return angle_normalize(delta_angle, 2.0f * (float32_t)Pi);
}

static uint8_t Chassis_YawCtrl_UpdateCurrentAngle(void)
{
    const float32_t raw_yaw_angle = IMU_data.Yaw;

    if (IMU_data_time <= 0) {
        g_chassis_debug.chassis_yaw_imu_ready = 0U;
        return 0U;
    }

    if (s_chassis_yaw_initialized == 0U) {
        s_chassis_last_raw_yaw_angle = raw_yaw_angle;
        s_chassis_current_yaw_angle = raw_yaw_angle;
        s_chassis_target_yaw_angle = raw_yaw_angle;
        s_chassis_input_yaw_rate = 0.0f;
        s_chassis_target_yaw_speed = 0.0f;
        s_chassis_yaw_initialized = 1U;
    } else {
        s_chassis_current_yaw_angle +=
            Chassis_YawCtrl_NormalizeDelta(raw_yaw_angle - s_chassis_last_raw_yaw_angle);
        s_chassis_last_raw_yaw_angle = raw_yaw_angle;
    }

    s_chassis_current_yaw_speed =
        limit(Chassis_Yaw_IMU_Speed_Polarity * IMU_data.YawSpeed,
              -Chassis_Yaw_Speed_Feedback_Max,
              Chassis_Yaw_Speed_Feedback_Max);

    g_chassis_debug.chassis_current_yaw_angle = s_chassis_current_yaw_angle;
    g_chassis_debug.chassis_target_yaw_angle = s_chassis_target_yaw_angle;
    g_chassis_debug.chassis_target_yaw_speed = s_chassis_target_yaw_speed;
    g_chassis_debug.chassis_current_yaw_speed = s_chassis_current_yaw_speed;
    g_chassis_debug.chassis_yaw_imu_ready = 1U;
    return 1U;
}

static float32_t Chassis_YawCtrl_MapInputToTargetRate(float32_t input_value,
                                                      float32_t deadzone,
                                                      float32_t input_limit,
                                                      float32_t polarity,
                                                      float32_t max_target_rate)
{
    if (fabsf(input_value) <= deadzone) {
        return 0.0f;
    }

    input_value = limit(input_value, -input_limit, input_limit);
    return polarity * map(input_value,
                          -input_limit,
                          input_limit,
                          -max_target_rate,
                          max_target_rate);
}

void Chassis_YawCtrl_Init(void)
{
    Chassis_YawCtrl_ResetPosPid();
    Chassis_YawCtrl_ResetSpdPid();
    s_chassis_target_yaw_angle = 0.0f;
    s_chassis_current_yaw_angle = 0.0f;
    s_chassis_input_yaw_rate = 0.0f;
    s_chassis_target_yaw_speed = 0.0f;
    s_chassis_current_yaw_speed = 0.0f;
    s_chassis_last_raw_yaw_angle = 0.0f;
    s_chassis_yaw_initialized = 0U;

    g_chassis_debug.chassis_target_yaw_angle = 0.0f;
    g_chassis_debug.chassis_current_yaw_angle = 0.0f;
    g_chassis_debug.chassis_yaw_angle_error = 0.0f;
    g_chassis_debug.chassis_input_yaw_rate = 0.0f;
    g_chassis_debug.chassis_target_yaw_speed = 0.0f;
    g_chassis_debug.chassis_current_yaw_speed = 0.0f;
    g_chassis_debug.chassis_yaw_output_wz = 0.0f;
    g_chassis_debug.chassis_yaw_imu_ready = 0U;

    Chassis_YawCtrl_HoldCurrentAngle();
}

void Chassis_YawCtrl_HoldCurrentAngle(void)
{
    (void)Chassis_YawCtrl_UpdateCurrentAngle();

    s_chassis_target_yaw_angle = s_chassis_current_yaw_angle;
    s_chassis_input_yaw_rate = 0.0f;
    s_chassis_target_yaw_speed = 0.0f;
    s_chassis_current_yaw_speed =
        limit(Chassis_Yaw_IMU_Speed_Polarity * IMU_data.YawSpeed,
              -Chassis_Yaw_Speed_Feedback_Max,
              Chassis_Yaw_Speed_Feedback_Max);
    Chassis_YawCtrl_ResetPosPid();
    Chassis_YawCtrl_ResetSpdPid();

    g_chassis_debug.chassis_target_yaw_angle = s_chassis_target_yaw_angle;
    g_chassis_debug.chassis_current_yaw_angle = s_chassis_current_yaw_angle;
    g_chassis_debug.chassis_yaw_angle_error = 0.0f;
    g_chassis_debug.chassis_input_yaw_rate = 0.0f;
    g_chassis_debug.chassis_target_yaw_speed = 0.0f;
    g_chassis_debug.chassis_current_yaw_speed = s_chassis_current_yaw_speed;
    g_chassis_debug.chassis_yaw_output_wz = 0.0f;
}

void Chassis_YawCtrl_UpdateTargetFromDbus(int16_t ch3)
{
    float32_t raw_target_rate = 0.0f;

    if (Chassis_YawCtrl_UpdateCurrentAngle() != 0U) {
        raw_target_rate = Chassis_YawCtrl_MapInputToTargetRate((float32_t)ch3,
                                                               (float32_t)Chassis_Yaw_Remoter_Deadzone,
                                                               (float32_t)Remoter_CHMAX,
                                                               Chassis_Yaw_Remoter_Polarity,
                                                               Chassis_Yaw_Remoter_TargetRate_Max);
        s_chassis_input_yaw_rate =
            Chassis_YawCtrl_ApplySlewRate(s_chassis_input_yaw_rate,
                                          raw_target_rate,
                                          Chassis_Yaw_InputRate_Accel_Max,
                                          Chassis_Yaw_InputRate_Decel_Max);
        s_chassis_target_yaw_angle += s_chassis_input_yaw_rate * Chassis_Task_Loop_Period_S;
    } else {
        s_chassis_input_yaw_rate = 0.0f;
    }

    g_chassis_debug.chassis_target_yaw_angle = s_chassis_target_yaw_angle;
    g_chassis_debug.chassis_current_yaw_angle = s_chassis_current_yaw_angle;
    g_chassis_debug.chassis_input_yaw_rate = s_chassis_input_yaw_rate;
}

void Chassis_YawCtrl_UpdateTargetFromMouse(int16_t mouse_x, uint8_t enable_input)
{
    float32_t raw_target_rate = 0.0f;

    if (Chassis_YawCtrl_UpdateCurrentAngle() != 0U) {
        if (enable_input != 0U) {
            raw_target_rate = Chassis_YawCtrl_MapInputToTargetRate((float32_t)mouse_x,
                                                                   (float32_t)Chassis_Yaw_Mouse_Deadzone,
                                                                   (float32_t)Chassis_Yaw_Mouse_Input_Limit,
                                                                   Chassis_Yaw_Mouse_Polarity,
                                                                   Chassis_Yaw_Mouse_TargetRate_Max);
        }

        s_chassis_input_yaw_rate =
            Chassis_YawCtrl_ApplySlewRate(s_chassis_input_yaw_rate,
                                          raw_target_rate,
                                          Chassis_Yaw_InputRate_Accel_Max,
                                          Chassis_Yaw_InputRate_Decel_Max);
        s_chassis_target_yaw_angle += s_chassis_input_yaw_rate * Chassis_Task_Loop_Period_S;
    } else {
        s_chassis_input_yaw_rate = 0.0f;
    }

    g_chassis_debug.chassis_target_yaw_angle = s_chassis_target_yaw_angle;
    g_chassis_debug.chassis_current_yaw_angle = s_chassis_current_yaw_angle;
    g_chassis_debug.chassis_input_yaw_rate = s_chassis_input_yaw_rate;
}

float32_t Chassis_YawCtrl_GetClosedLoopWz(void)
{
    float32_t abs_yaw_error_for_pid = 0.0f;
    float32_t input_rate_active = 0.0f;
    float32_t yaw_error = 0.0f;
    float32_t yaw_error_for_pid = 0.0f;
    float32_t yaw_pos_correction = 0.0f;
    float32_t yaw_speed_error = 0.0f;
    float32_t yaw_speed_error_for_pid = 0.0f;
    float32_t yaw_speed_correction = 0.0f;
    float32_t wz_output = 0.0f;

    if (Chassis_YawCtrl_UpdateCurrentAngle() == 0U) {
        g_chassis_debug.chassis_yaw_angle_error = 0.0f;
        g_chassis_debug.chassis_input_yaw_rate = 0.0f;
        g_chassis_debug.chassis_target_yaw_speed = 0.0f;
        g_chassis_debug.chassis_current_yaw_speed = 0.0f;
        g_chassis_debug.chassis_yaw_output_wz = 0.0f;
        return 0.0f;
    }

    yaw_error = s_chassis_target_yaw_angle - s_chassis_current_yaw_angle;
    yaw_error_for_pid =
        Chassis_YawCtrl_ApplySoftDeadzone(yaw_error, Chassis_Yaw_Angle_Deadzone);
    abs_yaw_error_for_pid = fabsf(yaw_error_for_pid);
    input_rate_active = fabsf(s_chassis_input_yaw_rate);
    if (yaw_error_for_pid != 0.0f) {
        yaw_pos_correction = PID_Calc_Pos(&s_chassis_yaw_pos_pid,
                                          0.0f,
                                          yaw_error_for_pid);

        if ((input_rate_active < Chassis_Yaw_InputRate_Active_Threshold) &&
            (abs_yaw_error_for_pid > Chassis_Yaw_Pos_Fast_Error_Threshold)) {
            yaw_pos_correction +=
                copysignf((abs_yaw_error_for_pid - Chassis_Yaw_Pos_Fast_Error_Threshold) *
                              Chassis_Yaw_Pos_Fast_Extra_kp,
                          yaw_error_for_pid);
        }

        if (input_rate_active < Chassis_Yaw_InputRate_Active_Threshold) {
            yaw_pos_correction -=
                Chassis_Yaw_Pos_SpeedDamping_Gain * s_chassis_current_yaw_speed;
        }

        yaw_pos_correction =
            limit(yaw_pos_correction,
                  -Chassis_Yaw_Pos_PID_Maxout,
                  Chassis_Yaw_Pos_PID_Maxout);
    } else {
        yaw_pos_correction = 0.0f;
        Chassis_YawCtrl_ResetPosPid();
    }

    /* 持续旋转时，输入本身就代表“期望角速度”。
     * 如果只积分成目标角，再靠位置误差慢慢逼出速度，实际转速就会出现“先快、再慢、误差积大后又突然快”的呼吸感。
     * 这里把输入角速度直接作为前馈，位置环只负责补偿跟随误差。
     */
    s_chassis_target_yaw_speed =
        Chassis_Yaw_InputRate_Feedforward_Gain * s_chassis_input_yaw_rate + yaw_pos_correction;
    s_chassis_target_yaw_speed =
        limit(s_chassis_target_yaw_speed,
              -Chassis_Yaw_Pos_PID_Maxout,
              Chassis_Yaw_Pos_PID_Maxout);

    yaw_speed_error = s_chassis_target_yaw_speed - s_chassis_current_yaw_speed;
    yaw_speed_error_for_pid =
        Chassis_YawCtrl_ApplySoftDeadzone(yaw_speed_error, Chassis_Yaw_Speed_Deadzone);
    if (yaw_speed_error_for_pid != 0.0f) {
        yaw_speed_correction = PID_Calc_Pos(&s_chassis_yaw_spd_pid,
                                            0.0f,
                                            yaw_speed_error_for_pid);
    } else {
        yaw_speed_correction = 0.0f;
        Chassis_YawCtrl_ResetSpdPid();
    }

    /* 速度环必须在目标角速度为 0 时也继续工作，此时它提供的是 -current_yaw_speed 的阻尼。
     * 如果在 0 附近把它关掉，受到一点扰动后就只剩“弹簧项”没有“阻尼项”，最容易来回摆。
     */
    wz_output = s_chassis_target_yaw_speed + yaw_speed_correction;
    wz_output = limit(wz_output, -Chassis_Yaw_Wz_Output_Max, Chassis_Yaw_Wz_Output_Max);

    g_chassis_debug.chassis_target_yaw_angle = s_chassis_target_yaw_angle;
    g_chassis_debug.chassis_current_yaw_angle = s_chassis_current_yaw_angle;
    g_chassis_debug.chassis_yaw_angle_error = yaw_error;
    g_chassis_debug.chassis_target_yaw_speed = s_chassis_target_yaw_speed;
    g_chassis_debug.chassis_current_yaw_speed = s_chassis_current_yaw_speed;
    g_chassis_debug.chassis_yaw_output_wz = wz_output;

    return wz_output;
}

float32_t Chassis_YawCtrl_GetCurrentAngle(void)
{
    (void)Chassis_YawCtrl_UpdateCurrentAngle();
    return s_chassis_current_yaw_angle;
}

float32_t Chassis_YawCtrl_GetTargetAngle(void)
{
    return s_chassis_target_yaw_angle;
}
