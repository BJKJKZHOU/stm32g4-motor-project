/*============================================================================
    File Name     : pid_controller_stub.c
    Description   : PID控制器的测试桩（简化实现）
    Author        : ZHOUHENG
    Date          : 2025-11-20
*=============================================================================*/

#include "PID_controller.h"
#include <math.h>

/**
 * @brief PID控制器（简化实现）
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @param dt 控制周期
 * @param params PID参数
 * @param state PID状态
 * @return 控制器输出
 */
float PID_Controller(float setpoint, float feedback, float dt,
                    PID_Params_t *params, PID_State_t *state)
{
    if (params == NULL || state == NULL) {
        return 0.0f;
    }

    // 计算误差
    float error = setpoint - feedback;

    // 比例项
    float proportional = params->kp * error;

    // 积分项
    state->integral += params->ki * error * dt;

    // 积分限幅
    if (state->integral > params->integral_limit) {
        state->integral = params->integral_limit;
    } else if (state->integral < -params->integral_limit) {
        state->integral = -params->integral_limit;
    }

    // 微分项（电流环通常不使用）
    float derivative = 0.0f;
    if (params->kd > 0.0f) {
        derivative = params->kd * (error - state->prev_error) / dt;
    }

    // 计算总输出
    float output = proportional + state->integral + derivative;

    // 输出限幅
    if (output > params->output_limit) {
        output = params->output_limit;
    } else if (output < -params->output_limit) {
        output = -params->output_limit;
    }

    // 更新上次误差
    state->prev_error = error;

    return output;
}
