/**==================================================================
 * @file PID_controller.c
 * @brief PID/PDFF统一控制器实现
 * @author ZHOUHENG-D
 * @date 2025-11-08
 *
 * 统一的PID/PDFF控制器，通过Kfr_speed参数自动切换模式
 * ==================================================================*/

#include "PID_controller.h"
#include <stddef.h>
#include <math.h>

float PID_Controller(float setpoint, float feedback, float dt,
                    PID_Params_t *params, PID_State_t *state)
{
    /* 参数有效性检查 */
    if (params == NULL || state == NULL || dt <= 0.0f) {
        return 0.0f;
    }
    
    float output = 0.0f;

    if (params->Kfr_speed == 0.0f) {

        float error = setpoint - feedback;
        
        /* 比例项 */
        float proportional = params->kp * error;
        
        /* 积分项 */
        float integral = state->integral;
        if (params->ki != 0.0f) {
            integral += params->ki * error * dt;
            
            /* 积分限幅 - 修复：使用输出限幅作为积分限幅上限 */
            float effective_integral_limit = fminf(params->integral_limit, params->output_limit);
            if (integral > effective_integral_limit) {
                integral = effective_integral_limit;
            } else if (integral < -effective_integral_limit) {
                integral = -effective_integral_limit;
            }
        } else {
            // 修复：当ki=0时清零积分项，避免之前的累积值继续影响输出
            integral = 0.0f;
        }
        
        /* 微分项*/
        float derivative = 0.0f;
        if (params->kd != 0.0f) {
            derivative = params->kd * (error - state->prev_error) / dt;
        }
        
        /* 计算总输出 */
        output = proportional + integral + derivative;
        
        /* 更新状态 */
        state->integral = integral;
        state->prev_error = error;
        
    } else {
        /* ====== PDFF算法 ====== */
        /* output = (setpoint * Kfr_speed) + ((setpoint - feedback) * ki/积分算子) - (feedback * kp) */
        
        /* 前馈项 */
        float feedforward = setpoint * params->Kfr_speed;
        
        /* 积分项 (error * ki * dt) */
        float error = setpoint - feedback;
        float integral = state->integral;
        if (params->ki != 0.0f) {
            integral += params->ki * error * dt;
            
            /* 积分限幅 - 修复：使用输出限幅作为积分限幅上限 */
            float effective_integral_limit = fminf(params->integral_limit, params->output_limit);
            if (integral > effective_integral_limit) {
                integral = effective_integral_limit;
            } else if (integral < -effective_integral_limit) {
                integral = -effective_integral_limit;
            }
        } else {
            // 修复：当ki=0时清零积分项，避免之前的累积值继续影响输出
            integral = 0.0f;
        }
        
        /* 反馈项（注意：这里kp作用在feedback上，符号为负）*/
        float feedback_term = -(params->kp * feedback);
        
        /* 计算总输出 */
        output = feedforward + integral + feedback_term;
        
        /* 更新状态 */
        state->integral = integral;
        state->prev_error = error;  // 保存误差以备后用
    }
    
    /* 输出限幅 */
    if (output > params->output_limit) {
        output = params->output_limit;
        // 修复：抗积分饱和 - 当输出饱和时，停止积分器累积
        if (params->ki != 0.0f) {
            // 限制积分值，确保不会导致输出进一步饱和
            float effective_integral_limit = fminf(params->integral_limit, params->output_limit);
            state->integral = fminf(state->integral, effective_integral_limit);
        }
    } else if (output < -params->output_limit) {
        output = -params->output_limit;
        // 修复：抗积分饱和 - 当输出饱和时，停止积分器累积
        if (params->ki != 0.0f) {
            // 限制积分值，确保不会导致输出进一步饱和
            float effective_integral_limit = fminf(params->integral_limit, params->output_limit);
            state->integral = fmaxf(state->integral, -effective_integral_limit);
        }
    }
    
    return output;
}