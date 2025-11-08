/**==================================================================
 * @file PID_controller.c
 * @brief PID控制器实现
 * @author ZHOUHENG-D
 * @date 2024-08-15
 * 
 * 实现FOC电机控制的PID控制器模块，包括：
 * - 电流环PI控制器（d轴、q轴）
 * - 速度环PI控制器
 * - 位置环PD控制器
 * - 抗积分饱和保护
 * - 电流环解耦
 * - 速度前馈控制
 * ==================================================================
 */

#include "PID_controller.h"

/* 全局PID控制器组定义 */
pid_controller_group_t pid_groups[motors_number];

/* ========================================================================
 * 私有函数声明
 * ======================================================================== */

/**
 * @brief 初始化单个PID控制器
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_limit 输出限幅
 * @param integral_limit 积分限幅
 */
static void PID_InitController(pid_controller_t *pid, float kp, float ki, float kd,
                             float output_limit, float integral_limit);

/**
 * @brief 应用抗积分饱和
 * @param pid PID控制器结构体指针
 * @param output 限幅前的输出
 * @param limited_output 限幅后的输出
 */
static void PID_ApplyAntiWindup(pid_controller_t *pid, float output, float limited_output);

/**
 * @brief 电流环解耦计算
 * @param decoupling 解耦参数结构体指针
 * @param id_feedback d轴电流反馈
 * @param iq_feedback q轴电流反馈
 * @param decoupling_d 输出d轴解耦电压
 * @param decoupling_q 输出q轴解耦电压
 */
static void PID_CalculateDecoupling(const current_decoupling_t *decoupling,
                                   float id_feedback, float iq_feedback,
                                   float *decoupling_d, float *decoupling_q);

/**
 * @brief 速度前馈计算
 * @param feed_forward 前馈参数结构体指针
 * @param omega 机械角速度
 * @param Ke 反电动势系数
 * @return 前馈电压
 */
static float PID_CalculateFeedForward(const speed_feed_forward_t *feed_forward,
                                     float omega, float Ke);

/* ========================================================================
 * PID控制器初始化
 * ======================================================================== */

void PID_Init(uint8_t motor_id)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    pid_controller_group_t *group = &pid_groups[motor_id];
    group->motor_id = motor_id;
    group->dt = 0.001f; // 默认1ms采样时间
    
    /* 初始化d轴电流环PI控制器 */
    PID_InitController(&group->current_d, 
                      PID_DEFAULT_KP_CURRENT_D, 
                      PID_DEFAULT_KI_CURRENT_D, 
                      PID_DEFAULT_KD_CURRENT_D,
                      PID_DEFAULT_OUTPUT_LIMIT_CURRENT,
                      PID_DEFAULT_INTEGRAL_LIMIT_CURRENT);
    
    /* 初始化q轴电流环PI控制器 */
    PID_InitController(&group->current_q, 
                      PID_DEFAULT_KP_CURRENT_Q, 
                      PID_DEFAULT_KI_CURRENT_Q, 
                      PID_DEFAULT_KD_CURRENT_Q,
                      PID_DEFAULT_OUTPUT_LIMIT_CURRENT,
                      PID_DEFAULT_INTEGRAL_LIMIT_CURRENT);
    
    /* 初始化速度环PI控制器 */
    PID_InitController(&group->speed, 
                      PID_DEFAULT_KP_SPEED, 
                      PID_DEFAULT_KI_SPEED, 
                      PID_DEFAULT_KD_SPEED,
                      PID_DEFAULT_OUTPUT_LIMIT_SPEED,
                      PID_DEFAULT_INTEGRAL_LIMIT_SPEED);
    
    /* 初始化位置环PD控制器 */
    PID_InitController(&group->position, 
                      PID_DEFAULT_KP_POSITION, 
                      PID_DEFAULT_KI_POSITION, 
                      PID_DEFAULT_KD_POSITION,
                      PID_DEFAULT_OUTPUT_LIMIT_POSITION,
                      PID_DEFAULT_INTEGRAL_LIMIT_POSITION);
    
    /* 初始化电流环解耦参数 */
    group->decoupling.ws = 0.0f;
    group->decoupling.Ld = 0.0f;
    group->decoupling.Lq = 0.0f;
    group->decoupling.Flux = 0.0f;
    group->decoupling.enable = false;
    
    /* 初始化速度前馈参数 */
    group->feed_forward.feed_forward_gain = PID_DEFAULT_SPEED_FEED_FORWARD_GAIN;
    group->feed_forward.omega_threshold = PID_DEFAULT_SPEED_FEED_FORWARD_THRESHOLD;
    group->feed_forward.enable = false;
}

static void PID_InitController(pid_controller_t *pid, float kp, float ki, float kd,
                             float output_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output = 0.0f;
    
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    
    pid->anti_windup_mode = ANTI_WINDUP_CLAMP;
    pid->back_calc_gain = PID_DEFAULT_BACK_CALC_GAIN;
    
    pid->derivative_filter_coeff = PID_DEFAULT_DERIVATIVE_FILTER_COEFF;
    pid->enable = true;
    pid->derivative_on_measurement = false; // 默认微分作用在误差上
}

/* ========================================================================
 * 核心PID控制算法
 * ======================================================================== */

float PID_Calculate(pid_controller_t *pid, float setpoint, float feedback, float dt)
{
    if (!pid->enable || dt <= 0.0f) {
        return 0.0f;
    }
    
    /* 计算误差 */
    float error = setpoint - feedback;
    
    /* 比例项 */
    float proportional = pid->kp * error;
    
    /* 积分项 */
    float integral = pid->integral;
    if (pid->ki != 0.0f) {
        integral += pid->ki * error * dt;
        
        /* 积分限幅 */
        if (integral > pid->integral_limit) {
            integral = pid->integral_limit;
        } else if (integral < -pid->integral_limit) {
            integral = -pid->integral_limit;
        }
    }
    
    /* 微分项 */
    float derivative = 0.0f;
    if (pid->kd != 0.0f) {
        if (pid->derivative_on_measurement) {
            /* 微分作用在测量值上，避免设定值突变引起的冲击 */
            derivative = -pid->kd * (feedback - pid->prev_measurement) / dt;
            pid->prev_measurement = feedback;
        } else {
            /* 微分作用在误差上 */
            derivative = pid->kd * (error - pid->prev_error) / dt;
        }
        pid->prev_error = error;
        
        /* 微分滤波 */
        if (pid->derivative_filter_coeff > 0.0f) {
            derivative = pid->prev_derivative * (1.0f - pid->derivative_filter_coeff) + 
                        derivative * pid->derivative_filter_coeff;
            pid->prev_derivative = derivative;
        }
    }
    
    /* 计算总输出 */
    float output = proportional + integral + derivative;
    
    /* 输出限幅 */
    float limited_output = output;
    if (limited_output > pid->output_limit) {
        limited_output = pid->output_limit;
    } else if (limited_output < -pid->output_limit) {
        limited_output = -pid->output_limit;
    }
    
    /* 抗积分饱和处理 */
    PID_ApplyAntiWindup(pid, output, limited_output);
    
    /* 更新积分项和输出 */
    pid->integral = integral;
    pid->output = limited_output;
    
    return limited_output;
}

static void PID_ApplyAntiWindup(pid_controller_t *pid, float output, float limited_output)
{
    switch (pid->anti_windup_mode) {
        case ANTI_WINDUP_NONE:
            /* 无抗积分饱和处理 */
            break;
            
        case ANTI_WINDUP_CLAMP:
            /* 积分限幅已在计算中处理 */
            break;
            
        case ANTI_WINDUP_BACK_CALC:
            /* 反向计算抗积分饱和 */
            if (pid->ki != 0.0f) {
                float tracking_error = limited_output - output;
                pid->integral += pid->back_calc_gain * tracking_error;
            }
            break;
    }
}

/* ========================================================================
 * 电流环控制函数
 * ======================================================================== */

float PID_CurrentD_Calculate(uint8_t motor_id, float id_ref, float id_feedback, float iq_feedback)
{
    if (motor_id >= motors_number) {
        return 0.0f;
    }
    
    pid_controller_group_t *group = &pid_groups[motor_id];
    pid_controller_t *pid = &group->current_d;
    
    /* PI控制器计算 */
    float pid_output = PID_Calculate(pid, id_ref, id_feedback, group->dt);
    
    /* 电流环解耦 */
    float decoupling_voltage = 0.0f;
    if (group->decoupling.enable) {
        float decoupling_d, decoupling_q;
        PID_CalculateDecoupling(&group->decoupling, id_feedback, iq_feedback, 
                               &decoupling_d, &decoupling_q);
        decoupling_voltage = decoupling_d;
    }
    
    /* 总输出 = PI输出 + 解耦电压 */
    return pid_output + decoupling_voltage;
}

float PID_CurrentQ_Calculate(uint8_t motor_id, float iq_ref, float iq_feedback, float id_feedback)
{
    if (motor_id >= motors_number) {
        return 0.0f;
    }
    
    pid_controller_group_t *group = &pid_groups[motor_id];
    pid_controller_t *pid = &group->current_q;
    
    /* PI控制器计算 */
    float pid_output = PID_Calculate(pid, iq_ref, iq_feedback, group->dt);
    
    /* 电流环解耦 */
    float decoupling_voltage = 0.0f;
    if (group->decoupling.enable) {
        float decoupling_d, decoupling_q;
        PID_CalculateDecoupling(&group->decoupling, id_feedback, iq_feedback, 
                               &decoupling_d, &decoupling_q);
        decoupling_voltage = decoupling_q;
    }
    
    /* 总输出 = PI输出 + 解耦电压 */
    return pid_output + decoupling_voltage;
}

static void PID_CalculateDecoupling(const current_decoupling_t *decoupling,
                                   float id_feedback, float iq_feedback,
                                   float *decoupling_d, float *decoupling_q)
{
    /* d轴解耦电压: -ws * Lq * iq_feedback */
    *decoupling_d = -decoupling->ws * decoupling->Lq * iq_feedback;
    
    /* q轴解耦电压: ws * (Ld * id_feedback + Flux) */
    *decoupling_q = decoupling->ws * (decoupling->Ld * id_feedback + decoupling->Flux);
}

/* ========================================================================
 * 速度环控制函数
 * ======================================================================== */

float PID_Speed_Calculate(uint8_t motor_id, float speed_ref, float speed_feedback)
{
    if (motor_id >= motors_number) {
        return 0.0f;
    }
    
    pid_controller_group_t *group = &pid_groups[motor_id];
    pid_controller_t *pid = &group->speed;
    
    /* PI控制器计算 */
    float pid_output = PID_Calculate(pid, speed_ref, speed_feedback, group->dt);
    
    /* 速度前馈 */
    float feed_forward_voltage = 0.0f;
    if (group->feed_forward.enable) {
        /* 获取电机参数 */
        const Motor_Params_t *motor = &motor_params[motor_id];
        
        /* 计算前馈电压 V_ff = Ke * ω */
        float omega_mechanical = speed_ref; // 假设speed_ref已经是机械角速度
        if (fabsf(omega_mechanical) > group->feed_forward.omega_threshold) {
            feed_forward_voltage = group->feed_forward.feed_forward_gain * motor->Ke * omega_mechanical;
        }
    }
    
    /* 总输出 = PI输出 + 前馈电压 */
    return pid_output + feed_forward_voltage;
}

/* ========================================================================
 * 位置环控制函数
 * ======================================================================== */

float PID_Position_Calculate(uint8_t motor_id, float position_ref, float position_feedback)
{
    if (motor_id >= motors_number) {
        return 0.0f;
    }
    
    pid_controller_group_t *group = &pid_groups[motor_id];
    pid_controller_t *pid = &group->position;
    
    /* PD控制器计算 */
    return PID_Calculate(pid, position_ref, position_feedback, group->dt);
}

/* ========================================================================
 * 参数设置函数
 * ======================================================================== */

void PID_SetParameters(uint8_t motor_id, pid_controller_type_t type, 
                      float kp, float ki, float kd)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    pid_controller_t *pid = NULL;
    pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return;
    }
    
    if (pid != NULL) {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
    }
}

void PID_SetOutputLimit(uint8_t motor_id, pid_controller_type_t type, float limit)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    pid_controller_t *pid = NULL;
    pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return;
    }
    
    if (pid != NULL) {
        pid->output_limit = limit;
    }
}

void PID_SetIntegralLimit(uint8_t motor_id, pid_controller_type_t type, float limit)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    pid_controller_t *pid = NULL;
    pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return;
    }
    
    if (pid != NULL) {
        pid->integral_limit = limit;
    }
}

void PID_SetAntiWindup(uint8_t motor_id, pid_controller_type_t type, 
                      anti_windup_mode_t mode, float back_calc_gain)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    pid_controller_t *pid = NULL;
    pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return;
    }
    
    if (pid != NULL) {
        pid->anti_windup_mode = mode;
        pid->back_calc_gain = back_calc_gain;
    }
}

void PID_SetSampleTime(uint8_t motor_id, float dt)
{
    if (motor_id >= motors_number && dt > 0.0f) {
        pid_groups[motor_id].dt = dt;
    }
}

/* ========================================================================
 * 解耦和前馈设置函数
 * ======================================================================== */

void PID_SetDecoupling(uint8_t motor_id, float ws, float Ld, float Lq, float Flux)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    current_decoupling_t *decoupling = &pid_groups[motor_id].decoupling;
    decoupling->ws = ws;
    decoupling->Ld = Ld;
    decoupling->Lq = Lq;
    decoupling->Flux = Flux;
}

void PID_EnableDecoupling(uint8_t motor_id, bool enable)
{
    if (motor_id >= motors_number) {
        pid_groups[motor_id].decoupling.enable = enable;
    }
}

void PID_SetSpeedFeedForward(uint8_t motor_id, float feed_forward_gain, float omega_threshold)
{
    if (motor_id >= motors_number) {
        speed_feed_forward_t *feed_forward = &pid_groups[motor_id].feed_forward;
        feed_forward->feed_forward_gain = feed_forward_gain;
        feed_forward->omega_threshold = omega_threshold;
    }
}

void PID_EnableSpeedFeedForward(uint8_t motor_id, bool enable)
{
    if (motor_id >= motors_number) {
        pid_groups[motor_id].feed_forward.enable = enable;
    }
}

/* ========================================================================
 * 控制器管理函数
 * ======================================================================== */

void PID_ResetController(uint8_t motor_id, pid_controller_type_t type)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    pid_controller_t *pid = NULL;
    pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return;
    }
    
    if (pid != NULL) {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        pid->prev_measurement = 0.0f;
        pid->prev_derivative = 0.0f;
    }
}

void PID_EnableController(uint8_t motor_id, pid_controller_type_t type, bool enable)
{
    if (motor_id >= motors_number) {
        return;
    }
    
    pid_controller_t *pid = NULL;
    pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return;
    }
    
    if (pid != NULL) {
        pid->enable = enable;
    }
}

float PID_GetOutput(uint8_t motor_id, pid_controller_type_t type)
{
    if (motor_id >= motors_number) {
        return 0.0f;
    }
    
    const pid_controller_t *pid = NULL;
    const pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return 0.0f;
    }
    
    return (pid != NULL) ? pid->output : 0.0f;
}

void PID_GetParameters(uint8_t motor_id, pid_controller_type_t type, 
                      float *kp, float *ki, float *kd)
{
    if (motor_id >= motors_number || kp == NULL || ki == NULL || kd == NULL) {
        return;
    }
    
    const pid_controller_t *pid = NULL;
    const pid_controller_group_t *group = &pid_groups[motor_id];
    
    switch (type) {
        case PID_TYPE_CURRENT_D:
            pid = &group->current_d;
            break;
        case PID_TYPE_CURRENT_Q:
            pid = &group->current_q;
            break;
        case PID_TYPE_SPEED:
            pid = &group->speed;
            break;
        case PID_TYPE_POSITION:
            pid = &group->position;
            break;
        default:
            return;
    }
    
    if (pid != NULL) {
        *kp = pid->kp;
        *ki = pid->ki;
        *kd = pid->kd;
    }
}