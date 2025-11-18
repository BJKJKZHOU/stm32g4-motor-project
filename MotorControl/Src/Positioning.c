/*============================================================================
    File Name     : Positioning.c
    Description   :  
    Author        : ZHOUHENG
    Date          : 2025-11-12
    ----------------------------------------------------------------------       
    
*=============================================================================
*/


#include "Positioning.h"
#include "motor_params.h"
#include "normalization.h"

#include <math.h>      // 用于 fabsf, atan2f, sqrtf 函数
#include <stdint.h>
#include <stdlib.h>    // 用于 rand() 函数
#include <string.h>


const float IPD_PULSE_ANGLES[12] = {0, 180, 30, 210, 60, 240, 90, 270, 120, 300, 150, 330};

#define NONLINEAR_OBS_ERROR_RATIO        (0.01f)
#define NONLINEAR_OBS_ERROR_DELTA_RATIO  (0.001f)
#define NONLINEAR_OBS_CONV_WINDOW        (50U)
#define NONLINEAR_OBS_FILTER_ALPHA       (0.2f)

static bool NonlinearObs_Position_UpdateBases(NonlinearObs_Position_t *obs)
{
    const normalization_base_values_t *bases = Normalization_GetBases(obs->motor_id);
    if (bases != NULL) {
        obs->base_values = *bases;
        obs->base_valid = true;
    }
    return obs->base_valid;
}


// 验证输入数据是否匹配预定义的脉冲角度序列
static bool IPD_ValidatePulseSequence(IPD_Pulse_t pulses[12])
{
    for (int i = 0; i < 12; i++) {
        // 检查角度是否匹配预定义的脉冲角度（允许0.1度误差）
        float expected_angle = IPD_PULSE_ANGLES[i];
        float actual_angle = pulses[i].angle_elec;
        
        if (fabsf(actual_angle - expected_angle) > 0.1f) {
            return false;  // 角度不匹配
        }
        
        // 检查电流值是否有效
        if (!isfinite(pulses[i].current_sample)) {
            return false;  // 电流值无效
        }
    }
    return true;
}

float IPD_CalculateRotorPosition(IPD_Pulse_t pulses[12])
{
    // 验证输入数据
    if (!IPD_ValidatePulseSequence(pulses)) {
        return -999.0f;  // 数据验证失败
    }

    float max_current_pair[6] = {0};  // 每对脉冲的最大电流值
    float rotor_angle_candidates[6] = {0}; // 每对脉冲对应的角度候选
    
    // 处理6对对角脉冲，确定每对的N极方向
    for (int pair = 0; pair < 6; pair++) {
        int pos_idx = pair * 2;      // 正角度脉冲索引
        int neg_idx = pair * 2 + 1;  // 负角度脉冲索引
        
        float current_pos = pulses[pos_idx].current_sample;
        float current_neg = pulses[neg_idx].current_sample;
        
        // 电流响应峰值大的为N极方向
        if (current_pos > current_neg) {
            max_current_pair[pair] = current_pos;
            rotor_angle_candidates[pair] = pulses[pos_idx].angle_elec;
        } else {
            max_current_pair[pair] = current_neg;
            rotor_angle_candidates[pair] = pulses[neg_idx].angle_elec;
        }
    }
    
    // 在6个候选角度中，找到电流响应峰值最大的为最终转子位置
    float max_current = 0;
    float final_rotor_angle = 0;
    
    for (int i = 0; i < 6; i++) {
        if (max_current_pair[i] > max_current) {
            max_current = max_current_pair[i];
            final_rotor_angle = rotor_angle_candidates[i];
        }
    }
    
    // 检查结果是否在合理范围内
    if (final_rotor_angle < 0 || final_rotor_angle >= 360) {
        // 如果结果异常，返回错误值
        return -999.0f;
    }
    
    return final_rotor_angle;
}


// 执行完整的12脉冲定位序列
bool IPD_ExecutePulseSequence(IPD_Pulse_t pulses[12])
{
    // 临时电流采样值
    float current_sample = 0.0f;
    
    // 按照预定义的12个角度顺序发送脉冲
    for (int i = 0; i < 12; i++) {
        // 设置当前脉冲的角度
        pulses[i].angle_elec = IPD_PULSE_ANGLES[i];
        
        // 发送电压脉冲到电机的代码
        
        
        
        // 注意：实际项目中应使用硬件定时器或延时函数
        for (volatile int j = 0; j < 10000; j++);
        
        // 采样电机电流的代码
        
        
        // 这里用随机值模拟电流采样（实际应从硬件获取）
        current_sample = (float)(rand() % 1000) / 1000.0f * 5.0f; // 模拟0-5A的电流
        
        // 存储电流采样值
        pulses[i].current_sample = current_sample;
        
        // 脉冲之间的间隔时间，确保电流衰减
        // 实际项目中应根据电机电气参数和母线电压调整
        for (volatile int j = 0; j < 20000; j++);
    }
    
    // 验证生成的脉冲序列是否有效
    if (!IPD_ValidatePulseSequence(pulses)) {
        return false; // 序列验证失败
    }
    
    return true; // 12脉冲序列执行成功
}


/*============================================================================
 * 非线性观测器实现 - 基于论文 "Sensorless Control of Surface-Mount
 * Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer"
 *===========================================================================*/

/**
 * @brief 初始化非线性位置观测器
 * @param obs 观测器结构体指针
 * @param motor_id 电机ID
 * @param gamma 观测器增益 (> 0)
 */
void NonlinearObs_Position_Init(NonlinearObs_Position_t *obs, uint8_t motor_id, float gamma)
{
    /* 参数有效性检查 */
    if (obs == NULL || motor_id >= motors_number) {
        return;
    }
    
    obs->motor_id = motor_id;
    obs->gamma = gamma;
    obs->base_valid = false;
    memset(&obs->base_values, 0, sizeof(obs->base_values));
    if (!NonlinearObs_Position_UpdateBases(obs)) {
        Normalization_UpdateMotor(motor_id);
        NonlinearObs_Position_UpdateBases(obs);
    }
    
    /* 重置所有状态变量 */
    NonlinearObs_Position_Reset(obs);
}

/**
 * @brief 重置非线性位置观测器状态
 * @param obs 观测器结构体指针
 */
void NonlinearObs_Position_Reset(NonlinearObs_Position_t *obs)
{
    /* 参数有效性检查 */
    if (obs == NULL) {
        return;
    }
    
    obs->x_hat_alpha_pu = 0.0f;
    obs->x_hat_beta_pu = 0.0f;
    obs->theta_hat_rad = 0.0f;
    obs->theta_hat_deg = 0.0f;
    obs->is_initialized = true;
    obs->is_converged = false;
    obs->eta_norm_sq_filtered = 0.0f;
    obs->eta_norm_sq_last = 0.0f;
    obs->stable_counter = 0;
    obs->stable_required = NONLINEAR_OBS_CONV_WINDOW;
    obs->error_metrics_valid = false;
}

/**
 * @brief 更新非线性位置观测器
 * @param obs 观测器结构体指针
 * @param i_alpha_pu α轴电流 (标幺值)
 * @param i_beta_pu β轴电流 (标幺值)
 * @param v_alpha_pu α轴电压 (标幺值)
 * @param v_beta_pu β轴电压 (标幺值)
 * @param dt 采样时间 (秒)
 */
void NonlinearObs_Position_Update(NonlinearObs_Position_t *obs, 
                                  float i_alpha_pu, float i_beta_pu, 
                                  float v_alpha_pu, float v_beta_pu, 
                                  float dt)
{
    if (obs == NULL || !obs->is_initialized || dt <= 0.0f) {
        return;
    }
    
    // 检查输入参数有效性
    if (!isfinite(i_alpha_pu) || !isfinite(i_beta_pu) || 
        !isfinite(v_alpha_pu) || !isfinite(v_beta_pu)) {
        return;
    }
    
    // 获取电机参数
    const Motor_Params_t *params = &motor_params[obs->motor_id];
    
    if (params == NULL || !NonlinearObs_Position_UpdateBases(obs)) {
        return;
    }
    
    const normalization_base_values_t *bases = &obs->base_values;
    if (bases->time_base <= 0.0f) {
        return;
    }
    
    const float dt_pu = dt / bases->time_base;
    
    // 计算归一化的电机参数
    // 表面贴装电机: Ld = Lq = L
    float L_n = params->Ld / bases->inductance_base;  // 归一化电感
    float Rs_n = params->Rs / bases->impedance_base;  // 归一化电阻
    float psi_m_n = params->Flux / bases->flux_base;  // 归一化磁链
    
    // 步骤3: 计算可测量变量 y = -Rs * i_αβ + v_αβ
    float y_alpha_pu = -Rs_n * i_alpha_pu + v_alpha_pu;
    float y_beta_pu = -Rs_n * i_beta_pu + v_beta_pu;
    
    // 步骤4: 计算误差项 η(hat{x}) = hat{x} - L * i_αβ
    float eta_alpha_pu = obs->x_hat_alpha_pu - L_n * i_alpha_pu;
    float eta_beta_pu = obs->x_hat_beta_pu - L_n * i_beta_pu;
    
    // 计算误差项的范数平方 ||η(hat{x})||^2
    float eta_norm_sq_pu = eta_alpha_pu * eta_alpha_pu + eta_beta_pu * eta_beta_pu;
    
    // 暂存上一次的滤波误差值（用于后续收敛判定）
    float eta_norm_sq_last_saved = obs->eta_norm_sq_last;

    // 更新误差统计（使用更新前的误差，用于观测器算法）
    if (!obs->error_metrics_valid || !isfinite(obs->eta_norm_sq_filtered)) {
        obs->eta_norm_sq_filtered = eta_norm_sq_pu;
        obs->eta_norm_sq_last = eta_norm_sq_pu;
        obs->error_metrics_valid = true;
    } else {
        obs->eta_norm_sq_filtered += NONLINEAR_OBS_FILTER_ALPHA *
            (eta_norm_sq_pu - obs->eta_norm_sq_filtered);
    }
    
    // 计算误差驱动项 [ψ_m^2 - ||η(hat{x})||^2]
    float error_term_pu = psi_m_n * psi_m_n - eta_norm_sq_pu;
    
    // 步骤4: 更新观测器状态
    // d\hat{x}/dt = y + (γ/2) * η(\hat{x}) * [ψ_m^2 - ||η(\hat{x})||^2]
    float dx_hat_alpha_dt = y_alpha_pu + (obs->gamma / 2.0f) * eta_alpha_pu * error_term_pu;
    float dx_hat_beta_dt = y_beta_pu + (obs->gamma / 2.0f) * eta_beta_pu * error_term_pu;
    
    // 欧拉积分更新状态估计
    obs->x_hat_alpha_pu += dx_hat_alpha_dt * dt_pu;
    obs->x_hat_beta_pu += dx_hat_beta_dt * dt_pu;

    // 步骤5: 提取位置估计
    // [cos\hat{θ}; sin\hat{θ}] = (1/ψ_m) * (\hat{x} - L * i_αβ)

    // 除零保护：检查归一化磁链是否有效
    if (fabsf(psi_m_n) < 1e-10f) {
        // 磁链过小，保持之前的位置估计
        return;
    }

    float cos_theta_hat = (obs->x_hat_alpha_pu - L_n * i_alpha_pu) / psi_m_n;
    float sin_theta_hat = (obs->x_hat_beta_pu - L_n * i_beta_pu) / psi_m_n;

    // 对向量整体归一化，维持在单位圆上
    float vec_norm_sq = cos_theta_hat * cos_theta_hat + sin_theta_hat * sin_theta_hat;
    if (vec_norm_sq < 1e-12f || !isfinite(vec_norm_sq)) {
        return;
    }
    float inv_norm = 1.0f / sqrtf(vec_norm_sq);
    cos_theta_hat *= inv_norm;
    sin_theta_hat *= inv_norm;

    // 使用 atan2 计算位置估计 (弧度)
    obs->theta_hat_rad = atan2f(sin_theta_hat, cos_theta_hat);

    // 转换为角度 (0-360度范围)
    obs->theta_hat_deg = obs->theta_hat_rad * 180.0f / PI;
    if (obs->theta_hat_deg < 0.0f) {
        obs->theta_hat_deg += 360.0f;
    }

    // 重新计算更新后的误差（用于收敛判定）
    // 注意：这里使用更新后的 x_hat 值
    float eta_alpha_updated = obs->x_hat_alpha_pu - L_n * i_alpha_pu;
    float eta_beta_updated = obs->x_hat_beta_pu - L_n * i_beta_pu;
    float eta_norm_sq_updated = eta_alpha_updated * eta_alpha_updated +
                                eta_beta_updated * eta_beta_updated;

    // 应用滤波器到更新后的误差
    float eta_norm_sq_filtered_updated = obs->eta_norm_sq_filtered +
        NONLINEAR_OBS_FILTER_ALPHA * (eta_norm_sq_updated - obs->eta_norm_sq_filtered);

    // 计算误差变化率（使用更新后的滤波误差）
    float eta_delta = fabsf(eta_norm_sq_filtered_updated - eta_norm_sq_last_saved);

    // 更新存储的滤波误差值
    obs->eta_norm_sq_last = eta_norm_sq_filtered_updated;

    // 检查收敛性：误差幅值 + 变化率 + 时间窗口
    float magnitude_threshold = NONLINEAR_OBS_ERROR_RATIO * psi_m_n * psi_m_n;
    float delta_threshold = NONLINEAR_OBS_ERROR_DELTA_RATIO * psi_m_n * psi_m_n;

    bool magnitude_ok = eta_norm_sq_filtered_updated < magnitude_threshold;
    bool delta_ok = eta_delta < delta_threshold;

    if (magnitude_ok && delta_ok) {
        if (obs->stable_counter < obs->stable_required) {
            obs->stable_counter++;
        }
    } else {
        obs->stable_counter = 0;
    }

    obs->is_converged = (obs->stable_counter >= obs->stable_required);
}

/**
 * @brief 获取位置估计 (弧度)
 * @param obs 观测器结构体指针
 * @return 转子位置估计 (电角度, 弧度)
 */
float NonlinearObs_Position_GetThetaRad(NonlinearObs_Position_t *obs)
{
    if (obs == NULL || !obs->is_initialized) {
        return 0.0f;
    }
    return obs->theta_hat_rad;
}

/**
 * @brief 获取位置估计 (度)
 * @param obs 观测器结构体指针
 * @return 转子位置估计 (电角度, 度)
 */
float NonlinearObs_Position_GetThetaDeg(NonlinearObs_Position_t *obs)
{
    if (obs == NULL || !obs->is_initialized) {
        return 0.0f;
    }
    return obs->theta_hat_deg;
}
