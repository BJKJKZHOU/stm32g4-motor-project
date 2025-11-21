/*============================================================================
    File Name     : positioning_stub.c
    Description   : 位置观测器的测试桩（简化实现）
    Author        : ZHOUHENG
    Date          : 2025-11-20
*=============================================================================*/

#include "Positioning.h"
#include <string.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief 非线性观测器初始化
 */
void NonlinearObs_Position_Init(NonlinearObs_Position_t *obs, uint8_t motor_id, float gamma)
{
    if (obs == NULL) {
        return;
    }

    memset(obs, 0, sizeof(NonlinearObs_Position_t));

    obs->motor_id = motor_id;
    obs->gamma = gamma;
    obs->is_initialized = true;
    obs->is_converged = false;

    // 初始化角度为0
    obs->theta_hat_rad = 0.0f;
    obs->theta_hat_deg = 0.0f;
}

/**
 * @brief 非线性观测器复位
 */
void NonlinearObs_Position_Reset(NonlinearObs_Position_t *obs)
{
    if (obs == NULL || !obs->is_initialized) {
        return;
    }

    obs->x_hat_alpha_pu = 0.0f;
    obs->x_hat_beta_pu = 0.0f;
    obs->theta_hat_rad = 0.0f;
    obs->theta_hat_deg = 0.0f;
    obs->is_converged = false;
}

/**
 * @brief 非线性观测器更新（简化实现）
 * @note 这是一个简化的桩函数，实际观测器算法更复杂
 */
void NonlinearObs_Position_Update(NonlinearObs_Position_t *obs,
                                  float i_alpha_pu, float i_beta_pu,
                                  float v_alpha_pu, float v_beta_pu,
                                  float dt)
{
    if (obs == NULL || !obs->is_initialized) {
        return;
    }

    // 简化实现：使用电流矢量角度作为位置估计
    // 实际的非线性观测器算法要复杂得多
    if (fabsf(i_alpha_pu) > 1e-6f || fabsf(i_beta_pu) > 1e-6f) {
        obs->theta_hat_rad = atan2f(i_beta_pu, i_alpha_pu);
        obs->theta_hat_deg = obs->theta_hat_rad * 180.0f / PI;
    }

    // 简化的状态估计
    obs->x_hat_alpha_pu = i_alpha_pu;
    obs->x_hat_beta_pu = i_beta_pu;

    // 标记为已收敛（简化处理）
    obs->is_converged = true;

    // 忽略电压输入（简化处理）
    (void)v_alpha_pu;
    (void)v_beta_pu;
    (void)dt;
}

/**
 * @brief 获取位置估计结果（弧度）
 */
float NonlinearObs_Position_GetThetaRad(NonlinearObs_Position_t *obs)
{
    if (obs == NULL || !obs->is_initialized) {
        return 0.0f;
    }

    return obs->theta_hat_rad;
}

/**
 * @brief 获取位置估计结果（度）
 */
float NonlinearObs_Position_GetThetaDeg(NonlinearObs_Position_t *obs)
{
    if (obs == NULL || !obs->is_initialized) {
        return 0.0f;
    }

    return obs->theta_hat_deg;
}

/**
 * @brief IPD检测转子位置（桩函数）
 */
bool IPD_DetectRotorPosition(const IPD_Config_t *config, float *angle_deg)
{
    if (config == NULL || angle_deg == NULL) {
        return false;
    }

    // 桩函数：返回固定角度
    *angle_deg = 0.0f;
    return true;
}

/**
 * @brief IPD执行脉冲序列（桩函数）
 */
bool IPD_ExecutePulseSequence(IPD_Pulse_t pulses[12], const IPD_Config_t *config)
{
    if (pulses == NULL || config == NULL) {
        return false;
    }

    // 桩函数：填充模拟数据
    for (int i = 0; i < 12; i++) {
        pulses[i].angle_elec = i * 30.0f;
        pulses[i].current_sample = 0.1f;
    }

    return true;
}

/**
 * @brief IPD计算转子位置（桩函数）
 */
float IPD_CalculateRotorPosition(IPD_Pulse_t pulses[12])
{
    if (pulses == NULL) {
        return -999.0f;
    }

    // 桩函数：返回固定角度
    return 0.0f;
}

/**
 * @brief IPD检测转子位置（带详细数据输出）（桩函数）
 */
bool IPD_DetectRotorPositionEx(const IPD_Config_t *config, float *angle_deg, IPD_Pulse_t pulses[12])
{
    if (config == NULL || angle_deg == NULL) {
        return false;
    }

    *angle_deg = 0.0f;

    if (pulses != NULL) {
        for (int i = 0; i < 12; i++) {
            pulses[i].angle_elec = i * 30.0f;
            pulses[i].current_sample = 0.1f;
        }
    }

    return true;
}

//=============================================================================
// PLL 速度观测器实现（从 Positioning.c 复制）
//=============================================================================

#include "motor_params.h"
#include <stdbool.h>

/**
 * @brief 角度归一化到 [-π, π]
 */
static inline float PLL_NormalizeAngle(float angle_rad)
{
    // 使用 fmodf 将角度限制在 [-2π, 2π] 范围内
    angle_rad = fmodf(angle_rad, 2.0f * PI);

    // 将角度映射到 [-π, π]
    if (angle_rad > PI) {
        angle_rad -= 2.0f * PI;
    } else if (angle_rad < -PI) {
        angle_rad += 2.0f * PI;
    }

    return angle_rad;
}

/**
 * @brief PLL速度观测器初始化
 */
//void PLL_SpeedObserver_Init(PLL_SpeedObserver_t *pll,
                            uint8_t motor_id,
                            float kp,
                            float ki,
                            float initial_theta_rad)
{
    if (pll == NULL) {
        return;
    }

    // 参数检查
    if (kp < 0.0f || ki < 0.0f) {
        return;  // 增益必须为正
    }

    // 保存配置参数
    pll->motor_id = motor_id;
    pll->kp = kp;
    pll->ki = ki;

    // 初始化状态变量
    pll->theta_hat_rad = PLL_NormalizeAngle(initial_theta_rad);
    pll->omega_hat_rad_s = 0.0f;  // 初始速度为0

    // 初始化输出变量
    pll->theta_hat_deg = pll->theta_hat_rad * 180.0f / PI;
    if (pll->theta_hat_deg < 0.0f) {
        pll->theta_hat_deg += 360.0f;
    }
    pll->speed_elec_rpm = 0.0f;
    pll->speed_mech_rpm = 0.0f;

    // 设置初始化标志
    pll->is_initialized = true;
}

/**
 * @brief PLL速度观测器更新
 */
void PLL_SpeedObserver_Update(PLL_SpeedObserver_t *pll,
                              float theta_measured_rad,
                              float dt)
{
    if (pll == NULL || !pll->is_initialized) {
        return;
    }

    // 参数检查
    if (dt <= 0.0f || dt > 0.01f) {
        return;  // 采样时间应在合理范围内（0-10ms）
    }

    // 检查输入角度有效性
    if (!isfinite(theta_measured_rad)) {
        return;
    }

    // 步骤1: 计算位置误差 Δθ = θ_measured - θ_hat
    float delta_theta = theta_measured_rad - pll->theta_hat_rad;

    // 步骤2: 归一化误差到 [-π, π]（处理角度跳变）
    delta_theta = PLL_NormalizeAngle(delta_theta);

    // 步骤3: 速度更新（积分环节）
    // ω_hat += Ki * Δθ * dt
    pll->omega_hat_rad_s += pll->ki * delta_theta * dt;

    // 步骤4: 位置更新（比例+积分）
    // θ_hat += (ω_hat + Kp * Δθ) * dt
    pll->theta_hat_rad += (pll->omega_hat_rad_s + pll->kp * delta_theta) * dt;

    // 步骤5: 归一化位置到 [-π, π]
    pll->theta_hat_rad = PLL_NormalizeAngle(pll->theta_hat_rad);

    // 步骤6: 计算输出变量

    // 6.1 角度（度），范围 [0, 360)
    pll->theta_hat_deg = pll->theta_hat_rad * 180.0f / PI;
    if (pll->theta_hat_deg < 0.0f) {
        pll->theta_hat_deg += 360.0f;
    }

    // 6.2 电转速（RPM）
    // RPM = (rad/s) * (60 / 2π)
    pll->speed_elec_rpm = pll->omega_hat_rad_s * 60.0f / (2.0f * PI);

    // 6.3 机械转速（RPM）= 电转速 / 极对数
    // 从电机参数模块获取极对数
    float pole_pairs = motor_params[pll->motor_id].Pn;
    if (pole_pairs > 0.0f) {
        pll->speed_mech_rpm = pll->speed_elec_rpm / pole_pairs;
    } else {
        pll->speed_mech_rpm = 0.0f;  // 极对数无效，返回0
    }

    // 检查输出有效性（防止NaN传播）
    if (!isfinite(pll->omega_hat_rad_s)) {
        pll->omega_hat_rad_s = 0.0f;
    }
    if (!isfinite(pll->speed_elec_rpm)) {
        pll->speed_elec_rpm = 0.0f;
    }
    if (!isfinite(pll->speed_mech_rpm)) {
        pll->speed_mech_rpm = 0.0f;
    }
}

/**
 * @brief 重置PLL速度观测器
 */
void PLL_SpeedObserver_Reset(PLL_SpeedObserver_t *pll, float new_theta_rad)
{
    if (pll == NULL || !pll->is_initialized) {
        return;
    }

    // 重置状态变量
    pll->theta_hat_rad = PLL_NormalizeAngle(new_theta_rad);
    pll->omega_hat_rad_s = 0.0f;

    // 重置输出变量
    pll->theta_hat_deg = pll->theta_hat_rad * 180.0f / PI;
    if (pll->theta_hat_deg < 0.0f) {
        pll->theta_hat_deg += 360.0f;
    }
    pll->speed_elec_rpm = 0.0f;
    pll->speed_mech_rpm = 0.0f;
}
