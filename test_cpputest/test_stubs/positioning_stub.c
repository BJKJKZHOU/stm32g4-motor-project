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
