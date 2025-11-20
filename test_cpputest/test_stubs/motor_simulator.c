/*============================================================================
    File Name     : motor_simulator.c
    Description   : PMSM电机模拟器实现
    Author        : ZHOUHENG
    Date          : 2025-11-20
    ----------------------------------------------------------------------
    实现说明：
    1. 使用RK4（四阶龙格库塔法）进行数值积分，精度高
    2. 电压方程在dq坐标系求解，然后转换为abc坐标系输出
    3. ADC模拟器模拟真实的采样过程，包括量化、偏移、噪声
*=============================================================================*/

#include "motor_simulator.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define TWO_PI (2.0f * PI)
#define SQRT3 1.732050807568877f
#define INV_SQRT3 0.577350269189626f

/* 全局变量定义 --------------------------------------------------------------*/
MotorSimParams_t g_motor_sim_params;
MotorSimState_t g_motor_sim_state;
ADCSimulator_t g_adc_simulator;

/* 内部辅助函数 --------------------------------------------------------------*/

/**
 * @brief 角度归一化到 [-π, π]
 */
static float normalize_angle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

/**
 * @brief 饱和限制
 */
static float saturate(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/**
 * @brief Park变换：αβ → dq
 */
static void park_transform(float alpha, float beta, float sin_theta, float cos_theta,
                          float *d, float *q) {
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

/**
 * @brief 逆Park变换：dq → αβ
 */
static void inverse_park_transform(float d, float q, float sin_theta, float cos_theta,
                                   float *alpha, float *beta) {
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

/**
 * @brief 逆Clarke变换：αβ → abc
 */
static void inverse_clarke_transform(float alpha, float beta,
                                     float *a, float *b, float *c) {
    *a = alpha;
    *b = -0.5f * alpha + SQRT3 * 0.5f * beta;
    *c = -0.5f * alpha - SQRT3 * 0.5f * beta;
}

/**
 * @brief 电机微分方程（状态空间方程）
 * @param state 当前状态 [Id, Iq, omega_e, theta_e]
 * @param deriv 输出导数 [dId/dt, dIq/dt, domega_e/dt, dtheta_e/dt]
 * @param Ud d轴电压 (V)
 * @param Uq q轴电压 (V)
 */
static void motor_dynamics(const float state[4], float deriv[4],
                          float Ud, float Uq,
                          const MotorSimParams_t *params) {
    float Id = state[0];
    float Iq = state[1];
    float omega_e = state[2];
    // float theta_e = state[3];  // 未使用

    // 电压方程
    // dId/dt = (Ud - Rs·Id + ω·Lq·Iq) / Ld
    deriv[0] = (Ud - params->Rs * Id + omega_e * params->Lq * Iq) / params->Ld;

    // dIq/dt = (Uq - Rs·Iq - ω·Ld·Id - ω·λ) / Lq
    deriv[1] = (Uq - params->Rs * Iq - omega_e * params->Ld * Id
                - omega_e * params->flux_linkage) / params->Lq;

    // 电磁转矩
    // Te = 1.5 * Pn * (λ·Iq + (Ld-Lq)·Id·Iq)
    float Te = 1.5f * params->pole_pairs *
               (params->flux_linkage * Iq + (params->Ld - params->Lq) * Id * Iq);

    // 机械方程
    // dω/dt = (Te - TL - B·ω) / J
    float omega_m = omega_e / params->pole_pairs;
    deriv[2] = (Te - params->T_load - params->B * omega_m) / params->J * params->pole_pairs;

    // dθ/dt = ω
    deriv[3] = omega_e;
}

/**
 * @brief RK4数值积分
 */
static void rk4_step(float state[4], float Ud, float Uq,
                    const MotorSimParams_t *params, float dt) {
    float k1[4], k2[4], k3[4], k4[4];
    float temp_state[4];
    int i;

    // k1 = f(state)
    motor_dynamics(state, k1, Ud, Uq, params);

    // k2 = f(state + 0.5*dt*k1)
    for (i = 0; i < 4; i++) {
        temp_state[i] = state[i] + 0.5f * dt * k1[i];
    }
    motor_dynamics(temp_state, k2, Ud, Uq, params);

    // k3 = f(state + 0.5*dt*k2)
    for (i = 0; i < 4; i++) {
        temp_state[i] = state[i] + 0.5f * dt * k2[i];
    }
    motor_dynamics(temp_state, k3, Ud, Uq, params);

    // k4 = f(state + dt*k3)
    for (i = 0; i < 4; i++) {
        temp_state[i] = state[i] + dt * k3[i];
    }
    motor_dynamics(temp_state, k4, Ud, Uq, params);

    // state = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    for (i = 0; i < 4; i++) {
        state[i] += (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
    }

    // 角度归一化
    state[3] = normalize_angle(state[3]);
}

/* 电机模拟器函数实现 --------------------------------------------------------*/

void MotorSim_Init(const MotorSimParams_t *params) {
    if (params != NULL) {
        memcpy(&g_motor_sim_params, params, sizeof(MotorSimParams_t));
    }
    MotorSim_Reset();
}

void MotorSim_SetDefaultParams(void) {
    // 基于项目中的典型PMSM参数
    g_motor_sim_params.Rs = 0.5f;              // 定子电阻 0.5Ω
    g_motor_sim_params.Ld = 0.0005f;           // d轴电感 0.5mH
    g_motor_sim_params.Lq = 0.0005f;           // q轴电感 0.5mH（表贴式电机 Ld≈Lq）
    g_motor_sim_params.flux_linkage = 0.01f;   // 磁链 0.01Wb
    g_motor_sim_params.pole_pairs = 4.0f;      // 4对极

    g_motor_sim_params.J = 0.00001f;           // 转动惯量 0.01 kg·m²×10⁻³
    g_motor_sim_params.B = 0.00001f;           // 摩擦系数

    g_motor_sim_params.V_dc = 24.0f;           // 24V母线电压
    g_motor_sim_params.T_load = 0.0f;          // 无负载

    g_motor_sim_params.Ts = 50e-6f;            // 50μs采样周期（20kHz）
    g_motor_sim_params.I_base = 10.0f;         // 10A基准电流
}

void MotorSim_Reset(void) {
    memset(&g_motor_sim_state, 0, sizeof(MotorSimState_t));
}

void MotorSim_Step_AlphaBeta(float U_alpha_pu, float U_beta_pu) {
    // 标幺值转换为物理值
    float U_alpha = U_alpha_pu * g_motor_sim_params.V_dc;
    float U_beta = U_beta_pu * g_motor_sim_params.V_dc;

    // Park变换：αβ → dq
    float sin_theta = sinf(g_motor_sim_state.theta_e);
    float cos_theta = cosf(g_motor_sim_state.theta_e);
    float Ud, Uq;
    park_transform(U_alpha, U_beta, sin_theta, cos_theta, &Ud, &Uq);

    // 调用dq坐标系步进
    MotorSim_Step_DQ(Ud / g_motor_sim_params.V_dc, Uq / g_motor_sim_params.V_dc);
}

void MotorSim_Step_DQ(float U_d_pu, float U_q_pu) {
    // 标幺值转换为物理值
    float Ud = U_d_pu * g_motor_sim_params.V_dc;
    float Uq = U_q_pu * g_motor_sim_params.V_dc;

    // 状态向量 [Id, Iq, omega_e, theta_e]
    float state[4] = {
        g_motor_sim_state.Id,
        g_motor_sim_state.Iq,
        g_motor_sim_state.omega_e,
        g_motor_sim_state.theta_e
    };

    // RK4积分
    rk4_step(state, Ud, Uq, &g_motor_sim_params, g_motor_sim_params.Ts);

    // 更新状态
    g_motor_sim_state.Id = state[0];
    g_motor_sim_state.Iq = state[1];
    g_motor_sim_state.omega_e = state[2];
    g_motor_sim_state.theta_e = state[3];

    // 计算机械状态
    g_motor_sim_state.omega_m = g_motor_sim_state.omega_e / g_motor_sim_params.pole_pairs;
    g_motor_sim_state.theta_m = g_motor_sim_state.theta_e / g_motor_sim_params.pole_pairs;

    // 计算电磁转矩
    g_motor_sim_state.Te = 1.5f * g_motor_sim_params.pole_pairs *
                           (g_motor_sim_params.flux_linkage * g_motor_sim_state.Iq +
                            (g_motor_sim_params.Ld - g_motor_sim_params.Lq) *
                            g_motor_sim_state.Id * g_motor_sim_state.Iq);

    // dq → αβ → abc
    float sin_theta = sinf(g_motor_sim_state.theta_e);
    float cos_theta = cosf(g_motor_sim_state.theta_e);
    float I_alpha, I_beta;
    inverse_park_transform(g_motor_sim_state.Id, g_motor_sim_state.Iq,
                          sin_theta, cos_theta, &I_alpha, &I_beta);
    inverse_clarke_transform(I_alpha, I_beta,
                            &g_motor_sim_state.Ia,
                            &g_motor_sim_state.Ib,
                            &g_motor_sim_state.Ic);

    // 更新仿真时间
    g_motor_sim_state.time += g_motor_sim_params.Ts;
}

const MotorSimState_t* MotorSim_GetState(void) {
    return &g_motor_sim_state;
}

void MotorSim_GetCurrents_ABC(float *Ia, float *Ib, float *Ic) {
    if (Ia) *Ia = g_motor_sim_state.Ia;
    if (Ib) *Ib = g_motor_sim_state.Ib;
    if (Ic) *Ic = g_motor_sim_state.Ic;
}

void MotorSim_GetCurrents_DQ(float *Id, float *Iq) {
    if (Id) *Id = g_motor_sim_state.Id;
    if (Iq) *Iq = g_motor_sim_state.Iq;
}

void MotorSim_GetMechanicalState(float *omega_m, float *theta_m, float *rpm) {
    if (omega_m) *omega_m = g_motor_sim_state.omega_m;
    if (theta_m) *theta_m = g_motor_sim_state.theta_m;
    if (rpm) *rpm = g_motor_sim_state.omega_m * 60.0f / TWO_PI;
}

void MotorSim_SetLoadTorque(float T_load) {
    g_motor_sim_params.T_load = T_load;
}

/* ADC模拟器函数实现 ---------------------------------------------------------*/

void ADCSim_Init(uint16_t resolution, float V_ref, float sense_resistor, float opamp_gain) {
    g_adc_simulator.resolution = resolution;
    g_adc_simulator.V_ref = V_ref;
    g_adc_simulator.sense_resistor = sense_resistor;
    g_adc_simulator.opamp_gain = opamp_gain;
    g_adc_simulator.offset_adc = (1 << (resolution - 1));  // 中点偏移

    // 默认无噪声和误差
    g_adc_simulator.noise_amplitude = 0.0f;
    g_adc_simulator.gain_error = 0.0f;
    g_adc_simulator.offset_error = 0.0f;

    memset(g_adc_simulator.adc_raw, 0, sizeof(g_adc_simulator.adc_raw));
}

void ADCSim_Sample(float Ia, float Ib, float Ic) {
    float currents[3] = {Ia, Ib, Ic};
    uint16_t max_adc = (1 << g_adc_simulator.resolution) - 1;

    for (int i = 0; i < 3; i++) {
        // 电流 → 采样电阻电压 → 运放输出电压
        float V_sense = currents[i] * g_adc_simulator.sense_resistor;
        float V_opamp = V_sense * g_adc_simulator.opamp_gain;

        // 转换为ADC值（双极性信号，中点偏移）
        float adc_ideal = (V_opamp / g_adc_simulator.V_ref) * max_adc + g_adc_simulator.offset_adc;

        // 增益误差
        adc_ideal *= (1.0f + g_adc_simulator.gain_error);

        // 偏移误差
        adc_ideal += g_adc_simulator.offset_error;

        // 噪声（简单的均匀分布噪声）
        if (g_adc_simulator.noise_amplitude > 0.0f) {
            float noise = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * g_adc_simulator.noise_amplitude;
            adc_ideal += noise;
        }

        // 量化和饱和
        int32_t adc_value = (int32_t)(adc_ideal + 0.5f);
        if (adc_value < 0) adc_value = 0;
        if (adc_value > max_adc) adc_value = max_adc;

        g_adc_simulator.adc_raw[i] = (uint16_t)adc_value;
    }
}

const uint16_t* ADCSim_GetRawValues(void) {
    return g_adc_simulator.adc_raw;
}

void ADCSim_SetErrors(float noise_amplitude, float gain_error, float offset_error) {
    g_adc_simulator.noise_amplitude = noise_amplitude;
    g_adc_simulator.gain_error = gain_error;
    g_adc_simulator.offset_error = offset_error;
}

const uint16_t* MotorSim_StepAndSample(float U_alpha_pu, float U_beta_pu) {
    // 电机步进
    MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);

    // ADC采样
    ADCSim_Sample(g_motor_sim_state.Ia, g_motor_sim_state.Ib, g_motor_sim_state.Ic);

    return ADCSim_GetRawValues();
}
