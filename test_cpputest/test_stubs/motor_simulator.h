/*============================================================================
    File Name     : motor_simulator.h
    Description   : PMSM电机模拟器 - 用于测试FOC控制算法
    Author        : ZHOUHENG
    Date          : 2025-11-20
    ----------------------------------------------------------------------
    功能说明：
    1. 接收SVPWM输出的电压指令（Uα, Uβ 或 Ua, Ub, Uc）
    2. 模拟PMSM电机的电气和机械动态特性
    3. 输出三相电流，模拟ADC采样值
    4. 支持负载转矩、转动惯量等参数配置

    数学模型：
    - 电压方程（dq坐标系）：
      dId/dt = (Ud - Rs·Id + ω·Lq·Iq) / Ld
      dIq/dt = (Uq - Rs·Iq - ω·Ld·Id - ω·λ) / Lq

    - 机械方程：
      Te = 1.5·Pn·(λ·Iq + (Ld-Lq)·Id·Iq)
      dω/dt = (Te - TL - B·ω) / J
      dθ/dt = ω

    - 数值积分：RK4（四阶龙格库塔法）
*=============================================================================*/

#ifndef __MOTOR_SIMULATOR_H
#define __MOTOR_SIMULATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 电机模拟器参数结构体 ------------------------------------------------------*/
typedef struct {
    // 电机电气参数
    float Rs;           // 定子电阻 (Ω)
    float Ld;           // d轴电感 (H)
    float Lq;           // q轴电感 (H)
    float flux_linkage; // 永磁体磁链 (Wb)
    float pole_pairs;   // 极对数

    // 机械参数
    float J;            // 转动惯量 (kg·m²)
    float B;            // 粘滞摩擦系数 (N·m·s/rad)

    // 运行参数
    float V_dc;         // 直流母线电压 (V)
    float T_load;       // 负载转矩 (N·m)

    // 采样参数
    float Ts;           // 仿真步长 (s)，通常等于PWM周期
    float I_base;       // 电流基准值 (A)，用于标幺化
} MotorSimParams_t;

/* 电机模拟器状态结构体 ------------------------------------------------------*/
typedef struct {
    // dq轴电流（状态变量）
    float Id;           // d轴电流 (A)
    float Iq;           // q轴电流 (A)

    // 机械状态
    float omega_e;      // 电角速度 (rad/s)
    float theta_e;      // 电角度 (rad)，范围 [-π, π]
    float omega_m;      // 机械角速度 (rad/s)
    float theta_m;      // 机械角度 (rad)

    // 三相电流（输出）
    float Ia;           // A相电流 (A)
    float Ib;           // B相电流 (A)
    float Ic;           // C相电流 (A)

    // 电磁转矩
    float Te;           // 电磁转矩 (N·m)

    // 仿真时间
    float time;         // 累计仿真时间 (s)
} MotorSimState_t;

/* ADC模拟器结构体 -----------------------------------------------------------*/
typedef struct {
    // ADC硬件参数
    uint16_t resolution;        // ADC分辨率（位数），默认12
    float V_ref;                // ADC参考电压 (V)
    float sense_resistor;       // 采样电阻 (Ω)
    float opamp_gain;           // 运放增益
    uint16_t offset_adc;        // 零点偏移（ADC值）

    // 噪声和误差模拟（可选）
    float noise_amplitude;      // 噪声幅度（ADC LSB）
    float gain_error;           // 增益误差（百分比）
    float offset_error;         // 偏移误差（ADC LSB）

    // 输出
    uint16_t adc_raw[3];        // ADC原始值：Ia, Ib, Ic
} ADCSimulator_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern MotorSimParams_t g_motor_sim_params;
extern MotorSimState_t g_motor_sim_state;
extern ADCSimulator_t g_adc_simulator;

/* 函数声明 ------------------------------------------------------------------*/

/**
 * @brief 初始化电机模拟器
 * @param params 电机参数指针
 * @note 必须在使用模拟器前调用
 */
void MotorSim_Init(const MotorSimParams_t *params);

/**
 * @brief 设置默认电机参数（基于项目中的motor_params）
 * @note 使用项目中定义的电机参数作为默认值
 */
void MotorSim_SetDefaultParams(void);

/**
 * @brief 电机模拟器步进（接收αβ坐标系电压）
 * @param U_alpha_pu α轴电压标幺值
 * @param U_beta_pu β轴电压标幺值
 * @note 执行一次仿真步进，更新电机状态
 */
void MotorSim_Step_AlphaBeta(float U_alpha_pu, float U_beta_pu);

/**
 * @brief 电机模拟器步进（接收dq坐标系电压）
 * @param U_d_pu d轴电压标幺值
 * @param U_q_pu q轴电压标幺值
 * @note 执行一次仿真步进，更新电机状态
 */
void MotorSim_Step_DQ(float U_d_pu, float U_q_pu);

/**
 * @brief 获取当前电机状态
 * @return 电机状态结构体指针
 */
const MotorSimState_t* MotorSim_GetState(void);

/**
 * @brief 获取三相电流（物理值）
 * @param Ia 输出A相电流 (A)
 * @param Ib 输出B相电流 (A)
 * @param Ic 输出C相电流 (A)
 */
void MotorSim_GetCurrents_ABC(float *Ia, float *Ib, float *Ic);

/**
 * @brief 获取dq轴电流（物理值）
 * @param Id 输出d轴电流 (A)
 * @param Iq 输出q轴电流 (A)
 */
void MotorSim_GetCurrents_DQ(float *Id, float *Iq);

/**
 * @brief 获取机械状态
 * @param omega_m 输出机械角速度 (rad/s)
 * @param theta_m 输出机械角度 (rad)
 * @param rpm 输出转速 (RPM)
 */
void MotorSim_GetMechanicalState(float *omega_m, float *theta_m, float *rpm);

/**
 * @brief 设置负载转矩
 * @param T_load 负载转矩 (N·m)
 */
void MotorSim_SetLoadTorque(float T_load);

/**
 * @brief 重置电机模拟器状态
 * @note 将所有状态变量清零
 */
void MotorSim_Reset(void);

/* ADC模拟器函数 -------------------------------------------------------------*/

/**
 * @brief 初始化ADC模拟器
 * @param resolution ADC分辨率（位数）
 * @param V_ref ADC参考电压 (V)
 * @param sense_resistor 采样电阻 (Ω)
 * @param opamp_gain 运放增益
 */
void ADCSim_Init(uint16_t resolution, float V_ref, float sense_resistor, float opamp_gain);

/**
 * @brief ADC采样模拟（从电机模拟器获取电流）
 * @param Ia A相电流 (A)
 * @param Ib B相电流 (A)
 * @param Ic C相电流 (A)
 * @note 将物理电流转换为ADC原始值
 */
void ADCSim_Sample(float Ia, float Ib, float Ic);

/**
 * @brief 获取ADC原始值
 * @return ADC原始值数组指针 [Ia_adc, Ib_adc, Ic_adc]
 */
const uint16_t* ADCSim_GetRawValues(void);

/**
 * @brief 设置ADC噪声和误差
 * @param noise_amplitude 噪声幅度（ADC LSB）
 * @param gain_error 增益误差（百分比，例如 0.01 表示 1%）
 * @param offset_error 偏移误差（ADC LSB）
 */
void ADCSim_SetErrors(float noise_amplitude, float gain_error, float offset_error);

/**
 * @brief 一步完成：电机步进 + ADC采样
 * @param U_alpha_pu α轴电压标幺值
 * @param U_beta_pu β轴电压标幺值
 * @return ADC原始值数组指针
 * @note 便捷函数，适合闭环测试
 */
const uint16_t* MotorSim_StepAndSample(float U_alpha_pu, float U_beta_pu);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SIMULATOR_H */
