/*============================================================================
    File Name     : FOC_Loop.c
    Description   : FOC循环模块 - 包含FOC循环相关的函数
    Author        : ZHOUHENG
    Date          : 2025-11-13
    ----------------------------------------------------------------------       
    开环 给定DQ参考电压
        1、先搭建开环控制流程，检查 SVPWM 输出是否正常
    电流环
        电流采样，位置反馈，坐标变换，发波，PID控制
    速度环 
*=============================================================================
*/

#include "FOC_Loop.h"
#include "main.h"
#include "FOC_math.h"

#include <math.h>

// 全局变量 - 用于跟踪电角度
static float g_electrical_angle = 0.0f;

// 电机参数 - 需要根据实际电机调整
#define MOTOR_POLE_PAIRS 6        // 电机极对数
#define VOLTAGE_AMPLITUDE 1.0f    // 电压幅值（标幺值），与SVPWM_minmax使用相同的定义 

#define SAMPLE_TIME 0.00005f     // 采样时间 50us (20kHz 中断频率)

// 用于调试 - 可以导出查看角度变化
volatile float g_debug_angle = 0.0f;
volatile float g_debug_sin = 0.0f;
volatile float g_debug_cos = 0.0f;
volatile float g_debug_frequency = 0.0f;  // 当前电频率

volatile uint8_t sector = 0;

void FOC_OpenLoopTest(float frequency_rad_s, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3)
{

    // 参数检查
    if (Tcm1 == NULL || Tcm2 == NULL || Tcm3 == NULL) {
        return;
    }
    
    // 1. 根据电频率计算角度增量
    float angle_increment = frequency_rad_s * SAMPLE_TIME;
    
    // 2. 更新电角度（保持在[-π, π]范围内）
    g_electrical_angle += angle_increment;
    
    // 角度归一化到[-π, π]
    if (g_electrical_angle > PI) {
        g_electrical_angle -= 2.0f * PI;
    } else if (g_electrical_angle < -PI) {
        g_electrical_angle += 2.0f * PI;
    }
    
    // 3. 计算正弦和余弦值（使用CORDIC硬件加速）
    float sin_theta, cos_theta;
    Sine_Cosine(g_electrical_angle, &sin_theta, &cos_theta);
    
    // 4. 生成三相120度正弦波（标幺值范围[-1, 1]）
    float U_phase = sin_theta;                                   // U相：sin(θ)
    float V_phase = sinf(g_electrical_angle - 2.0f * PI / 3.0f); // V相：sin(θ - 120°)
    //float W_phase = sinf(g_electrical_angle + 2.0f * PI / 3.0f); // W相：sin(θ + 120°)
    
    // 5. Clark变换：将三相电流转换为αβ坐标系
    float I_alpha, I_beta;
    Clarke_Transform(U_phase, V_phase, &I_alpha, &I_beta);
    
    // 6. Park变换：将αβ坐标系转换为dq坐标系
    float I_d, I_q;
    Park_Transform(I_alpha, I_beta, sin_theta, cos_theta, &I_d, &I_q);
    
    // 7. 设置固定的开环电压矢量
    // 在开环测试中，通常设置U_d=0，U_q为固定值来产生旋转磁场
    const float U_d_pu = 0.0f;  // d轴电压设为0
    const float U_q_pu = VOLTAGE_AMPLITUDE;  // q轴电压使用固定值
    
    // 8. 逆Park变换：将dq电压转换为αβ电压
    float U_alpha_pu, U_beta_pu;
    Inverse_Park_Transform(U_d_pu, U_q_pu, sin_theta, cos_theta, &U_alpha_pu, &U_beta_pu);
    
    // 9. SVPWM调制：将αβ电压转换为PWM占空比
    //SVPWM_minmax(U_alpha_pu, U_beta_pu, Tcm1, Tcm2, Tcm3);
    SVPWM_SectorBased(U_alpha_pu, U_beta_pu, Tcm1, Tcm2, Tcm3, (uint8_t *)&sector);
    // 调试信息更新 
    g_debug_angle = g_electrical_angle;
    g_debug_sin = sin_theta;
    g_debug_cos = cos_theta;
    g_debug_frequency = frequency_rad_s;
}

