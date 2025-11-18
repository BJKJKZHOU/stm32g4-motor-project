/*============================================================================
    File Name     : Positioning.h
    Description   : 电机位置相关 
    Author        : ZHOUHENG
    Date          : 2025-11-12
    ----------------------------------------------------------------------       
    ## 主要是根据电流信号来判断电机的位置
    1、转子初始位置估计，磁极，相位。
    2、位置观测。
    3、位置传感器的上层模块，暂时没有位置传感器。

    ### 脉冲法(IPD)实现转子初始定位 
        1、在 d 轴发送特定角度的电压脉冲，12个脉冲，每个脉冲角度差为 30 度。
            电压脉冲幅值：
            
            持续时间：
    
        3、记录每个脉冲对应的电流值。
            发一次脉冲后，下一次发脉冲在对角位置。
            顺序      角度       脉冲对标记       采样值
            1          0°         P1+           I1
            2          180°       P1-           I2  
            3          30°        P2+           I3
            4          210°       P2-           I4
            5          60°        P3+           I5
            6          240°       P3-           I6
            7          90°        P4+           I7  
            8          270°       P4-           I8
            9          120°       P5+           I9
            10         300°       P5-           I10
            11         150°       P6+           I11
            12         330°       P6-           I12

    
        4、根据电流值，判断电机的磁极和相位。
            在一对测试中，电流响应峰值大的N极。
            多个方向的测试中，电流响应峰值最大的角度为转子位置
            如有编码器，此角度与编码器角度有差异，需要校准。

    ### 转子位置估计(无位置传感器)
        论文：Sensorless Control of Surface-Mount  Permanent-Magnet  
                Synchronous Motors  Based on a Nonlinear Observer


*=============================================================================
*/


#ifndef POSITIONING_H
#define POSITIONING_H


#include <stdbool.h>
#include <stdint.h>

#include "normalization.h"

#ifdef __cplusplus
extern "C" {
#endif


 extern const float IPD_PULSE_ANGLES[12];
  

typedef struct {
    float angle_elec;       // 电角度 (度)
    float current_sample;   // 电流采样值
} IPD_Pulse_t;


float IPD_CalculateRotorPosition(IPD_Pulse_t pulses[12]);

//=============================================================================
// 非线性观测器结构体 (使用项目归一化参数)
typedef struct {
    uint8_t motor_id;                    // 电机ID，用于获取对应参数
    
    // 观测器增益参数 (归一化值)
    float gamma;                         // 观测器增益 > 0
    
    // 状态估计变量 (归一化值)
    float x_hat_alpha_pu;               // 状态估计值 x_alpha (标幺值)
    float x_hat_beta_pu;                // 状态估计值 x_beta (标幺值)
    
    // 位置估计输出
    float theta_hat_rad;                // 转子位置估计 (电角度, 弧度)
    float theta_hat_deg;                // 转子位置估计 (电角度, 度)
    
    // 观测器状态标志
    bool is_initialized;                // 初始化标志
    bool is_converged;                  // 收敛标志
    
    // 归一化基值缓存（用于在电机未激活时维持标幺运算）
    normalization_base_values_t base_values;
    bool base_valid;
    
    // 误差统计，用于收敛判定
    float eta_norm_sq_filtered;
    float eta_norm_sq_last;
    uint16_t stable_counter;
    uint16_t stable_required;
    bool error_metrics_valid;
    
} NonlinearObs_Position_t;

// 非线性观测器函数声明
void NonlinearObs_Position_Init(NonlinearObs_Position_t *obs, uint8_t motor_id, float gamma);
void NonlinearObs_Position_Reset(NonlinearObs_Position_t *obs);
void NonlinearObs_Position_Update(NonlinearObs_Position_t *obs, 
                                  float i_alpha_pu, float i_beta_pu, 
                                  float v_alpha_pu, float v_beta_pu, 
                                  float dt);

// 获取位置估计结果 (弧度)
float NonlinearObs_Position_GetThetaRad(NonlinearObs_Position_t *obs);
// 获取位置估计结果 (度)  
float NonlinearObs_Position_GetThetaDeg(NonlinearObs_Position_t *obs);




#ifdef __cplusplus
}
#endif

#endif /* POSITIONING_H */
