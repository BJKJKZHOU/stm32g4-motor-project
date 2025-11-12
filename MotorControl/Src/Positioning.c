/*============================================================================
    File Name     : Positioning.c
    Description   :  
    Author        : ZHOUHENG
    Date          : 2025-11-12
    ----------------------------------------------------------------------       
    
*=============================================================================
*/


#include "Positioning.h"

#include <math.h>  // 用于 fabsf 函数


const float IPD_PULSE_ANGLES[12] = {0, 180, 30, 210, 60, 240, 90, 270, 120, 300, 150, 330};


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