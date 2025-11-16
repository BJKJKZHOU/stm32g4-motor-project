/*============================================================================
    File Name     : FOC_Loop.h
    Description   : FOC循环模块 - 包含FOC循环相关的函数
    Author        : ZHOUHENG
    Date          : 2025-11-13
    ----------------------------------------------------------------------       
     
*=============================================================================
*/


#ifndef FOC_LOOP_H
#define FOC_LOOP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// 调试变量 - 用于观察角度变化
extern volatile float g_debug_angle;  // 当前电角度 (rad)
extern volatile float g_debug_sin;    // sin(θ)
extern volatile float g_debug_cos;    // cos(θ)
extern volatile float g_debug_frequency;  // 当前电频率 (rad/s)

extern volatile uint32_t g_tim1_interrupt_count;     // TIM1中断计数
extern volatile float g_tim1_interrupt_freq_hz;      // TIM1中断频率 (Hz)

// 开环测试函数
// 输入:
// - frequency_rad_s: 电频率参考值，单位 rad/s
// - Tcm1, Tcm2, Tcm3: 三相PWM占空比计数值 (0-ARR_PERIOD)
// 功能：
// 1. 根据电频率计算角度增量
// 2. 更新电角度（-π到π循环）
// 3. 生成三相正弦波（120°相位差）
// 4. 完整的FOC变换流程（Clark→Park→逆Park→SVPWM）
void FOC_OpenLoopTest(float frequency_rad_s, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3);

















#ifdef __cplusplus
}
#endif

#endif /* FOC_LOOP_H */

