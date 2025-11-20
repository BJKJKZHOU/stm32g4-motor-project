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
#include <stdbool.h>
#include "PID_controller.h"
#include "Positioning.h"

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


/* ======================================================
 开环测试函数
 输入:
 - frequency_rad_s: 电频率参考值，单位 rad/s
 - Tcm1, Tcm2, Tcm3: 三相PWM占空比计数值 (0-ARR_PERIOD)
 功能：
 1. 根据电频率计算角度增量
 2. 更新电角度（-π到π循环）
 3. 生成三相正弦波（120°相位差）
 4. 完整的FOC变换流程（Clark→Park→逆Park→SVPWM）
** ======================================================
*/
void FOC_OpenLoopTest(float frequency_rad_s, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3);



/* ======================================================
    电流环

    初始化
    各种模块的初始化
    PID 控制器初始化
    调用IPD相关模块

    输入：电流设定值。进入PID控制器
    经PID控制器输出为电压设定值，
    经过 Inverse Park Transform 变换后为dq轴电压
    经过SVPWM_SectorBased变换后为三相PWM占空比计数值(0-ARR_PERIOD)

    ADC 相关模块采集电机的三相电流，
    Clarke Transform1 转换为αβ轴电流，
    （需要一个根据电压基值和SVPWM输出的参数计算逆变器输出电压的函数）
    注意要将逆变器输出的三相电压也转换为αβ轴电压
    非线性观测器 估计电角度，注意要将电流和电压都输入非线性观测器
    获得角度，SinCos Embedded Optimized 计算sin和cos值，
    Park Transform 转换为dq轴电流，此为反馈的电流，用于PID控制器的输入。

** ======================================================
*/

// 电流环控制器数据结构
typedef struct {
    // PID控制器参数和状态
    PID_Params_t pid_d_params;      // d轴电流PID参数
    PID_State_t pid_d_state;        // d轴电流PID状态
    PID_Params_t pid_q_params;      // q轴电流PID参数
    PID_State_t pid_q_state;        // q轴电流PID状态

    // 非线性观测器
    NonlinearObs_Position_t position_observer;  // 位置观测器

    // 控制周期
    float dt;                       // 控制周期 (s)

    // 电机ID
    uint8_t motor_id;               // 电机ID，用于获取电机参数

    // 上一次的PWM输出（用于计算逆变器电压）
    uint32_t last_Tcm1;
    uint32_t last_Tcm2;
    uint32_t last_Tcm3;

    // 调试和监控变量
    float id_setpoint;              // d轴电流设定值 (标幺值)
    float iq_setpoint;              // q轴电流设定值 (标幺值)
    float id_feedback;              // d轴电流反馈值 (标幺值)
    float iq_feedback;              // q轴电流反馈值 (标幺值)
    float ud_output;                // d轴电压输出 (标幺值)
    float uq_output;                // q轴电压输出 (标幺值)
    float theta_elec;               // 电角度 (rad)
    float omega_elec;               // 电角速度 (rad/s)

    // 状态标志
    bool is_initialized;            // 初始化标志
    bool is_running;                // 运行标志

} CurrentLoop_t;

/**
 * @brief 电流环初始化
 * @param loop 电流环控制器结构体指针
 * @param motor_id 电机ID
 * @param dt 控制周期 (s)
 * @param kp_d d轴电流环比例系数
 * @param ki_d d轴电流环积分系数
 * @param kp_q q轴电流环比例系数
 * @param ki_q q轴电流环积分系数
 * @param observer_gamma 观测器增益参数
 */
void CurrentLoop_Init(CurrentLoop_t *loop, uint8_t motor_id, float dt,
                     float kp_d, float ki_d, float kp_q, float ki_q,
                     float observer_gamma);

/**
 * @brief 电流环主控制函数
 * @param loop 电流环控制器结构体指针
 * @param id_ref d轴电流设定值 (标幺值)
 * @param iq_ref q轴电流设定值 (标幺值)
 * @param Tcm1 输出：三相PWM占空比计数值 (0-ARR_PERIOD)
 * @param Tcm2 输出：三相PWM占空比计数值 (0-ARR_PERIOD)
 * @param Tcm3 输出：三相PWM占空比计数值 (0-ARR_PERIOD)
 * @return true=成功, false=失败
 *
 * @note 此函数执行完整的电流环控制流程：
 *       1. ADC采集三相电流
 *       2. Clarke变换：abc → αβ
 *       3. 计算逆变器输出电压（αβ坐标系）
 *       4. 非线性观测器估计电角度
 *       5. Park变换：αβ → dq（电流反馈）
 *       6. PID控制器计算dq轴电压（含解耦补偿）
 *       7. 逆Park变换：dq → αβ（电压输出）
 *       8. SVPWM生成PWM占空比
 */
bool CurrentLoop_Run(CurrentLoop_t *loop, float id_ref, float iq_ref,
                    uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3);

/**
 * @brief 电流环复位
 * @param loop 电流环控制器结构体指针
 */
void CurrentLoop_Reset(CurrentLoop_t *loop);



















#ifdef __cplusplus
}
#endif

#endif /* FOC_LOOP_H */

