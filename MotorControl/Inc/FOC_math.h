/*============================================================================
    File Name     : FOC_math.h
    Description   : FOC数学计算模块 - 包含FOC相关的数学计算函数
    Author        : ZHOUHENG
    Date          : 2025-11-04
    ----------------------------------------------------------------------       
    ## 部分说明
    中心对齐模式说明：
        计数器从0递增到ARR，然后递减到0
        完整周期 = 2 × ARR 个时钟周期  
        占空比 = CCR / ARR

    ARR_PERIOD为宏定义，为定时器的自动重装载值，即ARR
    
    标幺化方案
        PID输出 → U_d, U_q（电压值）
        标幺化处理 → U_d_pu, U_q_pu = f(U_d, U_q) ∈ [-1,1] 
        逆Park变换 → U_alpha_pu, U_beta_pu（自动限制）
        逆克拉克变换 → Ua_pu, Ub_pu, Uc_pu（标幺值）
        零序电压注入 → U_zero_pu = -0.5*(max+min)（标幺值）
        占空比计算 → Tcm_pu = U_injected_pu + 0.5（标幺值，范围[0,1]）
        最终恢复 → Tcm_actual = Tcm_pu * ARR_PERIOD（实际PWM计数值）

    ## 模块

    Inverse_Park_Transform 逆帕克变换
        Input ：PID模块 Output的电压设定值 U_d, U_q
                Sine-Cosine模块 Output的正弦余弦值 Sin(θe)、Cos(θe)

         Output : U_alpha，U_beta, 两相旋转坐标系(α-β)下设定电压信号

    SVPWM 空间矢量脉宽调制
        Input : Inverse Park Transform 的 Output，两相旋转坐标系(α-β)下的U_alpha、U_beta和
                直流母线电压 Vdc
        Output : Tcm1, Tcm2, Tcm3，定时器参数

        U_Limiter_module 限幅模块 放在SVPWM内部 使用标幺化参数时不使用此模块
            Input : U_alpha, U_beta 两相旋转坐标系(α-β)下设定电压信号
                    直流母线电压 Vdc
            Output : U_alpha_limited, U_beta_limited, 限幅后的两相旋转坐标系(α-β)下限幅电压信号
            方案 ：
                计算电压矢量幅值：U_mag = sqrt(U_alpha² + U_beta²)
                计算线性区上限：max_voltage = Vdc * SQRT3_OVER_TWO
                判断是否需要限幅：
                    if U_mag > max_voltage:
                        scale = max_voltage / U_mag
                        U_alpha_limited = U_alpha * scale
                        U_beta_limited = U_beta * scale

        Inverse_Clarke_Transform 逆克拉克变换  此模块放在 SVPWM 内部
            Input : U_alpha，U_beta，
            Output : U_a, U_b, U_c, 三相 (abc) 中的参考电压信号

            SVPWM方案：零序电压注入  此模块放在 SVPWM 内部

                U_zero  = -0.5*(max(Ua, Ub, Uc) + min(Ua, Ub, Uc))

                Ua_injected = Ua + U_zero 
                Ub_injected = Ub + U_zero 
                Uc_injected = Uc + U_zero 

                Tcm1_ = (Ua_injected/V_DC + 0.5)  //占空比，使用时需乘以ARR（ARR_PERIOD）
                Tcm2_ = (Ub_injected/V_DC + 0.5)  //占空比，使用时需乘以ARR（ARR_PERIOD）
                Tcm3_ = (Uc_injected/V_DC + 0.5)  //占空比，使用时需乘以ARR（ARR_PERIOD）

                Tcm1 = Tcm1_ * ARR_PERIOD  //占空比
                Tcm2 = Tcm2_ * ARR_PERIOD  //占空比
                Tcm3 = Tcm3_ * ARR_PERIOD  //占空比

            SVPWM标幺化方案：

                float U_zero_pu = -0.5f * (fmaxf(fmaxf(Ua_pu, Ub_pu), Uc_pu)
                         + fminf(fminf(Ua_pu, Ub_pu), Uc_pu));
                float Ua_injected_pu = Ua_pu + U_zero_pu;
                float Ub_injected_pu = Ub_pu + U_zero_pu;
                float Uc_injected_pu = Uc_pu + U_zero_pu;

                float Tcm1_pu = arm_sat_f32(Ua_injected_pu + 0.5f, 0.0f, 1.0f);
                float Tcm2_pu = arm_sat_f32(Ub_injected_pu + 0.5f, 0.0f, 1.0f);
                float Tcm3_pu = arm_sat_f32(Uc_injected_pu + 0.5f, 0.0f, 1.0f);

                float Tcm1 = Tcm1_pu * ARR_PERIOD;  //占空比 
                float Tcm2 = Tcm2_pu * ARR_PERIOD;  //占空比
                float Tcm3 = Tcm3_pu * ARR_PERIOD;  //占空比



    Clarke_Transform 克拉克变换(等幅值变换)
        Input : 三相 (abc) 中的两个电流信号，自动计算第三个信号
                ia + ib + ic = 0 => ic =0 - ia - ib
                基尔霍夫电流定律，电机三相电流和为 0.
        Output : I_alpha、I_beta，两相旋转坐标系(α-β)下电流信号

    Park_Transform 帕克变换
        Input : Clarke Transform 的 Output，两相旋转坐标系(α-β)下电流信号 I_alpha、I_beta
                Sine-Cosine模块 Output的正弦余弦值 Sin(θe)、Cos(θe)

        Output : id_feedback, iq_feedback，两相旋转坐标系(α-β)下电流信号
                 反馈的dq电流信号,与电流设定值做差后得到dq电流误差

    Sine_Cosine 角度转换为正弦余弦 注意输入信号单位
        Input : 角度信号 θe，单位为弧度 (rad)
                需要判断是否需要进行机械角和电角度的转换
        Output : Sine waveform output with a frequency that is identical to 
                 the position or phase signal () frequency.θe
                 Cosine waveform output with a frequency that is identical to 
                 the position or phase signal () frequency.θe

    

*=============================================================================
*/


#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h> 
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/* 数据格式约定:
 * - 相电流必须在调用Clarke_Transform之前转换为标幺值/定点数。
 * - 电角度以弧度(float)形式提供给Sine_Cosine；该函数仅将其包装到[-pi, pi]范围。
 * - Park/逆Park变换、SVPWM和下游PWM映射都基于标幺值进行操作。
 */
void Inverse_Park_Transform(float U_d_pu, float U_q_pu, float sin_theta, float cos_theta,
                           float *U_alpha_pu, float *U_beta_pu);

// 谐波注入SPWM实现SVPWM
void SVPWM_minmax(float U_alpha_pu, float U_beta_pu, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3);

bool Clarke_Transform(float ia_pu, float ib_pu, float *I_alpha_pu, float *I_beta_pu);

void Sine_Cosine(float theta_e, float *sin_theta_e, float *cos_theta_e);

void Park_Transform(float I_alpha_pu, float I_beta_pu, float sin_theta, float cos_theta, float *I_d, float *I_q);

// 谐波注入SPWM实现SVPWM 带扇区输出
void SVPWM_SectorBased(float Valpha, float Vbeta, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3, uint8_t *sector);

/**
 * @brief 根据PWM占空比和直流母线电压计算逆变器输出三相电压（实际值）
 * @param Tcm1 A相PWM占空比计数值 (0-ARR_PERIOD)
 * @param Tcm2 B相PWM占空比计数值 (0-ARR_PERIOD)
 * @param Tcm3 C相PWM占空比计数值 (0-ARR_PERIOD)
 * @param V_DC 直流母线电压 (V)
 * @param Ua 输出：A相电压 (V)
 * @param Ub 输出：B相电压 (V)
 * @param Uc 输出：C相电压 (V)
 *
 * @note 计算流程：
 *       1. PWM占空比计数值 → 占空比 (0-1)：duty = Tcm / ARR_PERIOD
 *       2. 占空比 → 三相电压：Ua = (duty - 0.5) * V_DC
 *       此函数模拟计算逆变器实际输出到电机的三相电压
 */
void PWM_To_Voltage_ABC(uint32_t Tcm1, uint32_t Tcm2, uint32_t Tcm3, float V_DC,
                       float *Ua, float *Ub, float *Uc);

                    
/*  低通滤波器模块 - 支持Hz和rad/s频率单位，单位在内部处理
    unit: true=Hz, false=rad/s
    sample_time: 采样时间，采样时间 = 1/采样频率，
    cutoff_freq: 截止频率
    此滤波器的执行频率就是采样频率，50微秒调用一次的化，采样时间为0.00005s。

*/

/* 低通滤波器结构体 - 支持多个独立滤波器实例 */
typedef struct {
    float state;  /* 滤波器状态变量 y[n-1] */
} LPF_Float_t;

float LPF_Filter(LPF_Float_t *filter, float input, float cutoff_freq, float sample_time, bool unit );


// ===================================================================================
// 简化低通滤波器 Q15
typedef union {
    int32_t sl;          // 32位整体访问（用于误差累积）
    struct {
        int16_t lo;      // 低16位 - 滤波输出值
        int16_t hi;      // 高16位 - 累积误差
    } sw;
} FilterOut_t;

// 滤波器结构体
typedef struct {
    FilterOut_t state;   // 滤波器状态
    uint16_t coeff;      // 滤波器系数（Q16格式）
} LPF_1stOrder_t;

// 函数声明
void LPF_Init(LPF_1stOrder_t* filter, float cutoff_freq, float sample_time, int16_t init_val);
int16_t LPF_Update(LPF_1stOrder_t* filter, int16_t input);

// ===================================================================================




#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
