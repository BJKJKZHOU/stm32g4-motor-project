/*============================================================================
    File Name     : FOC_math.c
    Description   : FOC数学计算模块 - 包含FOC相关的数学计算函数
    Author        : ZHOUHENG
    Date          : 2025-11-04
    ----------------------------------------------------------------------       

*=============================================================================
*/

#include "FOC_math.h"

#include <math.h>
#include <stdbool.h>
#include <limits.h>

#include "cordic.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif


#define INV_SQRT3_F           (0.5773502691896258f)
#define SQRT3_OVER_TWO_F      (0.8660254037844386f)
#define TWO_PI_F              (2.0f * PI)

#define Q31_SCALE_F           (2147483647.0f)


// 作用：数值饱和限制，确保值在指定范围内 (Value saturation: ensures value is within specified range)
static float foc_math_saturate(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}


// 作用：将角度包装到[-π, π]范围内 (Angle wrapping: constrains angle to [-π, π] range)
static float foc_math_wrap_angle(float angle)
{
    if (!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, TWO_PI_F);
    if (angle > PI) {
        angle -= TWO_PI_F;
    } else if (angle < -PI) {
        angle += TWO_PI_F;
    }
    return angle;
}

// 作用：计算三相电压的零序分量 (Zero-sequence calculation: computes zero-sequence component of three-phase voltages)
static float foc_math_zero_sequence(float ua, float ub, float uc)
{
    const float max_v = fmaxf(fmaxf(ua, ub), uc);
    const float min_v = fminf(fminf(ua, ub), uc);
    return -0.5f * (max_v + min_v);
}

// 作用：将标幺占空比转换为定时器计数值 (Per-unit duty to timer ticks: converts per-unit duty cycle to timer tick values)
static uint32_t foc_math_pu_to_ticks(float duty_pu)
{
    duty_pu = foc_math_saturate(duty_pu, 0.0f, 1.0f);

    float ticks = duty_pu * (float)ARR_PERIOD;
    if (ticks < 0.0f) {
        ticks = 0.0f;
    }

    if (ticks > (float)ARR_PERIOD) {
        ticks = (float)ARR_PERIOD;
    }

    return (uint32_t)(ticks + 0.5f);
}

static bool foc_math_is_valid_pu(float value)
{
    return isfinite(value) && fabsf(value) <= 1.2f;
}

void Inverse_Park_Transform(float U_d_pu, float U_q_pu, float sin_theta, float cos_theta,
                           float *U_alpha_pu, float *U_beta_pu)
{
    if (U_alpha_pu == NULL || U_beta_pu == NULL) {
        return;
    }

    if (!isfinite(U_d_pu) || !isfinite(U_q_pu)) {
        *U_alpha_pu = 0.0f;
        *U_beta_pu = 0.0f;
        return;
    }

    /* 使用CORDIC优化的逆Park变换：直接使用传入的正弦余弦值进行旋转矩阵计算 */
    /* 逆Park变换矩阵：[U_alpha] = [cosθ -sinθ][U_d] */
    /*                 [U_beta ]   [sinθ  cosθ][U_q] */
    
    /* 正弦余弦值来自CORDIC硬件加速，这里直接进行矩阵乘法 */
    *U_alpha_pu = U_d_pu * cos_theta - U_q_pu * sin_theta;
    *U_beta_pu  = U_d_pu * sin_theta + U_q_pu * cos_theta;
}

// 谐波注入SPWM实现SVPWM
void SVPWM_minmax(float U_alpha_pu, float U_beta_pu, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3)
{
    if (Tcm1 == NULL || Tcm2 == NULL || Tcm3 == NULL) {
        return;
    }

    // 输入参数验证
    if (!isfinite(U_alpha_pu) || !isfinite(U_beta_pu)) {
        *Tcm1 = *Tcm2 = *Tcm3 = ARR_PERIOD / 2;
        return;
    }

    const float U_a = U_alpha_pu;
    const float U_b = (-0.5f * U_alpha_pu) + (SQRT3_OVER_TWO_F * U_beta_pu);
    const float U_c = (-0.5f * U_alpha_pu) - (SQRT3_OVER_TWO_F * U_beta_pu);

    const float U_zero = foc_math_zero_sequence(U_a, U_b, U_c);

    const float Ua_injected = U_a + U_zero;
    const float Ub_injected = U_b + U_zero;
    const float Uc_injected = U_c + U_zero;

    const float Tcm1_pu = foc_math_saturate(Ua_injected + 0.5f, 0.0f, 1.0f);
    const float Tcm2_pu = foc_math_saturate(Ub_injected + 0.5f, 0.0f, 1.0f);
    const float Tcm3_pu = foc_math_saturate(Uc_injected + 0.5f, 0.0f, 1.0f);

    *Tcm1 = foc_math_pu_to_ticks(Tcm1_pu);
    *Tcm2 = foc_math_pu_to_ticks(Tcm2_pu);
    *Tcm3 = foc_math_pu_to_ticks(Tcm3_pu);
}


bool Clarke_Transform(float ia_pu, float ib_pu, float *I_alpha_pu, float *I_beta_pu)
{
    if (I_alpha_pu == NULL || I_beta_pu == NULL) {
        return false;
    }

    if (!foc_math_is_valid_pu(ia_pu) || !foc_math_is_valid_pu(ib_pu)) {
        *I_alpha_pu = 0.0f;
        *I_beta_pu = 0.0f;
        return false;
    }

    *I_alpha_pu = ia_pu;
    *I_beta_pu  = (ia_pu + 2.0f * ib_pu) * INV_SQRT3_F;
    return true;
}


void Sine_Cosine(float theta_e, float *sin_theta_e, float *cos_theta_e)
{
    if (sin_theta_e == NULL || cos_theta_e == NULL) {
        return;
    }

    const float wrapped = foc_math_wrap_angle(theta_e);
    
    /* 配置CORDIC用于余弦计算 */
    CORDIC_ConfigTypeDef sConfig = {0};
    sConfig.Function = CORDIC_FUNCTION_COSINE;      // 余弦函数
    sConfig.Scale = CORDIC_SCALE_0;                 // 无缩放
    sConfig.InSize = CORDIC_INSIZE_32BITS;          // 32位输入
    sConfig.OutSize = CORDIC_OUTSIZE_32BITS;        // 32位输出
    sConfig.NbWrite = CORDIC_NBWRITE_1;             // 1个输入
    sConfig.NbRead = CORDIC_NBREAD_2;               // 2个输出(cos, sin)
    sConfig.Precision = CORDIC_PRECISION_6CYCLES;   // 适中精度，适合电机控制
    
    if (HAL_CORDIC_Configure(&hcordic, &sConfig) != HAL_OK) {
        // 配置失败，回退到软件实现
        arm_sin_cos_f32(wrapped, sin_theta_e, cos_theta_e);
        return;
    }
    
    /* 将角度转换为Q1.31格式：[-π, π] → [-2^31, 2^31-1] */
    int32_t angle_q31 = (int32_t)(wrapped * Q31_SCALE_F / PI);
    int32_t results[2];
    
    /* 调用CORDIC计算 */
    if (HAL_CORDIC_Calculate(&hcordic, &angle_q31, results, 1, 20) == HAL_OK) {
        /* 转换Q1.31结果回浮点数 */
        *cos_theta_e = (float)results[0] / Q31_SCALE_F;
        *sin_theta_e = (float)results[1] / Q31_SCALE_F;
    } else {
        // 计算失败，回退到软件实现
        arm_sin_cos_f32(wrapped, sin_theta_e, cos_theta_e);
    }
}


void Park_Transform(float I_alpha_pu, float I_beta_pu, float sin_theta, float cos_theta, float *I_d, float *I_q)
{
    if (I_d == NULL || I_q == NULL) {
        return;
    }

    // 输入参数验证
    if (!isfinite(I_alpha_pu) || !isfinite(I_beta_pu) || !isfinite(sin_theta) || !isfinite(cos_theta)) {
        *I_d = 0.0f;
        *I_q = 0.0f;
        return;
    }

    // 检查sin和cos的有效性
    if (fabsf(sin_theta) > 1.1f || fabsf(cos_theta) > 1.1f) {
        *I_d = 0.0f;
        *I_q = 0.0f;
        return;
    }

    /* 使用CORDIC优化的Park变换：直接使用传入的正弦余弦值进行旋转矩阵计算 */
    /* Park变换矩阵：[I_d]   [cosθ  sinθ][I_alpha] */
    /*               [I_q] = [-sinθ cosθ][I_beta ]  */
    
    /* 由于CORDIC主要用于三角函数计算，而Park变换本质是2×2矩阵乘法 */
    /* 这里我们保持原有的矩阵乘法实现，但正弦余弦值可以来自CORDIC */
    *I_d = I_alpha_pu * cos_theta + I_beta_pu * sin_theta;
    *I_q = -I_alpha_pu * sin_theta + I_beta_pu * cos_theta;
}


/* =============================================================================
     低通滤波器模块实现 (Low Pass Filter Module Implementation)
     -----------------------------------------------------------------------------
     功能：实现一阶低通滤波器，支持Hz和rad/s频率单位
     
     滤波器传递函数：H(s) = ωc / (s + ωc)
     离散化实现（双线性变换）：
         y[n] = α * x[n] + (1-α) * y[n-1]
         其中 α = (2 * ωc * Ts) / (2 + ωc * Ts)
     
     频率单位转换：
         - Hz: ω = 2π * f
         - rad/s: 直接使用
   ============================================================================= */

float LPF_Filter(LPF_Float_t *filter, float input, float cutoff_freq, float sample_time, bool unit )
{
    /* 参数验证 */
    if (!isfinite(input) || !isfinite(cutoff_freq) || !isfinite(sample_time)) {
        return input;
    }
    
    if (cutoff_freq <= 0.0f || sample_time <= 0.0f) {
        return input;
    }
    
    /* 频率单位转换 */
    float omega_c;
    if (unit) {
        /* Hz转换为rad/s */
        omega_c = TWO_PI_F * cutoff_freq;
    } else {
        /* 直接使用rad/s */
        omega_c = cutoff_freq;
    }
    
    /* 计算滤波器系数 α = (2 * ωc * Ts) / (2 + ωc * Ts) */
    float denominator = 2.0f + omega_c * sample_time;
    if (denominator <= 0.0f) {
        return input;
    }
    
    float alpha = (2.0f * omega_c * sample_time) / denominator;
    
    /* 确保alpha在有效范围内 */
    alpha = foc_math_saturate(alpha, 0.0f, 1.0f);
    
    /* 低通滤波器递推公式：y[n] = α * x[n] + (1-α) * y[n-1] */
    float output = alpha * input + (1.0f - alpha) * filter->state;
    
    /* 更新状态 */
    filter->state = output;
    
    return output;
}



/**
 * @brief 初始化滤波器
 * @param filter 滤波器指针
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_time 采样时间 (秒)
 * @param init_val 初始输出值
 * 
 * @note 系数计算: α = 1 - exp(-2π * fc * Ts)
 */
void LPF_Init(LPF_1stOrder_t* filter, float cutoff_freq, float sample_time, int16_t init_val) {
    // 计算滤波器系数: α = 1 - exp(-2π * fc * Ts)
    float alpha = 1.0f - expf(-2.0f * 3.1415926535f * cutoff_freq * sample_time);
    
    // 限制系数范围确保稳定性
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    // 转换为Q16格式 (0x0000-0xFFFF 对应 0.0-1.0)
    filter->coeff = (uint16_t)(alpha * 65536.0f);
    
    // 初始化状态
    filter->state.sw.lo = init_val;
    filter->state.sw.hi = 0;
}

/**
 * @brief 滤波器更新函数 - 核心算法
 * @param filter 滤波器指针
 * @param input 输入信号
 * @return 滤波后的输出信号
 * 
 * 算法原理：
 * state.sl = state.sl + (input - state.sw.hi) * coeff
 * - 利用32位累加避免截断误差
 * - 结果自动分离：lo=输出，hi=累积误差
 */
int16_t LPF_Update(LPF_1stOrder_t* filter, int16_t input) {
    filter->state.sl = filter->state.sl + (int32_t)(input - filter->state.sw.hi) * filter->coeff;
    return filter->state.sw.lo;
}


void SVPWM_SectorBased(float Valpha, float Vbeta, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3, uint8_t *sector)
{
    // 输出变量初始化
    if (Tcm1 != NULL) *Tcm1 = 0;
    if (Tcm2 != NULL) *Tcm2 = 0;
    if (Tcm3 != NULL) *Tcm3 = 0;
    if (sector != NULL) *sector = 0;
    
 
    if (!isfinite(Valpha) || !isfinite(Vbeta)) {
        return;
    }
    
    // 使用宏定义计算PWM周期
    float Tpwm = 1.0f / PWM_FREQUENCY;  // PWM周期 = 1 / PWM频率
    
    float angle = atan2f(Vbeta, Valpha);
    uint8_t temp_sector = 0;
    
    // 六分仪判断：将360度分为6个60度扇区
    if (angle >= -PI/6 && angle < PI/6) {
        temp_sector = 1;
    } else if (angle >= PI/6 && angle < PI/2) {
        temp_sector = 2;
    } else if (angle >= PI/2 && angle < 5*PI/6) {
        temp_sector = 3;
    } else if (angle >= 5*PI/6 || angle < -5*PI/6) {
        temp_sector = 4;
    } else if (angle >= -5*PI/6 && angle < -PI/2) {
        temp_sector = 5;
    } else if (angle >= -PI/2 && angle < -PI/6) {
        temp_sector = 6;
    }
    
    if (sector != NULL) {
        *sector = temp_sector;
    }
    
    // 电压矢量幅值限制（标幺值范围[-1,1]）
    float V_mag = sqrtf(Valpha*Valpha + Vbeta*Vbeta);
    if (V_mag > 1.0f) {
        // 过调制处理：按比例缩小
        float scale = 1.0f / V_mag;
        Valpha *= scale;
        Vbeta *= scale;
        V_mag = 1.0f;
    }
    
    float T1 = 0.0f, T2 = 0.0f;
    
    // 根据扇区计算T1和T2（标幺化处理）
    switch(temp_sector) {
        case 1:
            T1 = -Valpha + Vbeta * INV_SQRT3_F;
            T2 = 2.0f * Vbeta * INV_SQRT3_F;
            break;
        case 2:
            T1 = Valpha + Vbeta * INV_SQRT3_F;
            T2 = -Valpha + Vbeta * INV_SQRT3_F;
            break;
        case 3:
            T1 = 2.0f * Vbeta * INV_SQRT3_F;
            T2 = -Valpha - Vbeta * INV_SQRT3_F;
            break;
        case 4:
            T1 = -2.0f * Vbeta * INV_SQRT3_F;
            T2 = Valpha - Vbeta * INV_SQRT3_F;
            break;
        case 5:
            T1 = -Valpha - Vbeta * INV_SQRT3_F;
            T2 = -2.0f * Vbeta * INV_SQRT3_F;
            break;
        case 6:
            T1 = Valpha - Vbeta * INV_SQRT3_F;
            T2 = Valpha + Vbeta * INV_SQRT3_F;
            break;
        default:
            T1 = 0.0f;
            T2 = 0.0f;
            break;
    }
    
    // 过调制处理
    if (T1 + T2 > Tpwm) {
        float scale = Tpwm / (T1 + T2);
        T1 = T1 * scale;
        T2 = T2 * scale;
    }
    
    // 7段式PWM生成（与原有实现保持一致）
    float ta = (Tpwm - (T1 + T2)) / 4.0f;
    float tb = ta + T1 / 2.0f;
    float tc = tb + T2 / 2.0f;
    
    // 输出调制信号
    float temp_Tcm1 = 0.0f, temp_Tcm2 = 0.0f, temp_Tcm3 = 0.0f;
    
    switch(temp_sector) {
        case 1:
            temp_Tcm1 = tb;
            temp_Tcm2 = ta;
            temp_Tcm3 = tc;
            break;
        case 2:
            temp_Tcm1 = ta;
            temp_Tcm2 = tc;
            temp_Tcm3 = tb;
            break;
        case 3:
            temp_Tcm1 = ta;
            temp_Tcm2 = tb;
            temp_Tcm3 = tc;
            break;
        case 4:
            temp_Tcm1 = tc;
            temp_Tcm2 = tb;
            temp_Tcm3 = ta;
            break;
        case 5:
            temp_Tcm1 = tc;
            temp_Tcm2 = ta;
            temp_Tcm3 = tb;
            break;
        case 6:
            temp_Tcm1 = tb;
            temp_Tcm2 = tc;
            temp_Tcm3 = ta;
            break;
        default:
            temp_Tcm1 = Tpwm / 2.0f;
            temp_Tcm2 = Tpwm / 2.0f;
            temp_Tcm3 = Tpwm / 2.0f;
            break;
    }
    
    // 归一化处理，与SVPWM函数保持一致
    const float Tcm1_pu = foc_math_saturate(2.0f * temp_Tcm1 / Tpwm, 0.0f, 1.0f);
    const float Tcm2_pu = foc_math_saturate(2.0f * temp_Tcm2 / Tpwm, 0.0f, 1.0f);
    const float Tcm3_pu = foc_math_saturate(2.0f * temp_Tcm3 / Tpwm, 0.0f, 1.0f);

    // 转换为定时器计数值
    if (Tcm1 != NULL) {
        *Tcm1 = foc_math_pu_to_ticks(Tcm1_pu);
    }
    if (Tcm2 != NULL) {
        *Tcm2 = foc_math_pu_to_ticks(Tcm2_pu);
    }
    if (Tcm3 != NULL) {
        *Tcm3 = foc_math_pu_to_ticks(Tcm3_pu);
    }
}