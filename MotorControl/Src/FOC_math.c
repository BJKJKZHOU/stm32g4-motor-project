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
#define Q31_MAX               ((q31_t)0x7FFFFFFF)
#define Q31_MIN               ((q31_t)0x80000000)
#define Q31_HALF              ((q31_t)0x40000000)
#define Q31_NEG_HALF          ((q31_t)0xC0000000)
#define INV_SQRT3_Q31         ((q31_t)0x49E69D16)
#define SQRT3_OVER_TWO_Q31    ((q31_t)0x6ED9EBA1)


static inline q31_t foc_math_q31_saturate_int64(int64_t value)
{
    if (value > (int64_t)INT32_MAX) {
        return (q31_t)INT32_MAX;
    }
    if (value < (int64_t)INT32_MIN) {
        return (q31_t)INT32_MIN;
    }
    return (q31_t)value;
}

static q31_t foc_math_q31_add(q31_t a, q31_t b)
{
    int64_t sum = (int64_t)a + (int64_t)b;
    return foc_math_q31_saturate_int64(sum);
}

static q31_t foc_math_q31_sub(q31_t a, q31_t b)
{
    int64_t diff = (int64_t)a - (int64_t)b;
    return foc_math_q31_saturate_int64(diff);
}

static q31_t foc_math_q31_mul(q31_t a, q31_t b)
{
    int64_t product = (int64_t)a * (int64_t)b;
    product >>= 31;
    return foc_math_q31_saturate_int64(product);
}

static q31_t foc_math_q31_clamp(q31_t value, q31_t min_value, q31_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static inline float foc_math_q31_to_float(q31_t value)
{
    return (float)value / Q31_SCALE_F;
}

static q31_t foc_math_q31_from_float(float value)
{
    if (!isfinite(value)) {
        return 0;
    }
    if (value >= (float)(Q31_MAX - 1) / Q31_SCALE_F) {
        return Q31_MAX;
    }
    if (value <= -1.0f) {
        return Q31_MIN;
    }
    float scaled = value * Q31_SCALE_F;
    if (scaled > (float)INT32_MAX) {
        return Q31_MAX;
    }
    if (scaled < (float)INT32_MIN) {
        return Q31_MIN;
    }
    return (q31_t)scaled;
}

static bool foc_math_q31_is_unit(q31_t value)
{
    return value <= Q31_MAX && value >= Q31_MIN;
}

static inline q31_t foc_math_q31_max3(q31_t a, q31_t b, q31_t c)
{
    q31_t max_ab = (a > b) ? a : b;
    return (max_ab > c) ? max_ab : c;
}

static inline q31_t foc_math_q31_min3(q31_t a, q31_t b, q31_t c)
{
    q31_t min_ab = (a < b) ? a : b;
    return (min_ab < c) ? min_ab : c;
}

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

bool Inverse_Park_TransformQ31(q31_t U_d_q31, q31_t U_q_q31, q31_t sin_theta_q31, q31_t cos_theta_q31,
                               q31_t *U_alpha_q31, q31_t *U_beta_q31)
{
    if (U_alpha_q31 == NULL || U_beta_q31 == NULL) {
        return false;
    }

    if (!foc_math_q31_is_unit(sin_theta_q31) || !foc_math_q31_is_unit(cos_theta_q31)) {
        *U_alpha_q31 = 0;
        *U_beta_q31 = 0;
        return false;
    }

    /* 使用CORDIC优化的逆Park变换Q31版本 */
    /* 正弦余弦值来自CORDIC硬件加速，这里直接进行Q31矩阵乘法 */
    q31_t term_d_cos = foc_math_q31_mul(U_d_q31, cos_theta_q31);
    q31_t term_q_sin = foc_math_q31_mul(U_q_q31, sin_theta_q31);
    q31_t term_d_sin = foc_math_q31_mul(U_d_q31, sin_theta_q31);
    q31_t term_q_cos = foc_math_q31_mul(U_q_q31, cos_theta_q31);

    *U_alpha_q31 = foc_math_q31_sub(term_d_cos, term_q_sin);
    *U_beta_q31  = foc_math_q31_add(term_d_sin, term_q_cos);

    return true;
}

void SVPWM(float U_alpha_pu, float U_beta_pu, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3)
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

bool SVPWM_Q31(q31_t U_alpha_q31, q31_t U_beta_q31, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3)
{
    if (Tcm1 == NULL || Tcm2 == NULL || Tcm3 == NULL) {
        return false;
    }

    if (!foc_math_q31_is_unit(U_alpha_q31) || !foc_math_q31_is_unit(U_beta_q31)) {
        *Tcm1 = *Tcm2 = *Tcm3 = ARR_PERIOD / 2;
        return false;
    }

    const q31_t U_a = U_alpha_q31;
    const q31_t half_alpha = foc_math_q31_mul(Q31_NEG_HALF, U_alpha_q31);
    const q31_t beta_term = foc_math_q31_mul(SQRT3_OVER_TWO_Q31, U_beta_q31);

    const q31_t U_b = foc_math_q31_add(half_alpha, beta_term);
    const q31_t U_c = foc_math_q31_sub(half_alpha, beta_term);

    const q31_t max_v = foc_math_q31_max3(U_a, U_b, U_c);
    const q31_t min_v = foc_math_q31_min3(U_a, U_b, U_c);
    const q31_t sum_extremes = foc_math_q31_add(max_v, min_v);
    const q31_t U_zero = foc_math_q31_mul(sum_extremes, Q31_NEG_HALF);

    const q31_t Ua_injected = foc_math_q31_add(U_a, U_zero);
    const q31_t Ub_injected = foc_math_q31_add(U_b, U_zero);
    const q31_t Uc_injected = foc_math_q31_add(U_c, U_zero);

    q31_t Tcm1_q31 = foc_math_q31_add(Ua_injected, Q31_HALF);
    q31_t Tcm2_q31 = foc_math_q31_add(Ub_injected, Q31_HALF);
    q31_t Tcm3_q31 = foc_math_q31_add(Uc_injected, Q31_HALF);

    Tcm1_q31 = foc_math_q31_clamp(Tcm1_q31, 0, Q31_MAX);
    Tcm2_q31 = foc_math_q31_clamp(Tcm2_q31, 0, Q31_MAX);
    Tcm3_q31 = foc_math_q31_clamp(Tcm3_q31, 0, Q31_MAX);

    *Tcm1 = foc_math_pu_to_ticks(foc_math_q31_to_float(Tcm1_q31));
    *Tcm2 = foc_math_pu_to_ticks(foc_math_q31_to_float(Tcm2_q31));
    *Tcm3 = foc_math_pu_to_ticks(foc_math_q31_to_float(Tcm3_q31));

    return true;
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

bool Clarke_TransformQ31(q31_t ia_q31, q31_t ib_q31, q31_t *I_alpha_q31, q31_t *I_beta_q31)
{
    if (I_alpha_q31 == NULL || I_beta_q31 == NULL) {
        return false;
    }

    if (!foc_math_q31_is_unit(ia_q31) || !foc_math_q31_is_unit(ib_q31)) {
        *I_alpha_q31 = 0;
        *I_beta_q31 = 0;
        return false;
    }

    const q31_t two_ib = foc_math_q31_add(ib_q31, ib_q31);
    const q31_t sum = foc_math_q31_add(ia_q31, two_ib);

    *I_alpha_q31 = ia_q31;
    *I_beta_q31  = foc_math_q31_mul(sum, INV_SQRT3_Q31);

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

void Sine_CosineQ31(float theta_e, q31_t *sin_theta_q31, q31_t *cos_theta_q31)
{
    if (sin_theta_q31 == NULL || cos_theta_q31 == NULL) {
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
        float sin_val, cos_val;
        arm_sin_cos_f32(wrapped, &sin_val, &cos_val);
        *sin_theta_q31 = foc_math_q31_from_float(sin_val);
        *cos_theta_q31 = foc_math_q31_from_float(cos_val);
        return;
    }
    
    /* 将角度转换为Q1.31格式：[-π, π] → [-2^31, 2^31-1] */
    int32_t angle_q31 = (int32_t)(wrapped * Q31_SCALE_F / PI);
    int32_t results[2];
    
    /* 调用CORDIC计算 */
    if (HAL_CORDIC_Calculate(&hcordic, &angle_q31, results, 1, 20) == HAL_OK) {
        /* 直接使用Q1.31格式结果 */
        *cos_theta_q31 = (q31_t)results[0];
        *sin_theta_q31 = (q31_t)results[1];
    } else {
        // 计算失败，回退到软件实现
        float sin_val, cos_val;
        arm_sin_cos_f32(wrapped, &sin_val, &cos_val);
        *sin_theta_q31 = foc_math_q31_from_float(sin_val);
        *cos_theta_q31 = foc_math_q31_from_float(cos_val);
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

bool Park_TransformQ31(q31_t I_alpha_q31, q31_t I_beta_q31, q31_t sin_theta_q31, q31_t cos_theta_q31,
                       q31_t *I_d_q31, q31_t *I_q_q31)
{
    if (I_d_q31 == NULL || I_q_q31 == NULL) {
        return false;
    }

    if (!foc_math_q31_is_unit(sin_theta_q31) || !foc_math_q31_is_unit(cos_theta_q31)) {
        *I_d_q31 = 0;
        *I_q_q31 = 0;
        return false;
    }

    /* 使用CORDIC优化的Park变换Q31版本 */
    /* 正弦余弦值来自CORDIC硬件加速，这里直接进行Q31矩阵乘法 */
    const q31_t term_alpha_cos = foc_math_q31_mul(I_alpha_q31, cos_theta_q31);
    const q31_t term_beta_sin  = foc_math_q31_mul(I_beta_q31, sin_theta_q31);
    const q31_t term_alpha_sin = foc_math_q31_mul(I_alpha_q31, sin_theta_q31);
    const q31_t term_beta_cos  = foc_math_q31_mul(I_beta_q31, cos_theta_q31);

    *I_d_q31 = foc_math_q31_add(term_alpha_cos, term_beta_sin);
    *I_q_q31 = foc_math_q31_sub(term_beta_cos, term_alpha_sin);

    return true;
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

q31_t LPF_FilterQ31(LPF_Q31_t *filter, q31_t input, float cutoff_freq, float sample_time, bool unit)
{
    /* 参数验证 */
    if (!isfinite(cutoff_freq) || !isfinite(sample_time) || cutoff_freq <= 0.0f || sample_time <= 0.0f) {
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
    
    /* 转换为Q31格式 */
    q31_t alpha_q31 = foc_math_q31_from_float(alpha);
    q31_t one_minus_alpha_q31 = Q31_MAX - alpha_q31;
    
    /* Q31低通滤波器递推公式：y[n] = α * x[n] + (1-α) * y[n-1] */
    q31_t term_input = foc_math_q31_mul(alpha_q31, input);
    q31_t term_state = foc_math_q31_mul(one_minus_alpha_q31, filter->state);
    q31_t output = foc_math_q31_add(term_input, term_state);
    
    /* 更新状态 */
    filter->state = output;
    
    return output;
}
