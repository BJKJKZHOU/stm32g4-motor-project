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

#ifndef PI
#define PI 3.14159265358979323846f
#endif


#define INV_SQRT3_F           (0.5773502691896258f)
#define SQRT3_OVER_TWO_F      (0.8660254037844386f)
#define TWO_PI_F              (2.0f * PI)


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

    *U_alpha_pu = U_d_pu * cos_theta - U_q_pu * sin_theta;
    *U_beta_pu  = U_d_pu * sin_theta + U_q_pu * cos_theta;
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
    arm_sin_cos_f32(wrapped, sin_theta_e, cos_theta_e);
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

    *I_d = I_alpha_pu * cos_theta + I_beta_pu * sin_theta;
    *I_q = -I_alpha_pu * sin_theta + I_beta_pu * cos_theta;
}
