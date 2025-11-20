/*============================================================================
    File Name     : FOC_math.c
    Description   : FOC数学计算模块 - 包含FOC相关的数学计算函数
    Author        : ZHOUHENG
    Date          : 2025-11-04
    ----------------------------------------------------------------------       

*=============================================================================
*/

#include "FOC_math.h"
#include "main.h"
#include "arm_math.h"

#include <math.h>
#include <limits.h>

#include "cordic.h"


#define INV_SQRT3_F           (0.5773502691896258f)      // 1/√3
#define SQRT3_OVER_TWO_F      (0.8660254037844386f)      // √3/2
#define TWO_BY_SQRT3_F        (1.1547005383792515f)      // 2/√3
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

    /* 逆Park变换矩阵：[U_alpha] = [cosθ -sinθ][U_d] */
    /*                 [U_beta ]   [sinθ  cosθ][U_q] */
    
    /* 正弦余弦值来自CORDIC硬件加速，这里直接进行矩阵乘法 */
    *U_alpha_pu = U_d_pu * cos_theta - U_q_pu * sin_theta;
    *U_beta_pu  = U_d_pu * sin_theta + U_q_pu * cos_theta;
}

// 谐波注入SPWM实现SVPWM (Min-Max方法生成马鞍波)
// 输入[-1~+1]，输出[0~ARR_PERIOD]
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

    // 输入范围说明：
    // U_alpha_pu, U_beta_pu 是标幺值，范围 [-1, 1]
    // 其中 ±1.0 对应电压基值 V_base = V_DC/√3
    // SVPWM 线性调制区：矢量幅值 ≤ 1/√3 ≈ 0.577
    // 过调制区：0.577 < 幅值 ≤ 1.0
    // 逆Clarke变换：将αβ坐标系的电压转换为三相电压（标幺值）
    // 标准逆Clarke变换公式（等幅值变换）：
    // U_a = U_alpha
    // U_b = -0.5 * U_alpha + (√3/2) * U_beta
    // U_c = -0.5 * U_alpha - (√3/2) * U_beta
    // 输出范围：[-1, 1] 标幺值
    const float U_a = U_alpha_pu;
    const float U_b = (-0.5f * U_alpha_pu) + (SQRT3_OVER_TWO_F * U_beta_pu);
    const float U_c = (-0.5f * U_alpha_pu) - (SQRT3_OVER_TWO_F * U_beta_pu);

    // 零序分量 = -(max + min) / 2
    // 这会产生三次谐波注入，形成马鞍波形状，提升电压利用率约15%
    const float U_zero = foc_math_zero_sequence(U_a, U_b, U_c);

    // 将零序分量注入到三相电压中（标幺值）
    // 注入后的范围：仍为 [-1, 1] 左右，但峰值被削平
    const float Ua_injected = U_a + U_zero;
    const float Ub_injected = U_b + U_zero;
    const float Uc_injected = U_c + U_zero;

    // 将标幺电压 [-1, 1] 转换为占空比 [0, 1]
    // 映射关系：
    //   标幺值 -1.0 → 占空比 0.0 (0%)
    //   标幺值  0.0 → 占空比 0.5 (50%, 中点)
    //   标幺值 +1.0 → 占空比 1.0 (100%)
    // 公式：duty = (voltage_pu + 1.0) / 2.0 = voltage_pu * 0.5 + 0.5
    const float Tcm1_pu = foc_math_saturate(Ua_injected * 0.5f + 0.5f, 0.0f, 1.0f);
    const float Tcm2_pu = foc_math_saturate(Ub_injected * 0.5f + 0.5f, 0.0f, 1.0f);
    const float Tcm3_pu = foc_math_saturate(Uc_injected * 0.5f + 0.5f, 0.0f, 1.0f);

    // 转换为定时器计数值
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

bool Clarke_Transform_Physical(float a, float b, float *alpha, float *beta)
{
    if (alpha == NULL || beta == NULL) {
        return false;
    }
    
    *alpha = a;
    *beta  = (a + 2.0f * b) * INV_SQRT3_F;
    
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
        // arm_sin_cos_f32需要角度输入（度），需要将弧度转换为度
        float theta_degrees = wrapped * (180.0f / PI);
        arm_sin_cos_f32(theta_degrees, sin_theta_e, cos_theta_e);
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
        // arm_sin_cos_f32需要角度输入（度），需要将弧度转换为度
        float theta_degrees = wrapped * (180.0f / PI);
        arm_sin_cos_f32(theta_degrees, sin_theta_e, cos_theta_e);
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

// 输入：Valpha, Vbeta 标幺值 [-1, 1]，与SVPWM_minmax使用相同的电压定义
// 输出：Tcm1, Tcm2, Tcm3 定时器比较值 [0, ARR_PERIOD]，sector 扇区编号 [1-6]
void SVPWM_SectorBased(float Valpha, float Vbeta, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3, uint8_t *sector)
{
    // 参数定义
    const float sqrt3 = 1.73205080757f; // √3

    // 扇区判断变量
    float Vref1, Vref2, Vref3;

    // 初始化扇区输出
    *sector = 0;
    
    // 扇区计算
    Vref1 = Vbeta;
    Vref2 = (sqrt3 * Valpha - Vbeta) / 2.0f;
    Vref3 = (-sqrt3 * Valpha - Vbeta) / 2.0f;
    
    uint8_t n = 0;
    if (Vref1 > 0) n |= 1; // A
    if (Vref2 > 0) n |= 2; // B
    if (Vref3 > 0) n |= 4; // C

    // 根据N值查表确定扇区
    // N=3 -> 扇区1 (0-60)
    // N=1 -> 扇区2 (60-120)
    // N=5 -> 扇区3 (120-180)
    // N=4 -> 扇区4 (180-240)
    // N=6 -> 扇区5 (240-300)
    // N=2 -> 扇区6 (300-360)
    switch (n) {
        case 3: *sector = 1; break;
        case 1: *sector = 2; break;
        case 5: *sector = 3; break;
        case 4: *sector = 4; break;
        case 6: *sector = 5; break;
        case 2: *sector = 6; break;
        default: *sector = 0; break; // 错误或边界情况
    }
    
    // 扇区内合成矢量作用时间计算
    // 使用与 SVPWM_minmax 相同的标幺值定义
    // 通过逆Clarke变换计算三相电压，然后提取 T1, T2

    // 幅值缩放：将输入电压缩放到满幅值
    // 缩放因子 = 2/√3 ≈ 1.1547，使得输入 1.0 对应满幅值输出
    const float scale_factor = TWO_BY_SQRT3_F;  // 2/√3
    const float Valpha_scaled = Valpha * scale_factor;
    const float Vbeta_scaled = Vbeta * scale_factor;

    // 逆Clarke变换：αβ → abc
    const float Ua_pu = Valpha_scaled;
    const float Ub_pu = -0.5f * Valpha_scaled + (sqrt3 / 2.0f) * Vbeta_scaled;
    const float Uc_pu = -0.5f * Valpha_scaled - (sqrt3 / 2.0f) * Vbeta_scaled;

    // 计算零序分量
    const float U_zero_pu = -0.5f * (fmaxf(fmaxf(Ua_pu, Ub_pu), Uc_pu) +
                                      fminf(fminf(Ua_pu, Ub_pu), Uc_pu));

    // 注入零序分量
    const float Ua_inj = Ua_pu + U_zero_pu;
    const float Ub_inj = Ub_pu + U_zero_pu;
    const float Uc_inj = Uc_pu + U_zero_pu;

    // 转换为占空比 [0, 1]
    const float duty_a = Ua_inj * 0.5f + 0.5f;
    const float duty_b = Ub_inj * 0.5f + 0.5f;
    const float duty_c = Uc_inj * 0.5f + 0.5f;

    // 转换为时间计数值
    *Tcm1 = foc_math_pu_to_ticks(duty_a);
    *Tcm2 = foc_math_pu_to_ticks(duty_b);
    *Tcm3 = foc_math_pu_to_ticks(duty_c);
}

/**
 * @brief 根据PWM占空比和直流母线电压计算逆变器输出三相电压（实际值）
 *
 * 此函数模拟逆变器的实际输出，用于观测器和控制算法中需要电压反馈的场景。
 *
 * 计算原理：
 * 1. PWM占空比决定了每相桥臂上管的导通时间比例
 * 2. 在中心对齐PWM模式下，相电压 = (占空比 - 0.5) * V_DC
 * 3. 占空比0.5对应零电压，占空比1.0对应+V_DC/2，占空比0对应-V_DC/2
 */
void PWM_To_Voltage_ABC(uint32_t Tcm1, uint32_t Tcm2, uint32_t Tcm3, float V_DC,
                       float *Ua, float *Ub, float *Uc)
{
    // 参数检查
    if (Ua == NULL || Ub == NULL || Uc == NULL) {
        return;
    }

    // 1. 将PWM计数值转换为占空比 [0, 1]
    const float duty_a = (float)Tcm1 / (float)ARR_PERIOD;
    const float duty_b = (float)Tcm2 / (float)ARR_PERIOD;
    const float duty_c = (float)Tcm3 / (float)ARR_PERIOD;

    // 2. 将占空比转换为相电压（实际值）
    // 在中心对齐PWM模式下：
    // - 占空比0.5 → 零电压
    // - 占空比1.0 → +V_DC/2
    // - 占空比0.0 → -V_DC/2
    *Ua = (duty_a - 0.5f) * V_DC;
    *Ub = (duty_b - 0.5f) * V_DC;
    *Uc = (duty_c - 0.5f) * V_DC;
}
