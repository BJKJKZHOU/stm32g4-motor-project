/*============================================================================
    File Name     : Positioning.c
    Description   :  
    Author        : ZHOUHENG
    Date          : 2025-11-12
    ----------------------------------------------------------------------       
    
*=============================================================================
*/


#include "Positioning.h"
#include "motor_params.h"
#include "normalization.h"
#include "FOC_math.h"
#include "Current.h"

// 条件编译：单元测试环境使用stub头文件，嵌入式环境使用真实HAL
#ifdef UNIT_TESTING
    #include "stm32_hal_stubs.h"  // 测试环境：使用stub
#else
    #include "tim.h"              // 嵌入式环境：使用真实HAL
    #include "main.h"
#endif

#include <math.h>
#include <stdint.h>
#include <string.h>


static const float IPD_PULSE_ANGLES[12] = {
    0, 180, 30, 210, 60, 240, 90, 270, 120, 300, 150, 330
};

// ============================================================================
// IPD 微秒级延时函数（基于 DWT 硬件计数器）
// ============================================================================

/**
 * @brief 初始化 DWT（Data Watchpoint and Trace）计数器
 * @note DWT 是 ARM Cortex-M 内核的调试单元，提供高精度时间戳计数器
 *       计数器频率 = CPU 频率（170MHz），精度约 5.88ns
 *        
 */
static void IPD_DWT_Init(void)
{
    // 使能 DWT 和 ITM（Instrumentation Trace Macrocell）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 复位 DWT 计数器
    DWT->CYCCNT = 0;

    // 使能 DWT 计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 微秒级精确延时函数
 * @param us 延时时间（微秒）
 * @note 基于 DWT 硬件计数器实现，精度为 1 个 CPU 时钟周期（约 5.88ns @ 170MHz）
 *       适用于 IPD 脉冲序列中的短延时（50-5000us）
 *       不会阻塞 RTOS 调度器，但会占用 CPU
 */
static void IPD_DelayUs(uint32_t us)
{
#ifdef UNIT_TESTING
    // 测试环境：不执行实际延时
    (void)us;
#else
    // 嵌入式环境：使用DWT硬件计数器
    // 确保 DWT 计数器已初始化
    static uint8_t dwt_initialized = 0;
    if (!dwt_initialized) {
        IPD_DWT_Init();
        dwt_initialized = 1;
    }

    // 计算需要的时钟周期数
    // CPU 频率 = 170MHz，1us = 170 个时钟周期
    uint32_t cycles = us * (SystemCoreClock / 1000000);

    // 记录起始时间
    uint32_t start = DWT->CYCCNT;

    // 等待指定的时钟周期数（考虑计数器溢出）
    while ((DWT->CYCCNT - start) < cycles) {
        // 空循环，等待计数器增加
        // 注意：DWT->CYCCNT 是 32 位无符号数，减法会自动处理溢出
    }
#endif
}

// ============================================================================

#define NONLINEAR_OBS_ERROR_RATIO        (0.01f)
#define NONLINEAR_OBS_ERROR_DELTA_RATIO  (0.001f)
#define NONLINEAR_OBS_CONV_WINDOW        (50U)
#define NONLINEAR_OBS_FILTER_ALPHA       (0.2f)

static bool NonlinearObs_Position_UpdateBases(NonlinearObs_Position_t *obs)
{
    const normalization_base_values_t *bases = Normalization_GetBases(obs->motor_id);
    if (bases != NULL) {
        obs->base_values = *bases;
        obs->base_valid = true;
    }
    return obs->base_valid;
}


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


// ============================================================================
// IPD 高层封装接口实现
// ============================================================================

/**
 * @brief 检测转子初始位置（简化接口）
 * @param config IPD 配置参数
 * @param angle_deg 输出：转子位置（度），范围 [0, 360)
 * @return true=成功, false=失败
 */
bool IPD_DetectRotorPosition(const IPD_Config_t *config, float *angle_deg)
{
    // 参数检查
    if (config == NULL || angle_deg == NULL) {
        return false;
    }

    // 内部分配脉冲数据数组
    IPD_Pulse_t pulses[12];

    // 执行脉冲序列
    if (!IPD_ExecutePulseSequence(pulses, config)) {
        return false;
    }

    // 计算转子位置
    float position = IPD_CalculateRotorPosition(pulses);

    // 检查结果有效性
    if (position < 0.0f) {
        return false;  // 计算失败
    }

    *angle_deg = position;
    return true;
}

/**
 * @brief 检测转子初始位置（带详细数据输出）
 * @param config IPD 配置参数
 * @param angle_deg 输出：转子位置（度），范围 [0, 360)
 * @param pulses 输出：12个脉冲的详细数据（可选，传 NULL 则不输出）
 * @return true=成功, false=失败
 */

bool IPD_DetectRotorPositionEx(const IPD_Config_t *config, float *angle_deg, IPD_Pulse_t pulses[12])
{
    // 参数检查
    if (config == NULL || angle_deg == NULL) {
        return false;
    }

    // 如果用户不需要详细数据，使用内部数组
    IPD_Pulse_t internal_pulses[12];
    IPD_Pulse_t *pulse_data = (pulses != NULL) ? pulses : internal_pulses;

    // 执行脉冲序列
    if (!IPD_ExecutePulseSequence(pulse_data, config)) {
        return false;
    }

    // 计算转子位置
    float position = IPD_CalculateRotorPosition(pulse_data);

    // 检查结果有效性
    if (position < 0.0f) {
        return false;  // 计算失败
    }

    *angle_deg = position;
    return true;
}

// ============================================================================
// IPD 底层实现
// ============================================================================

/**
 * @brief 执行完整的12脉冲IPD定位序列
 * @param pulses 脉冲数据数组（输出12个脉冲的角度和电流采样值）
 * @param config IPD配置参数（脉冲电压、持续时间、衰减时间）
 * @return true=成功, false=失败
 *
 * @note 实现原理：
 *       1. 在预定义的12个角度上依次施加d轴电压脉冲
 *       2. 每个脉冲后等待电流稳定，然后采样电流幅值
 *       3. 通过对比不同角度的电流响应，确定转子N极位置
 *
 * @note 硬件集成：
 *       - 使用 SVPWM_minmax() 生成三相PWM波形
 *       - 使用 TIM1 输出PWM控制逆变器
 *       - 使用 ADC注入模式采样三相电流
 *       - 使用 IPD_DelayUs() 实现微秒级延时
 */
bool IPD_ExecutePulseSequence(IPD_Pulse_t pulses[12], const IPD_Config_t *config)
{
    // 参数有效性检查
    if (pulses == NULL || config == NULL) {
        return false;
    }

    // 检查配置参数合理性
    if (config->pulse_voltage_pu <= 0.0f || config->pulse_voltage_pu > 0.5f) {
        return false;  // 电压幅值应在 0-0.5 标幺值范围内（安全考虑）
    }

    if (config->pulse_duration_us < 50 || config->pulse_duration_us > 1000) {
        return false;  // 脉冲持续时间应在 50-1000us 范围内
    }

    if (config->decay_time_us < 100 || config->decay_time_us > 5000) {
        return false;  // 衰减时间应在 100-5000us 范围内
    }

    // 按照预定义的12个角度顺序发送脉冲
    for (int i = 0; i < 12; i++) {
        // 1. 设置当前脉冲的角度
        pulses[i].angle_elec = IPD_PULSE_ANGLES[i];
        float angle_rad = pulses[i].angle_elec * (PI / 180.0f);  // 转换为弧度

        // 2. 计算d轴电压对应的α-β坐标系电压
        //    IPD在d轴施加电压，q轴为0
        //    使用逆Park变换：Vα = Vd*cos(θ) - Vq*sin(θ)
        //                    Vβ = Vd*sin(θ) + Vq*cos(θ)
        float sin_theta, cos_theta;
        Sine_Cosine(angle_rad, &sin_theta, &cos_theta);

        float Vd_pu = config->pulse_voltage_pu;  // d轴电压（标幺值）
        float Vq_pu = 0.0f;                      // q轴电压为0

        float V_alpha_pu, V_beta_pu;
        Inverse_Park_Transform(Vd_pu, Vq_pu, sin_theta, cos_theta,
                              &V_alpha_pu, &V_beta_pu);

        // 3. 使用SVPWM生成三相PWM占空比
        uint32_t Tcm1, Tcm2, Tcm3;
        SVPWM_minmax(V_alpha_pu, V_beta_pu, &Tcm1, &Tcm2, &Tcm3);

        // 4. 输出PWM波形（施加电压脉冲）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Tcm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Tcm2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Tcm3);

        // 5. 等待脉冲持续时间（电流建立）
        //    使用 DWT 硬件计数器实现微秒级精确延时
        IPD_DelayUs(config->pulse_duration_us);

        // 6. 采样三相电流并计算电流矢量幅值
        //    从全局变量 g_CurrentSample 读取最新的电流采样值
        float ia_pu, ib_pu, ic_pu;
        Current_GetPU(&ia_pu, &ib_pu, &ic_pu);

        // 计算电流矢量幅值（使用Clarke变换到α-β坐标系）
        float I_alpha_pu, I_beta_pu;
        if (!Clarke_Transform(ia_pu, ib_pu, &I_alpha_pu, &I_beta_pu)) {
            // Clarke变换失败，使用简化计算
            I_alpha_pu = ia_pu;
            I_beta_pu = 0.0f;
        }

        // 电流幅值 = sqrt(Iα² + Iβ²)
        float current_magnitude_pu = sqrtf(I_alpha_pu * I_alpha_pu +
                                          I_beta_pu * I_beta_pu);

        // 转换为物理值（A）并存储
        // 注意：这里需要从标幺值转换回物理值
        // 假设电流基值为额定电流，需要从归一化模块获取
        // 简化处理：直接使用标幺值作为相对幅值
        pulses[i].current_sample = current_magnitude_pu;

        // 7. 关闭PWM输出（停止施加电压）
        uint32_t Tcm_zero = ARR_PERIOD / 2;  // 50%占空比 = 零电压矢量
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Tcm_zero);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Tcm_zero);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Tcm_zero);

        // 8. 等待电流衰减（确保下一次脉冲不受影响）
        IPD_DelayUs(config->decay_time_us);
    }

    // 9. 验证生成的脉冲序列是否有效
    if (!IPD_ValidatePulseSequence(pulses)) {
        return false; // 序列验证失败
    }

    return true; // 12脉冲序列执行成功
}


/*============================================================================
 * 非线性观测器实现 - 基于论文 "Sensorless Control of Surface-Mount
 * Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer"
 *===========================================================================*/

/**
 * @brief 初始化非线性位置观测器
 * @param obs 观测器结构体指针
 * @param motor_id 电机ID
 * @param gamma 观测器增益 (> 0)
 */
void NonlinearObs_Position_Init(NonlinearObs_Position_t *obs, uint8_t motor_id, float gamma)
{
    /* 参数有效性检查 */
    if (obs == NULL || motor_id >= motors_number) {
        return;
    }
    
    obs->motor_id = motor_id;
    obs->gamma = gamma;
    obs->base_valid = false;
    memset(&obs->base_values, 0, sizeof(obs->base_values));
    if (!NonlinearObs_Position_UpdateBases(obs)) {
        Normalization_UpdateMotor(motor_id);
        NonlinearObs_Position_UpdateBases(obs);
    }
    
    /* 重置所有状态变量 */
    NonlinearObs_Position_Reset(obs);
}

/**
 * @brief 重置非线性位置观测器状态
 * @param obs 观测器结构体指针
 */
void NonlinearObs_Position_Reset(NonlinearObs_Position_t *obs)
{
    /* 参数有效性检查 */
    if (obs == NULL) {
        return;
    }
    
    obs->x_hat_alpha_pu = 0.0f;
    obs->x_hat_beta_pu = 0.0f;
    obs->theta_hat_rad = 0.0f;
    obs->theta_hat_deg = 0.0f;
    obs->is_initialized = true;
    obs->is_converged = false;
    obs->eta_norm_sq_filtered = 0.0f;
    obs->eta_norm_sq_last = 0.0f;
    obs->stable_counter = 0;
    obs->stable_required = NONLINEAR_OBS_CONV_WINDOW;
    obs->error_metrics_valid = false;
}

/**
 * @brief 更新非线性位置观测器
 * @param obs 观测器结构体指针
 * @param i_alpha_pu α轴电流 (标幺值)
 * @param i_beta_pu β轴电流 (标幺值)
 * @param v_alpha_pu α轴电压 (标幺值)
 * @param v_beta_pu β轴电压 (标幺值)
 * @param dt 采样时间 (秒)
 */
void NonlinearObs_Position_Update(NonlinearObs_Position_t *obs, 
                                  float i_alpha_pu, float i_beta_pu, 
                                  float v_alpha_pu, float v_beta_pu, 
                                  float dt)
{
    if (obs == NULL || !obs->is_initialized || dt <= 0.0f) {
        return;
    }
    
    // 检查输入参数有效性
    if (!isfinite(i_alpha_pu) || !isfinite(i_beta_pu) || 
        !isfinite(v_alpha_pu) || !isfinite(v_beta_pu)) {
        return;
    }
    
    // 获取电机参数
    const Motor_Params_t *params = &motor_params[obs->motor_id];
    
    if (params == NULL || !NonlinearObs_Position_UpdateBases(obs)) {
        return;
    }
    
    const normalization_base_values_t *bases = &obs->base_values;
    if (bases->time_base <= 0.0f) {
        return;
    }
    
    const float dt_pu = dt / bases->time_base;
    
    // 计算归一化的电机参数
    // float L_n = params->Ld / bases->inductance_base;  // 归一化电感
    // float Rs_n = params->Rs / bases->impedance_base;  // 归一化电阻
    // float psi_m_n = params->Flux / bases->flux_base;  // 归一化磁链
    
    float L_n = Normalization_ToPerUnit(obs->motor_id, NORMALIZE_INDUCTANCE, params->Ld);
    float Rs_n = Normalization_ToPerUnit(obs->motor_id, NORMALIZE_IMPEDANCE, params->Rs);
    float psi_m_n = Normalization_ToPerUnit(obs->motor_id, NORMALIZE_FLUX, params->Flux);
    // 修复1: 正确计算总磁链的期望幅值
    // 对于表面贴装永磁同步电机，总磁链幅值就是永磁体磁链
    const float expected_flux_norm_sq = psi_m_n * psi_m_n;
    
    // 计算可测量变量 y = -Rs * i_αβ + v_αβ
    float y_alpha_pu = -Rs_n * i_alpha_pu + v_alpha_pu;
    float y_beta_pu = -Rs_n * i_beta_pu + v_beta_pu;
    
    // 计算误差项 η = x̂ - L * i_αβ (这就是估计的总磁链)
    float eta_alpha_pu = obs->x_hat_alpha_pu - L_n * i_alpha_pu;
    float eta_beta_pu = obs->x_hat_beta_pu - L_n * i_beta_pu;
    
    // 计算误差项的范数平方 ||η||²
    float eta_norm_sq_pu = eta_alpha_pu * eta_alpha_pu + eta_beta_pu * eta_beta_pu;
    
    // 修复2: 保存当前误差用于收敛判定（在状态更新前）
    float current_eta_norm_sq = eta_norm_sq_pu;
    
    // 修复3: 计算误差驱动项 [ψ_m² - ||η||²]
    float error_term_pu = expected_flux_norm_sq - eta_norm_sq_pu;

    // 论文优化: 强制误差项非正以提高收敛性
    // 参考: http://cas.ensmp.fr/Publications/Publications/Papers/ObserverPermanentMagnet.pdf
    // 和 https://arxiv.org/pdf/1905.00833.pdf
    if (error_term_pu > 0.0f) {
        error_term_pu = 0.0f;
    }

    // 更新观测器状态
    // dx̂/dt = y + (γ/2) * η * [ψ_m² - ||η||²]
    float dx_hat_alpha_dt = y_alpha_pu + (obs->gamma / 2.0f) * eta_alpha_pu * error_term_pu;
    float dx_hat_beta_dt = y_beta_pu + (obs->gamma / 2.0f) * eta_beta_pu * error_term_pu;
    
    // 欧拉积分更新状态估计
    obs->x_hat_alpha_pu += dx_hat_alpha_dt * dt_pu;
    obs->x_hat_beta_pu += dx_hat_beta_dt * dt_pu;

    // 修复4: 重新计算更新后的磁链估计（用于位置计算）
    float eta_alpha_updated = obs->x_hat_alpha_pu - L_n * i_alpha_pu;
    float eta_beta_updated = obs->x_hat_beta_pu - L_n * i_beta_pu;
    
    // 修复5: 统一误差滤波逻辑（只在状态更新后滤波一次）
    float eta_norm_sq_updated = eta_alpha_updated * eta_alpha_updated + 
                               eta_beta_updated * eta_beta_updated;
    
    if (!obs->error_metrics_valid || !isfinite(obs->eta_norm_sq_filtered)) {
        obs->eta_norm_sq_filtered = eta_norm_sq_updated;
        obs->error_metrics_valid = true;
    } else {
        // 只在这里进行一次滤波
        obs->eta_norm_sq_filtered += NONLINEAR_OBS_FILTER_ALPHA * 
                                   (eta_norm_sq_updated - obs->eta_norm_sq_filtered);
    }
    
    // 修复6: 改进位置计算 - 避免观测器卡死
    // 计算状态估计的幅值（总磁链）
    float x_hat_norm = sqrtf(obs->x_hat_alpha_pu * obs->x_hat_alpha_pu +
                             obs->x_hat_beta_pu * obs->x_hat_beta_pu);

    // 论文优化: 磁链幅值保护 - 防止观测器发散
    // 参考 VESC 实现: 当磁链幅值过小时，放大状态估计
    if (x_hat_norm < (psi_m_n * 0.5f)) {
        obs->x_hat_alpha_pu *= 1.1f;
        obs->x_hat_beta_pu *= 1.1f;
        // 重新计算更新后的磁链估计
        eta_alpha_updated = obs->x_hat_alpha_pu - L_n * i_alpha_pu;
        eta_beta_updated = obs->x_hat_beta_pu - L_n * i_beta_pu;
    }

    // 计算磁链估计的幅值
    float vec_norm = sqrtf(eta_alpha_updated * eta_alpha_updated +
                          eta_beta_updated * eta_beta_updated);

    if (vec_norm < 1e-6f || !isfinite(vec_norm)) {
        // 极端情况：磁链估计过小，使用单位向量避免除零
        eta_alpha_updated = 1e-6f;
        eta_beta_updated = 0.0f;
        vec_norm = 1e-6f;
    }
    
    // 计算单位向量 [cosθ, sinθ] = η / ||η||
    float cos_theta_hat = eta_alpha_updated / vec_norm;
    float sin_theta_hat = eta_beta_updated / vec_norm;
    
    // 修复7: 确保单位向量在单位圆上（数值修正）
    float correction_factor = 1.0f / sqrtf(cos_theta_hat * cos_theta_hat + 
                                          sin_theta_hat * sin_theta_hat);
    cos_theta_hat *= correction_factor;
    sin_theta_hat *= correction_factor;
    
    // 使用 atan2 计算位置估计
    obs->theta_hat_rad = atan2f(sin_theta_hat, cos_theta_hat);
    
    // 转换为角度 (0-360度范围)
    obs->theta_hat_deg = obs->theta_hat_rad * 180.0f / PI;
    if (obs->theta_hat_deg < 0.0f) {
        obs->theta_hat_deg += 360.0f;
    }
    
    // 修复8: 简化收敛判定逻辑
    float magnitude_threshold = NONLINEAR_OBS_ERROR_RATIO * expected_flux_norm_sq;
    
    if (obs->eta_norm_sq_filtered < magnitude_threshold) {
        if (obs->stable_counter < obs->stable_required) {
            obs->stable_counter++;
        }
    } else {
        // 只有当误差显著增大时才重置计数器
        if (obs->eta_norm_sq_filtered > magnitude_threshold * 1.5f) {
            obs->stable_counter = 0;
        }
    }
    
    obs->is_converged = (obs->stable_counter >= obs->stable_required);
    
    // 修复9: 保存当前误差用于下一次的收敛判定
    obs->eta_norm_sq_last = current_eta_norm_sq;
}

/**
 * @brief 获取位置估计 (弧度)
 * @param obs 观测器结构体指针
 * @return 转子位置估计 (电角度, 弧度)
 */
float NonlinearObs_Position_GetThetaRad(NonlinearObs_Position_t *obs)
{
    if (obs == NULL || !obs->is_initialized) {
        return 0.0f;
    }
    return obs->theta_hat_rad;
}

/**
 * @brief 获取位置估计 (度)
 * @param obs 观测器结构体指针
 * @return 转子位置估计 (电角度, 度)
 */
float NonlinearObs_Position_GetThetaDeg(NonlinearObs_Position_t *obs)
{
    if (obs == NULL || !obs->is_initialized) {
        return 0.0f;
    }
    return obs->theta_hat_deg;
}

//=============================================================================
// PLL 速度观测器实现
//=============================================================================

/**
 * @brief 角度归一化到 [-π, π]
 * @param angle_rad 输入角度（弧度）
 * @return 归一化后的角度（弧度）
 */
static inline float PLL_NormalizeAngle(float angle_rad)
{
    // 使用 fmodf 将角度限制在 [-2π, 2π] 范围内
    angle_rad = fmodf(angle_rad, 2.0f * PI);

    // 将角度映射到 [-π, π]
    if (angle_rad > PI) {
        angle_rad -= 2.0f * PI;
    } else if (angle_rad < -PI) {
        angle_rad += 2.0f * PI;
    }

    return angle_rad;
}

/**
 * @brief PLL速度观测器初始化
 *
 * @param pll PLL观测器结构体指针
 * @param motor_id 电机ID（用于获取极对数）
 * @param kp 比例增益（推荐：100-500）
 * @param ki 积分增益（推荐：1000-5000）
 * @param initial_theta_rad 初始角度（电角度，弧度）
 */
void PLL_SpeedObserver_Init(PLL_SpeedObserver_t *pll,
                            uint8_t motor_id,
                            float kp,
                            float ki,
                            float initial_theta_rad)
{
    if (pll == NULL) {
        return;
    }

    // 参数检查
    if (kp < 0.0f || ki < 0.0f) {
        return;  // 增益必须为正
    }

    // 保存配置参数
    pll->motor_id = motor_id;
    pll->kp = kp;
    pll->ki = ki;

    // 初始化状态变量
    pll->theta_hat_rad = PLL_NormalizeAngle(initial_theta_rad);
    pll->omega_hat_rad_s = 0.0f;  // 初始速度为0

    // 初始化输出变量
    pll->theta_hat_deg = pll->theta_hat_rad * 180.0f / PI;
    if (pll->theta_hat_deg < 0.0f) {
        pll->theta_hat_deg += 360.0f;
    }
    pll->speed_elec_rpm = 0.0f;
    pll->speed_mech_rpm = 0.0f;

    // 设置初始化标志
    pll->is_initialized = true;
}

/**
 * @brief PLL速度观测器更新（每个控制周期调用一次）
 *
 * @param pll PLL观测器结构体指针
 * @param theta_measured_rad 测量角度（电角度，弧度）
 * @param dt 采样时间（秒）
 *
 * @note 实现原理（基于参考代码）：
 *       1. 计算位置误差：Δθ = θ_measured - θ_hat
 *       2. 归一化误差到 [-π, π]（处理角度跳变）
 *       3. 速度更新：ω_hat += Ki * Δθ * dt  （积分环节）
 *       4. 位置更新：θ_hat += (ω_hat + Kp * Δθ) * dt  （比例+积分）
 *       5. 归一化位置到 [-π, π]
 *       6. 计算输出：角度（度）、电转速（RPM）、机械转速（RPM）
 */
void PLL_SpeedObserver_Update(PLL_SpeedObserver_t *pll,
                              float theta_measured_rad,
                              float dt)
{
    if (pll == NULL || !pll->is_initialized) {
        return;
    }

    // 参数检查
    if (dt <= 0.0f || dt > 0.01f) {
        return;  // 采样时间应在合理范围内（0-10ms）
    }

    // 检查输入角度有效性
    if (!isfinite(theta_measured_rad)) {
        return;
    }

    // 步骤1: 计算位置误差 Δθ = θ_measured - θ_hat
    float delta_theta = theta_measured_rad - pll->theta_hat_rad;

    // 步骤2: 归一化误差到 [-π, π]（处理角度跳变）
    delta_theta = PLL_NormalizeAngle(delta_theta);

    // 步骤3: 速度更新（积分环节）
    // ω_hat += Ki * Δθ * dt
    pll->omega_hat_rad_s += pll->ki * delta_theta * dt;

    // 步骤4: 位置更新（比例+积分）
    // θ_hat += (ω_hat + Kp * Δθ) * dt
    pll->theta_hat_rad += (pll->omega_hat_rad_s + pll->kp * delta_theta) * dt;

    // 步骤5: 归一化位置到 [-π, π]
    pll->theta_hat_rad = PLL_NormalizeAngle(pll->theta_hat_rad);

    // 步骤6: 计算输出变量

    // 6.1 角度（度），范围 [0, 360)
    pll->theta_hat_deg = pll->theta_hat_rad * 180.0f / PI;
    if (pll->theta_hat_deg < 0.0f) {
        pll->theta_hat_deg += 360.0f;
    }

    // 6.2 电转速（RPM）
    // RPM = (rad/s) * (60 / 2π)
    pll->speed_elec_rpm = pll->omega_hat_rad_s * 60.0f / (2.0f * PI);

    // 6.3 机械转速（RPM）= 电转速 / 极对数
    // 从电机参数模块获取极对数
    float pole_pairs = motor_params[pll->motor_id].Pn;
    if (pole_pairs > 0.0f) {
        pll->speed_mech_rpm = pll->speed_elec_rpm / pole_pairs;
    } else {
        pll->speed_mech_rpm = 0.0f;  // 极对数无效，返回0
    }

    // 检查输出有效性
    if (!isfinite(pll->omega_hat_rad_s)) {
        pll->omega_hat_rad_s = 0.0f;
    }
    if (!isfinite(pll->speed_elec_rpm)) {
        pll->speed_elec_rpm = 0.0f;
    }
    if (!isfinite(pll->speed_mech_rpm)) {
        pll->speed_mech_rpm = 0.0f;
    }
}


void PLL_SpeedObserver_Reset(PLL_SpeedObserver_t *pll, float new_theta_rad)
{
    if (pll == NULL || !pll->is_initialized) {
        return;
    }

    // 重置状态变量
    pll->theta_hat_rad = PLL_NormalizeAngle(new_theta_rad);
    pll->omega_hat_rad_s = 0.0f;

    // 重置输出变量
    pll->theta_hat_deg = pll->theta_hat_rad * 180.0f / PI;
    if (pll->theta_hat_deg < 0.0f) {
        pll->theta_hat_deg += 360.0f;
    }
    pll->speed_elec_rpm = 0.0f;
    pll->speed_mech_rpm = 0.0f;
}
