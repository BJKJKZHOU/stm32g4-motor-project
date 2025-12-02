/*============================================================================
    File Name     : FOC_Loop.c
    Description   : FOC循环模块 - 包含FOC循环相关的函数
    Author        : ZHOUHENG
    Date          : 2025-11-13
    ----------------------------------------------------------------------       
    开环 给定DQ参考电压
        1、先搭建开环控制流程，检查 SVPWM 输出是否正常
    电流环
        电流采样，位置反馈，坐标变换，发波，PID控制
    速度环 
*=============================================================================
*/

#include "FOC_Loop.h"
#include "main.h"
#include "FOC_math.h"
#include "Current.h"
#include "motor_params.h"
#include "normalization.h"
#include "Positioning.h"

#include <math.h>
#include <string.h>

// 全局变量 - 用于跟踪电角度
static float g_electrical_angle = 0.0f;

// 电机参数 - 默认回退值（实际参数由 motor_params 模块提供）
#define OPEN_LOOP_DEFAULT_POLE_PAIRS 6.0f
#define OPEN_LOOP_MIN_VOLTAGE_PU   0.05f
#define OPEN_LOOP_MAX_VOLTAGE_PU   0.98f


// 用于调试 - 可以导出查看角度变化
volatile float g_debug_angle = 0.0f;
volatile float g_debug_sin = 0.0f;
volatile float g_debug_cos = 0.0f;
volatile float g_debug_frequency = 0.0f;  // 当前电频率

volatile uint8_t sector = 0;

/**
 * @brief 获取当前生效的电机ID（未激活则默认选择0号电机）
 */
static uint8_t FOC_GetEffectiveMotorId(void)
{
    uint8_t active_motor = MotorParams_GetActiveMotor();
    if (active_motor >= motors_number) {
        active_motor = MOTOR_0;
    }
    return active_motor;
}

/**
 * @brief 获取电机极对数（若未配置则回退到默认值）
 */
static float FOC_GetPolePairs(uint8_t motor_id)
{
    const Motor_Params_t *params = &motor_params[motor_id];
    return (params->Pn > 0.0f) ? params->Pn : OPEN_LOOP_DEFAULT_POLE_PAIRS;
}

/**
 * @brief 简单V/f曲线：根据电角速度计算q轴电压幅值（标幺）
 */
static float FOC_ComputeVfVoltagePu(float electrical_rad_s, uint8_t motor_id)
{
    const Motor_Params_t *params = &motor_params[motor_id];

    float omega_base = 0.0f;
    if (params->RPM_rated > 0.0f && params->Pn > 0.0f) {
        omega_base = (params->RPM_rated * 2.0f * PI / 60.0f) * params->Pn;
    }

    if (omega_base <= 0.0f) {
        return OPEN_LOOP_MIN_VOLTAGE_PU;
    }

    float norm_freq = fabsf(electrical_rad_s) / omega_base;
    if (norm_freq > 1.0f) {
        norm_freq = 1.0f;
    }

    float voltage_pu = OPEN_LOOP_MIN_VOLTAGE_PU +
                       norm_freq * (OPEN_LOOP_MAX_VOLTAGE_PU - OPEN_LOOP_MIN_VOLTAGE_PU);

    if (voltage_pu > OPEN_LOOP_MAX_VOLTAGE_PU) {
        voltage_pu = OPEN_LOOP_MAX_VOLTAGE_PU;
    }

    return voltage_pu;
}

void FOC_OpenLoopTest(float speed_rpm, uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3)
{

    // 参数检查
    if (Tcm1 == NULL || Tcm2 == NULL || Tcm3 == NULL) {
        return;
    }

    const uint8_t motor_id = FOC_GetEffectiveMotorId();
    const float pole_pairs = FOC_GetPolePairs(motor_id);
    
    // 1. 将机械转速(rpm)转换为电角速度(rad/s)
    const float rpm_to_rad_per_sec = 2.0f * PI / 60.0f;
    const float mechanical_rad_s = speed_rpm * rpm_to_rad_per_sec;
    const float electrical_rad_s = mechanical_rad_s * pole_pairs;

    // 2. 根据电角速度计算角度增量
    float angle_increment = electrical_rad_s * TPWM_PERIOD;

    // 3. 更新电角度（保持在[-π, π]范围内）
    g_electrical_angle += angle_increment;

    g_electrical_angle = FOC_WrapAngle(g_electrical_angle);
    
    // 4. 计算正弦和余弦值（使用CORDIC硬件加速）
    float sin_theta, cos_theta;
    Sine_Cosine(g_electrical_angle, &sin_theta, &cos_theta);
    
    // 5. 生成三相120度正弦波（标幺值范围[-1, 1]）
    // 使用 CORDIC 计算 V 相的 sin/cos
    float sin_theta_v, cos_theta_v;
    Sine_Cosine(g_electrical_angle - 2.0f * PI / 3.0f, &sin_theta_v, &cos_theta_v);

    float U_phase = sin_theta;      // U相：sin(θ)
    float V_phase = sin_theta_v;    // V相：sin(θ - 120°)
    //float W_phase = sinf(g_electrical_angle + 2.0f * PI / 3.0f); // W相：sin(θ + 120°)
    
    // 6. Clark变换：将三相电流转换为αβ坐标系
    float I_alpha, I_beta;
    Clarke_Transform(U_phase, V_phase, &I_alpha, &I_beta);
    
    // 7. Park变换：将αβ坐标系转换为dq坐标系
    float I_d, I_q;
    Park_Transform(I_alpha, I_beta, sin_theta, cos_theta, &I_d, &I_q);
    
    // 8. 设置开环电压矢量
    // 在开环测试中，设置U_d=0，U_q按照V/f曲线随速度提升
    const float U_d_pu = 0.0f;  // d轴电压设为0
    const float U_q_pu = FOC_ComputeVfVoltagePu(electrical_rad_s, motor_id);
    
    // 9. 逆Park变换：将dq电压转换为αβ电压
    float U_alpha_pu, U_beta_pu;
    Inverse_Park_Transform(U_d_pu, U_q_pu, sin_theta, cos_theta, &U_alpha_pu, &U_beta_pu);
    
    // 10. SVPWM调制：将αβ电压转换为PWM占空比
    //SVPWM_minmax(U_alpha_pu, U_beta_pu, Tcm1, Tcm2, Tcm3);
    SVPWM_SectorBased(U_alpha_pu, U_beta_pu, Tcm1, Tcm2, Tcm3, (uint8_t *)&sector);
    // 调试信息更新
    g_debug_angle = g_electrical_angle;
    g_debug_sin = sin_theta;
    g_debug_cos = cos_theta;
    g_debug_frequency = electrical_rad_s;
}

/* ======================================================
 * 电流环控制函数实现
 * ======================================================
 */

/**
 * @brief 电流环初始化
 */
void CurrentLoop_Init(CurrentLoop_t *loop, uint8_t motor_id, float dt,
                     float kp_d, float ki_d, float kp_q, float ki_q,
                     float observer_gamma)
{
    // 参数检查
    if (loop == NULL) {
        return;
    }

    // 清零结构体
    memset(loop, 0, sizeof(CurrentLoop_t));

    // 设置基本参数
    loop->motor_id = motor_id;
    loop->dt = dt;

    // 初始化d轴PID控制器
    loop->pid_d_params.kp = kp_d;
    loop->pid_d_params.ki = ki_d;
    loop->pid_d_params.kd = 0.0f;  // 电流环不使用微分项
    loop->pid_d_params.Kfr_speed = 0.0f;  // 电流环不使用前馈
    loop->pid_d_params.integral_limit = 1.0f;  // 积分限幅（标幺值）
    loop->pid_d_params.output_limit = 1.0f;    // 输出限幅（标幺值）

    loop->pid_d_state.integral = 0.0f;
    loop->pid_d_state.prev_error = 0.0f;

    // 初始化q轴PID控制器
    loop->pid_q_params.kp = kp_q;
    loop->pid_q_params.ki = ki_q;
    loop->pid_q_params.kd = 0.0f;
    loop->pid_q_params.Kfr_speed = 0.0f;
    loop->pid_q_params.integral_limit = 1.0f;
    loop->pid_q_params.output_limit = 1.0f;

    loop->pid_q_state.integral = 0.0f;
    loop->pid_q_state.prev_error = 0.0f;

    // 初始化非线性磁链观测器
    NonlinearObs_Position_Init(&loop->position_observer, motor_id, observer_gamma);

    // 初始化PWM输出为中点值（50%占空比）
    loop->last_Tcm1 = ARR_PERIOD / 2;
    loop->last_Tcm2 = ARR_PERIOD / 2;
    loop->last_Tcm3 = ARR_PERIOD / 2;

    // 设置标志
    loop->is_initialized = true;
    loop->is_running = false;
}

/**
 * @brief 电流环复位
 */
void CurrentLoop_Reset(CurrentLoop_t *loop)
{
    if (loop == NULL || !loop->is_initialized) {
        return;
    }

    // 复位PID状态
    loop->pid_d_state.integral = 0.0f;
    loop->pid_d_state.prev_error = 0.0f;
    loop->pid_q_state.integral = 0.0f;
    loop->pid_q_state.prev_error = 0.0f;

    // 复位观测器
    NonlinearObs_Position_Reset(&loop->position_observer);

    // 复位PWM输出为中点值
    loop->last_Tcm1 = ARR_PERIOD / 2;
    loop->last_Tcm2 = ARR_PERIOD / 2;
    loop->last_Tcm3 = ARR_PERIOD / 2;

    // 清零监控变量
    loop->id_setpoint = 0.0f;
    loop->iq_setpoint = 0.0f;
    loop->id_feedback = 0.0f;
    loop->iq_feedback = 0.0f;
    loop->ud_output = 0.0f;
    loop->uq_output = 0.0f;
    loop->theta_elec = 0.0f;
    loop->omega_elec = 0.0f;

    loop->is_running = false;
}

/**
 * @brief 电流环主控制函数
 */
bool CurrentLoop_Run(CurrentLoop_t *loop, float id_ref, float iq_ref,
                    uint32_t *Tcm1, uint32_t *Tcm2, uint32_t *Tcm3)
{
    // 参数检查
    if (loop == NULL || !loop->is_initialized) {
        return false;
    }
    if (Tcm1 == NULL || Tcm2 == NULL || Tcm3 == NULL) {
        return false;
    }

    // 获取电机参数
    extern Motor_Params_t motor_params[];
    const Motor_Params_t *motor = &motor_params[loop->motor_id];
    // const normalization_base_values_t *bases = Normalization_GetBases(loop->motor_id);  // 未使用，已注释

    // ========================================================================
    // 步骤1：ADC采集三相电流
    // ========================================================================
    float ia_pu, ib_pu, ic_pu;
    Current_GetPU(&ia_pu, &ib_pu, &ic_pu);

    // ========================================================================
    // 步骤2：Clarke变换：abc → αβ
    // ========================================================================
    float i_alpha_pu, i_beta_pu;
    if (!Clarke_Transform(ia_pu, ib_pu, &i_alpha_pu, &i_beta_pu)) {
        return false;
    }

    // ========================================================================
    // 步骤3：计算逆变器输出电压（αβ坐标系）
    // ========================================================================
    // 使用上一次的PWM输出计算实际电压
    float Ua, Ub, Uc;
    PWM_To_Voltage_ABC(loop->last_Tcm1, loop->last_Tcm2, loop->last_Tcm3,
                      motor->V_DC, &Ua, &Ub, &Uc);

    // Clarke变换：abc电压 → αβ电压
    float u_alpha, u_beta;
    Clarke_Transform_Physical(Ua, Ub, &u_alpha, &u_beta);

    // 转换为标幺值
    float u_alpha_pu = Normalization_ToPerUnit(loop->motor_id, NORMALIZE_VOLTAGE, u_alpha);
    float u_beta_pu = Normalization_ToPerUnit(loop->motor_id, NORMALIZE_VOLTAGE, u_beta);

    // ========================================================================
    // 步骤4：非线性磁链观测器估计电角度
    // ========================================================================
    NonlinearObs_Position_Update(&loop->position_observer,
                                i_alpha_pu, i_beta_pu,
                                u_alpha_pu, u_beta_pu,
                                loop->dt);

    // 获取估计的电角度
    float theta_elec = NonlinearObs_Position_GetThetaRad(&loop->position_observer);
    loop->theta_elec = theta_elec;

    // ========================================================================
    // 步骤5：计算sin和cos值，Park变换：αβ → dq（电流反馈）
    // ========================================================================
    float sin_theta, cos_theta;
    Sine_Cosine(theta_elec, &sin_theta, &cos_theta);

    float id_feedback, iq_feedback;
    Park_Transform(i_alpha_pu, i_beta_pu, sin_theta, cos_theta, &id_feedback, &iq_feedback);

    // 保存反馈值
    loop->id_feedback = id_feedback;
    loop->iq_feedback = iq_feedback;
    loop->id_setpoint = id_ref;
    loop->iq_setpoint = iq_ref;

    // ========================================================================
    // 步骤6：PID控制器计算dq轴电压（含解耦补偿）
    // ========================================================================
    // d轴PID控制
    float ud_pid = PID_Controller(id_ref, id_feedback, loop->dt,
                                  &loop->pid_d_params, &loop->pid_d_state);

    // q轴PID控制
    float uq_pid = PID_Controller(iq_ref, iq_feedback, loop->dt,
                                  &loop->pid_q_params, &loop->pid_q_state);

    // 解耦补偿（默认开启）
    // 根据PID_controller.h中的说明：
    // q轴解耦: ws * (Ld * id_feedback + Flux)
    // d轴解耦: -ws * (Lq * iq_feedback)
    // 这里需要电角速度ws，可以通过观测器或者差分计算得到
    // 简化处理：暂时不加解耦，后续可以根据需要添加

    // 最终输出电压
    float ud_output = ud_pid;
    float uq_output = uq_pid;

    loop->ud_output = ud_output;
    loop->uq_output = uq_output;

    // ========================================================================
    // 步骤7：逆Park变换：dq → αβ（电压输出）
    // ========================================================================
    float u_alpha_out_pu, u_beta_out_pu;
    Inverse_Park_Transform(ud_output, uq_output, sin_theta, cos_theta,
                          &u_alpha_out_pu, &u_beta_out_pu);

    // ========================================================================
    // 步骤8：SVPWM生成PWM占空比
    // ========================================================================
    uint8_t sector;
    SVPWM_SectorBased(u_alpha_out_pu, u_beta_out_pu, Tcm1, Tcm2, Tcm3, &sector);

    // 保存本次PWM输出，用于下次计算逆变器电压
    loop->last_Tcm1 = *Tcm1;
    loop->last_Tcm2 = *Tcm2;
    loop->last_Tcm3 = *Tcm3;

    // 设置运行标志
    loop->is_running = true;

    return true;
}

/* ======================================================
 * 速度环控制函数实现
 * ======================================================
 */

/**
 * @brief 速度环初始化
 */
void SpeedLoop_Init(SpeedLoop_t *loop, uint8_t motor_id, float dt,
                   float kp, float ki,
                   float pll_kp, float pll_ki,
                   float initial_theta_rad)
{
    // 参数检查
    if (loop == NULL) {
        return;
    }

    // 清零结构体
    memset(loop, 0, sizeof(SpeedLoop_t));

    // 设置基本参数
    loop->motor_id = motor_id;
    loop->dt = dt;

    // 初始化速度环PID控制器
    loop->pid_params.kp = kp;
    loop->pid_params.ki = ki;
    loop->pid_params.kd = 0.0f;  // 速度环不使用微分项
    loop->pid_params.Kfr_speed = 0.0f;  // 速度环不使用前馈（标准PI模式）
    loop->pid_params.integral_limit = 1.0f;  // 积分限幅（标幺值）
    loop->pid_params.output_limit = 1.0f;    // 输出限幅（标幺值）

    loop->pid_state.integral = 0.0f;
    loop->pid_state.prev_error = 0.0f;

    // 初始化PLL速度观测器
    PLL_SpeedObserver_Init(&loop->speed_observer, motor_id, pll_kp, pll_ki, initial_theta_rad);

    // 获取电机参数并计算速度限制
    extern Motor_LimitParams_t motor_limit_params[];
    extern Motor_Params_t motor_params[];

    float speed_limit_rpm = motor_limit_params[motor_id].speed_limit_actual;
    float pole_pairs = motor_params[motor_id].Pn;

    // 转换为电角速度（rad/s）
    // omega_elec = RPM * (2π/60) * Pn
    loop->omega_limit = speed_limit_rpm * (2.0f * PI / 60.0f) * pole_pairs;

    // 转换为标幺值
    loop->omega_limit_pu = Normalization_ToPerUnit(motor_id,
                                                   NORMALIZE_SPEED_ELECTRICAL,
                                                   loop->omega_limit);

    // 获取电流限制（标幺值）
    float i_limit_actual = motor_limit_params[motor_id].I_limit_actual;
    loop->iq_limit_pu = Normalization_ToPerUnit(motor_id,
                                                NORMALIZE_CURRENT,
                                                i_limit_actual);

    // 设置标志
    loop->is_initialized = true;
    loop->is_running = false;
}

/**
 * @brief 速度环主控制函数
 */
bool SpeedLoop_Run(SpeedLoop_t *loop, float omega_ref,
                  float theta_measured_rad,
                  float *id_ref, float *iq_ref)
{
    // 参数检查
    if (loop == NULL || !loop->is_initialized) {
        return false;
    }
    if (id_ref == NULL || iq_ref == NULL) {
        return false;
    }

    // ========================================================================
    // 步骤1：PLL观测器更新，提取速度反馈（电角速度，rad/s）
    // ========================================================================
    PLL_SpeedObserver_Update(&loop->speed_observer, theta_measured_rad, loop->dt);

    // 获取电角速度反馈（rad/s）
    loop->omega_feedback = loop->speed_observer.omega_hat_rad_s;

    // ========================================================================
    // 步骤2：速度限幅保护（电角速度）
    // ========================================================================
    float omega_ref_limited = omega_ref;
    if (omega_ref_limited > loop->omega_limit) {
        omega_ref_limited = loop->omega_limit;
    } else if (omega_ref_limited < -loop->omega_limit) {
        omega_ref_limited = -loop->omega_limit;
    }

    // 保存设定值
    loop->omega_setpoint = omega_ref_limited;

    // ========================================================================
    // 步骤3：转换为标幺值
    // ========================================================================
    loop->omega_setpoint_pu = Normalization_ToPerUnit(loop->motor_id,
                                                      NORMALIZE_SPEED_ELECTRICAL,
                                                      omega_ref_limited);

    loop->omega_feedback_pu = Normalization_ToPerUnit(loop->motor_id,
                                                      NORMALIZE_SPEED_ELECTRICAL,
                                                      loop->omega_feedback);

    // ========================================================================
    // 步骤4：PI控制器计算q轴电流设定值（标幺值域）
    // ========================================================================
    float iq_output_pu = PID_Controller(loop->omega_setpoint_pu,       // 设定值（标幺）
                                        loop->omega_feedback_pu,        // 反馈值（标幺）
                                        loop->dt,                       // 控制周期
                                        &loop->pid_params,              // PID参数
                                        &loop->pid_state);              // PID状态

    // ========================================================================
    // 步骤5：电流限幅保护
    // ========================================================================
    if (iq_output_pu > loop->iq_limit_pu) {
        iq_output_pu = loop->iq_limit_pu;
    } else if (iq_output_pu < -loop->iq_limit_pu) {
        iq_output_pu = -loop->iq_limit_pu;
    }

    loop->iq_output = iq_output_pu;

    // ========================================================================
    // 步骤6：d轴电流设为0（表贴式永磁同步电机）
    // ========================================================================
    *id_ref = 0.0f;  // SPMSM: id=0，最大转矩电流比控制
    *iq_ref = iq_output_pu;

    // 设置运行标志
    loop->is_running = true;

    return true;
}

/**
 * @brief 速度环复位
 */
void SpeedLoop_Reset(SpeedLoop_t *loop, float new_theta_rad)
{
    if (loop == NULL || !loop->is_initialized) {
        return;
    }

    // 复位PID状态
    loop->pid_state.integral = 0.0f;
    loop->pid_state.prev_error = 0.0f;

    // 复位PLL观测器
    PLL_SpeedObserver_Reset(&loop->speed_observer, new_theta_rad);

    // 清零监控变量
    loop->omega_setpoint = 0.0f;
    loop->omega_feedback = 0.0f;
    loop->omega_setpoint_pu = 0.0f;
    loop->omega_feedback_pu = 0.0f;
    loop->iq_output = 0.0f;

    loop->is_running = false;
}

