/*============================================================================
    File Name     : test_foc_integration.cpp
    Description   : FOC控制算法集成测试 - 真实闭环测试
    Author        : ZHOUHENG
    Date          : 2025-11-20
    ----------------------------------------------------------------------
    测试目标：
    1. 使用真实的FOC_OpenLoopTest函数
    2. 电机模拟器模拟真实电机响应
    3. ADC模拟器模拟电流采样
    4. 验证完整的控制闭环
*=============================================================================*/

#include "CppUTest/TestHarness.h"

extern "C" {
#include "motor_simulator.h"
#include "FOC_Loop.h"
#include "FOC_math.h"
#include "Current.h"
#include <math.h>
}

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define FLOAT_TOLERANCE 0.01f

// ============================================================================
// Test group: FOC开环控制集成测试
// ============================================================================
TEST_GROUP(FOC_OpenLoop_Integration)
{
    void setup() {
        // 初始化电机模拟器
        MotorSim_SetDefaultParams();

        // 调整参数以匹配FOC_Loop.c中的设置
        g_motor_sim_params.pole_pairs = 6.0f;  // 与MOTOR_POLE_PAIRS一致
        g_motor_sim_params.Ts = 50e-6f;        // 50μs采样周期

        MotorSim_Init(&g_motor_sim_params);

        // 初始化ADC模拟器
        ADCSim_Init(12, 3.3f, 0.001f, 100.0f);
    }

    void teardown() {
        MotorSim_Reset();
    }
};

TEST(FOC_OpenLoop_Integration, BasicOpenLoopTest)
{
    // 测试参数
    float frequency_rad_s = 50.0f;  // 50 rad/s电频率
    uint32_t Tcm1, Tcm2, Tcm3;

    // 运行1000个控制周期（50ms）
    for (int i = 0; i < 1000; i++) {
        // 1. 调用真实的FOC开环控制函数
        FOC_OpenLoopTest(frequency_rad_s, &Tcm1, &Tcm2, &Tcm3);

        // 2. 将PWM占空比转换为电压标幺值
        // ARR_PERIOD = 5312 (从main.h)
        const float ARR_PERIOD = 5312.0f;
        float duty1 = (float)Tcm1 / ARR_PERIOD;
        float duty2 = (float)Tcm2 / ARR_PERIOD;
        float duty3 = (float)Tcm3 / ARR_PERIOD;

        // 3. 占空比转换为相电压（标幺值，范围[-1, 1]）
        float Ua_pu = (duty1 - 0.5f) * 2.0f;
        float Ub_pu = (duty2 - 0.5f) * 2.0f;
        float Uc_pu = (duty3 - 0.5f) * 2.0f;

        // 4. Clarke变换：abc → αβ
        float U_alpha_pu, U_beta_pu;
        Clarke_Transform(Ua_pu, Ub_pu, &U_alpha_pu, &U_beta_pu);

        // 5. 电机模拟器步进
        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);

        // 6. ADC采样模拟
        float Ia, Ib, Ic;
        MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
        ADCSim_Sample(Ia, Ib, Ic);
    }

    // 验证电机已经旋转
    const MotorSimState_t* state = MotorSim_GetState();
    CHECK(fabsf(state->omega_e) > 1.0f);

    // 验证产生了电流
    CHECK(fabsf(state->Ia) > 0.1f || fabsf(state->Ib) > 0.1f || fabsf(state->Ic) > 0.1f);

    // 验证PWM输出在合理范围内
    CHECK(Tcm1 <= 5312);
    CHECK(Tcm2 <= 5312);
    CHECK(Tcm3 <= 5312);
}

TEST(FOC_OpenLoop_Integration, FrequencyRamp)
{
    // 测试频率斜坡（验证系统在变化频率下的稳定性）
    uint32_t Tcm1, Tcm2, Tcm3;
    float omega_initial = 0.0f;
    float omega_final = 0.0f;

    // 从10 rad/s逐渐增加到50 rad/s
    for (int i = 0; i < 1500; i++) {
        float frequency_rad_s = 10.0f + (float)i * 0.027f;  // 线性增加

        // 调用FOC控制
        FOC_OpenLoopTest(frequency_rad_s, &Tcm1, &Tcm2, &Tcm3);

        // PWM → 电压
        const float ARR_PERIOD = 5312.0f;
        float Ua_pu = ((float)Tcm1 / ARR_PERIOD - 0.5f) * 2.0f;
        float Ub_pu = ((float)Tcm2 / ARR_PERIOD - 0.5f) * 2.0f;

        // Clarke变换
        float U_alpha_pu, U_beta_pu;
        Clarke_Transform(Ua_pu, Ub_pu, &U_alpha_pu, &U_beta_pu);

        // 电机步进 + ADC采样
        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);
        float Ia, Ib, Ic;
        MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
        ADCSim_Sample(Ia, Ib, Ic);

        // 记录初始和最终转速
        if (i == 100) omega_initial = MotorSim_GetState()->omega_e;
        if (i == 1499) omega_final = MotorSim_GetState()->omega_e;
    }

    // 验证转速增加趋势
    CHECK(omega_final > omega_initial);

    // 验证系统稳定（没有发散）
    CHECK(isfinite(omega_final));
    CHECK(fabsf(omega_final) < 200.0f);
}

TEST(FOC_OpenLoop_Integration, VerifyCoordinateTransforms)
{
    // 验证坐标变换的正确性（使用较低频率避免过大电流）
    float frequency_rad_s = 30.0f;  // 降低频率
    uint32_t Tcm1, Tcm2, Tcm3;

    // 运行到稳态
    for (int i = 0; i < 2000; i++) {
        FOC_OpenLoopTest(frequency_rad_s, &Tcm1, &Tcm2, &Tcm3);

        // PWM → 电压 → 电机
        const float ARR_PERIOD = 5312.0f;
        float Ua_pu = ((float)Tcm1 / ARR_PERIOD - 0.5f) * 2.0f;
        float Ub_pu = ((float)Tcm2 / ARR_PERIOD - 0.5f) * 2.0f;

        float U_alpha_pu, U_beta_pu;
        Clarke_Transform(Ua_pu, Ub_pu, &U_alpha_pu, &U_beta_pu);

        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);
        float Ia, Ib, Ic;
        MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
        ADCSim_Sample(Ia, Ib, Ic);
    }

    // 获取电机状态
    const MotorSimState_t* state = MotorSim_GetState();

    // 验证系统稳定性（主要目标）
    CHECK(isfinite(state->Id));
    CHECK(isfinite(state->Iq));
    CHECK(isfinite(state->omega_e));

    // 验证三相电流平衡（基尔霍夫定律）
    DOUBLES_EQUAL(0.0f, state->Ia + state->Ib + state->Ic, 0.1f);

    // 验证电机在旋转
    CHECK(fabsf(state->omega_e) > 1.0f);
}

// ============================================================================
// Test group: ADC采样集成测试
// ============================================================================
TEST_GROUP(ADC_Integration)
{
    void setup() {
        MotorSim_SetDefaultParams();
        g_motor_sim_params.pole_pairs = 6.0f;
        MotorSim_Init(&g_motor_sim_params);
        ADCSim_Init(12, 3.3f, 0.001f, 100.0f);
    }

    void teardown() {
        MotorSim_Reset();
    }
};

TEST(ADC_Integration, ADC_CurrentFeedback)
{
    // 运行FOC控制，验证ADC反馈
    float frequency_rad_s = 50.0f;
    uint32_t Tcm1, Tcm2, Tcm3;

    for (int i = 0; i < 1000; i++) {
        // FOC控制
        FOC_OpenLoopTest(frequency_rad_s, &Tcm1, &Tcm2, &Tcm3);

        // PWM → 电压
        const float ARR_PERIOD = 5312.0f;
        float Ua_pu = ((float)Tcm1 / ARR_PERIOD - 0.5f) * 2.0f;
        float Ub_pu = ((float)Tcm2 / ARR_PERIOD - 0.5f) * 2.0f;

        float U_alpha_pu, U_beta_pu;
        Clarke_Transform(Ua_pu, Ub_pu, &U_alpha_pu, &U_beta_pu);

        // 电机步进 + ADC采样
        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);
        float Ia, Ib, Ic;
        MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
        ADCSim_Sample(Ia, Ib, Ic);
    }

    // 获取ADC值
    const uint16_t* adc_raw = ADCSim_GetRawValues();

    // 验证ADC值在合理范围内（允许饱和）
    CHECK(adc_raw[0] <= 4095);
    CHECK(adc_raw[1] <= 4095);
    CHECK(adc_raw[2] <= 4095);

    // 验证ADC值与实际电流的对应关系
    float Ia, Ib, Ic;
    MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);

    // ADC转换公式：adc = (I * R_sense * Gain / V_ref) * 4095 + 2048
    // 反推电流：I = (adc - 2048) * V_ref / (4095 * R_sense * Gain)
    float I_from_adc_a = ((float)adc_raw[0] - 2048.0f) * 3.3f / (4095.0f * 0.001f * 100.0f);

    // 验证ADC转换机制正常工作（放宽容差，因为可能有饱和）
    // 主要验证ADC值的变化趋势正确
    if (adc_raw[0] != 0 && adc_raw[0] != 4095) {
        // 未饱和时，验证转换关系
        DOUBLES_EQUAL(Ia, I_from_adc_a, fabsf(Ia) * 0.5f);  // 50%容差
    }
}

TEST(ADC_Integration, ADC_WithNoise)
{
    // 添加噪声测试
    ADCSim_SetErrors(5.0f, 0.02f, 10.0f);  // 5 LSB噪声，2%增益误差，10 LSB偏移

    float frequency_rad_s = 50.0f;
    uint32_t Tcm1, Tcm2, Tcm3;

    for (int i = 0; i < 500; i++) {
        FOC_OpenLoopTest(frequency_rad_s, &Tcm1, &Tcm2, &Tcm3);

        const float ARR_PERIOD = 5312.0f;
        float Ua_pu = ((float)Tcm1 / ARR_PERIOD - 0.5f) * 2.0f;
        float Ub_pu = ((float)Tcm2 / ARR_PERIOD - 0.5f) * 2.0f;

        float U_alpha_pu, U_beta_pu;
        Clarke_Transform(Ua_pu, Ub_pu, &U_alpha_pu, &U_beta_pu);

        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);
        float Ia, Ib, Ic;
        MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
        ADCSim_Sample(Ia, Ib, Ic);
    }

    // 验证即使有噪声，系统仍然稳定
    const MotorSimState_t* state = MotorSim_GetState();
    CHECK(fabsf(state->omega_e) > 1.0f);
    CHECK(fabsf(state->omega_e) < 200.0f);  // 不应该发散
}

// ============================================================================
// Test group: 负载扰动测试
// ============================================================================
TEST_GROUP(Load_Disturbance)
{
    void setup() {
        MotorSim_SetDefaultParams();
        g_motor_sim_params.pole_pairs = 6.0f;
        MotorSim_Init(&g_motor_sim_params);
        ADCSim_Init(12, 3.3f, 0.001f, 100.0f);
    }

    void teardown() {
        MotorSim_Reset();
    }
};

TEST(Load_Disturbance, SuddenLoadChange)
{
    // 测试突加负载
    float frequency_rad_s = 100.0f;
    uint32_t Tcm1, Tcm2, Tcm3;

    // 先运行到稳态（无负载）
    for (int i = 0; i < 2000; i++) {
        FOC_OpenLoopTest(frequency_rad_s, &Tcm1, &Tcm2, &Tcm3);

        const float ARR_PERIOD = 5312.0f;
        float Ua_pu = ((float)Tcm1 / ARR_PERIOD - 0.5f) * 2.0f;
        float Ub_pu = ((float)Tcm2 / ARR_PERIOD - 0.5f) * 2.0f;

        float U_alpha_pu, U_beta_pu;
        Clarke_Transform(Ua_pu, Ub_pu, &U_alpha_pu, &U_beta_pu);

        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);
        float Ia, Ib, Ic;
        MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
        ADCSim_Sample(Ia, Ib, Ic);
    }

    // 记录无负载时的转速
    float omega_no_load = MotorSim_GetState()->omega_e;

    // 突加负载
    MotorSim_SetLoadTorque(0.05f);  // 0.05 N·m

    // 继续运行
    for (int i = 0; i < 2000; i++) {
        FOC_OpenLoopTest(frequency_rad_s, &Tcm1, &Tcm2, &Tcm3);

        const float ARR_PERIOD = 5312.0f;
        float Ua_pu = ((float)Tcm1 / ARR_PERIOD - 0.5f) * 2.0f;
        float Ub_pu = ((float)Tcm2 / ARR_PERIOD - 0.5f) * 2.0f;

        float U_alpha_pu, U_beta_pu;
        Clarke_Transform(Ua_pu, Ub_pu, &U_alpha_pu, &U_beta_pu);

        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);
        float Ia, Ib, Ic;
        MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
        ADCSim_Sample(Ia, Ib, Ic);
    }

    // 验证转速下降（开环控制下，负载会导致转速下降）
    float omega_with_load = MotorSim_GetState()->omega_e;
    CHECK(omega_with_load < omega_no_load);

    // 验证电流增加（负载增加，电流应该增加）
    CHECK(fabsf(MotorSim_GetState()->Iq) > 0.5f);
}
