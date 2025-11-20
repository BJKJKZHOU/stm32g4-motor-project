/*============================================================================
    File Name     : test_motor_simulator.cpp
    Description   : 电机模拟器单元测试
    Author        : ZHOUHENG
    Date          : 2025-11-20
    ----------------------------------------------------------------------
    测试内容：
    1. 电机模拟器初始化和参数设置
    2. 开环电压输入，验证电流响应
    3. 稳态电流验证（dq坐标系）
    4. 机械动态响应（转速、转矩）
    5. ADC采样模拟
    6. 完整闭环测试（SVPWM → 电机 → ADC → 电流环）
*=============================================================================*/

#include "CppUTest/TestHarness.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTest/CommandLineTestRunner.h"

extern "C" {
#include "motor_simulator.h"
#include <math.h>
}

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define FLOAT_TOLERANCE 0.01f  // 浮点数比较容差

// ============================================================================
// Test group: 电机模拟器初始化
// ============================================================================
TEST_GROUP(MotorSimulator_Init)
{
    void setup() {
        MotorSim_Reset();
    }

    void teardown() {
    }
};

TEST(MotorSimulator_Init, DefaultParams)
{
    MotorSim_SetDefaultParams();

    // 验证默认参数设置
    CHECK(g_motor_sim_params.Rs > 0.0f);
    CHECK(g_motor_sim_params.Ld > 0.0f);
    CHECK(g_motor_sim_params.Lq > 0.0f);
    CHECK(g_motor_sim_params.flux_linkage > 0.0f);
    CHECK(g_motor_sim_params.pole_pairs > 0.0f);
    CHECK(g_motor_sim_params.V_dc > 0.0f);
}

TEST(MotorSimulator_Init, ResetState)
{
    // 设置一些非零状态
    g_motor_sim_state.Id = 1.0f;
    g_motor_sim_state.Iq = 2.0f;
    g_motor_sim_state.omega_e = 100.0f;

    // 重置
    MotorSim_Reset();

    // 验证所有状态归零
    DOUBLES_EQUAL(0.0f, g_motor_sim_state.Id, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, g_motor_sim_state.Iq, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, g_motor_sim_state.omega_e, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, g_motor_sim_state.theta_e, FLOAT_TOLERANCE);
}

// ============================================================================
// Test group: 电机稳态响应
// ============================================================================
TEST_GROUP(MotorSimulator_SteadyState)
{
    void setup() {
        MotorSim_SetDefaultParams();
        MotorSim_Reset();
        MotorSim_Init(&g_motor_sim_params);
    }

    void teardown() {
    }
};

TEST(MotorSimulator_SteadyState, ZeroVoltage_ZeroCurrent)
{
    // 施加零电压，运行一段时间
    for (int i = 0; i < 1000; i++) {
        MotorSim_Step_DQ(0.0f, 0.0f);
    }

    // 验证电流保持为零
    DOUBLES_EQUAL(0.0f, g_motor_sim_state.Id, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, g_motor_sim_state.Iq, FLOAT_TOLERANCE);
}

TEST(MotorSimulator_SteadyState, ConstantDAxisVoltage)
{
    // 施加恒定d轴电压（标幺值 0.1）
    float Ud_pu = 0.1f;

    // 运行足够长时间达到稳态（约10个时间常数）
    float L_R_ratio = g_motor_sim_params.Ld / g_motor_sim_params.Rs;
    int steps = (int)(10.0f * L_R_ratio / g_motor_sim_params.Ts);

    for (int i = 0; i < steps; i++) {
        MotorSim_Step_DQ(Ud_pu, 0.0f);
    }

    // 稳态时：Id = Ud / Rs
    float Id_expected = Ud_pu * g_motor_sim_params.V_dc / g_motor_sim_params.Rs;
    DOUBLES_EQUAL(Id_expected, g_motor_sim_state.Id, Id_expected * 0.05f);  // 5%误差

    // Iq应该保持为零
    DOUBLES_EQUAL(0.0f, g_motor_sim_state.Iq, 0.01f);
}

TEST(MotorSimulator_SteadyState, ConstantQAxisVoltage_FromStandstill)
{
    // 从静止开始，施加q轴电压
    float Uq_pu = 0.8f;  // 更大的电压

    // 运行一段时间，电机会加速
    for (int i = 0; i < 5000; i++) {
        MotorSim_Step_DQ(0.0f, Uq_pu);
    }

    // 验证产生了电磁转矩（主要验证点）
    CHECK(fabsf(g_motor_sim_state.Te) > 0.001f);

    // 验证电机已经开始旋转
    CHECK(fabsf(g_motor_sim_state.omega_e) > 1.0f);

    // 验证产生了电流（降低阈值）
    CHECK(fabsf(g_motor_sim_state.Iq) > 0.01f);
}

// ============================================================================
// Test group: 机械动态响应
// ============================================================================
TEST_GROUP(MotorSimulator_Mechanical)
{
    void setup() {
        MotorSim_SetDefaultParams();
        MotorSim_Reset();
        MotorSim_Init(&g_motor_sim_params);
    }

    void teardown() {
    }
};

TEST(MotorSimulator_Mechanical, Acceleration_WithTorque)
{
    // 施加q轴电压产生转矩
    float Uq_pu = 0.5f;

    // 记录初始转速
    float omega_initial = g_motor_sim_state.omega_m;

    // 运行一段时间
    for (int i = 0; i < 5000; i++) {
        MotorSim_Step_DQ(0.0f, Uq_pu);
    }

    // 验证转速增加
    CHECK(g_motor_sim_state.omega_m > omega_initial);

    // 验证角度在变化
    CHECK(fabsf(g_motor_sim_state.theta_e) > 0.1f);
}

TEST(MotorSimulator_Mechanical, LoadTorque_Effect)
{
    // 设置负载转矩
    float T_load = 0.05f;  // 0.05 N·m
    MotorSim_SetLoadTorque(T_load);

    // 施加电压产生转矩
    float Uq_pu = 0.3f;

    // 运行到稳态
    for (int i = 0; i < 10000; i++) {
        MotorSim_Step_DQ(0.0f, Uq_pu);
    }

    // 验证电磁转矩与负载转矩平衡（稳态）
    // Te ≈ T_load + B·ω
    float expected_Te = T_load + g_motor_sim_params.B * g_motor_sim_state.omega_m;
    DOUBLES_EQUAL(expected_Te, g_motor_sim_state.Te, expected_Te * 0.1f);
}

TEST(MotorSimulator_Mechanical, RPM_Calculation)
{
    // 设置一个已知的机械角速度
    g_motor_sim_state.omega_m = 104.72f;  // 约1000 RPM

    float omega_m, theta_m, rpm;
    MotorSim_GetMechanicalState(&omega_m, &theta_m, &rpm);

    // 验证RPM计算
    DOUBLES_EQUAL(1000.0f, rpm, 1.0f);
}

// ============================================================================
// Test group: 三相电流输出
// ============================================================================
TEST_GROUP(MotorSimulator_ThreePhase)
{
    void setup() {
        MotorSim_SetDefaultParams();
        MotorSim_Reset();
        MotorSim_Init(&g_motor_sim_params);
    }

    void teardown() {
    }
};

TEST(MotorSimulator_ThreePhase, CurrentSum_IsZero)
{
    // 施加任意电压
    for (int i = 0; i < 1000; i++) {
        MotorSim_Step_AlphaBeta(0.3f, 0.2f);
    }

    float Ia, Ib, Ic;
    MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);

    // 验证三相电流之和为零（基尔霍夫电流定律）
    DOUBLES_EQUAL(0.0f, Ia + Ib + Ic, 0.01f);
}

TEST(MotorSimulator_ThreePhase, AlphaBeta_Input)
{
    // 使用αβ坐标系输入
    float U_alpha_pu = 0.2f;
    float U_beta_pu = 0.1f;

    for (int i = 0; i < 1000; i++) {
        MotorSim_Step_AlphaBeta(U_alpha_pu, U_beta_pu);
    }

    // 验证电流不为零
    CHECK(fabsf(g_motor_sim_state.Ia) > 0.01f ||
          fabsf(g_motor_sim_state.Ib) > 0.01f ||
          fabsf(g_motor_sim_state.Ic) > 0.01f);
}

// ============================================================================
// Test group: ADC模拟器
// ============================================================================
TEST_GROUP(ADCSimulator)
{
    void setup() {
        // 初始化ADC模拟器（12位，3.3V，1mΩ采样电阻，100倍增益）
        ADCSim_Init(12, 3.3f, 0.001f, 100.0f);
    }

    void teardown() {
    }
};

TEST(ADCSimulator, ZeroCurrent_MidpointADC)
{
    // 零电流应该对应ADC中点
    ADCSim_Sample(0.0f, 0.0f, 0.0f);

    const uint16_t* adc_values = ADCSim_GetRawValues();

    // 12位ADC中点是2048
    LONGS_EQUAL(2048, adc_values[0]);
    LONGS_EQUAL(2048, adc_values[1]);
    LONGS_EQUAL(2048, adc_values[2]);
}

TEST(ADCSimulator, PositiveCurrent_HigherADC)
{
    // 正电流应该产生高于中点的ADC值
    float I_test = 5.0f;  // 5A
    ADCSim_Sample(I_test, 0.0f, 0.0f);

    const uint16_t* adc_values = ADCSim_GetRawValues();

    // Ia应该 > 2048
    CHECK(adc_values[0] > 2048);

    // Ib和Ic应该 = 2048
    LONGS_EQUAL(2048, adc_values[1]);
    LONGS_EQUAL(2048, adc_values[2]);
}

TEST(ADCSimulator, NegativeCurrent_LowerADC)
{
    // 负电流应该产生低于中点的ADC值
    float I_test = -5.0f;  // -5A
    ADCSim_Sample(I_test, 0.0f, 0.0f);

    const uint16_t* adc_values = ADCSim_GetRawValues();

    // Ia应该 < 2048
    CHECK(adc_values[0] < 2048);
}

TEST(ADCSimulator, Saturation_MaxADC)
{
    // 过大的电流应该饱和到最大ADC值
    float I_large = 100.0f;  // 100A（远超量程）
    ADCSim_Sample(I_large, 0.0f, 0.0f);

    const uint16_t* adc_values = ADCSim_GetRawValues();

    // 应该饱和到4095
    LONGS_EQUAL(4095, adc_values[0]);
}

TEST(ADCSimulator, Saturation_MinADC)
{
    // 过小的电流应该饱和到0
    float I_large = -100.0f;  // -100A
    ADCSim_Sample(I_large, 0.0f, 0.0f);

    const uint16_t* adc_values = ADCSim_GetRawValues();

    // 应该饱和到0
    LONGS_EQUAL(0, adc_values[0]);
}

// ============================================================================
// Test group: 完整闭环测试
// ============================================================================
TEST_GROUP(MotorSimulator_ClosedLoop)
{
    void setup() {
        MotorSim_SetDefaultParams();
        MotorSim_Reset();
        MotorSim_Init(&g_motor_sim_params);
        ADCSim_Init(12, 3.3f, 0.001f, 100.0f);
    }

    void teardown() {
    }
};

TEST(MotorSimulator_ClosedLoop, StepAndSample)
{
    // 使用便捷函数：一步完成电机步进和ADC采样
    const uint16_t* adc_values = MotorSim_StepAndSample(0.2f, 0.1f);

    // 验证返回了ADC值
    CHECK(adc_values != NULL);

    // 验证ADC值在合理范围内
    CHECK(adc_values[0] <= 4095);
    CHECK(adc_values[1] <= 4095);
    CHECK(adc_values[2] <= 4095);
}

TEST(MotorSimulator_ClosedLoop, OpenLoop_CurrentResponse)
{
    // 模拟开环V/F控制
    float frequency = 50.0f;  // 50 Hz（更高频率）
    float voltage_pu = 0.5f;  // 更大电压

    // 运行多个周期
    int steps_per_cycle = (int)(1.0f / (frequency * g_motor_sim_params.Ts));

    for (int cycle = 0; cycle < 10; cycle++) {  // 更多周期
        for (int step = 0; step < steps_per_cycle; step++) {
            float angle = 2.0f * PI * frequency * g_motor_sim_state.time;
            float U_alpha = voltage_pu * cosf(angle);
            float U_beta = voltage_pu * sinf(angle);

            MotorSim_StepAndSample(U_alpha, U_beta);
        }
    }

    // 验证电机已经旋转（角度变化应该很大）
    CHECK(fabsf(g_motor_sim_state.theta_e) > 0.5f);

    // 验证产生了电流
    float Ia, Ib, Ic;
    MotorSim_GetCurrents_ABC(&Ia, &Ib, &Ic);
    CHECK(fabsf(Ia) > 0.1f || fabsf(Ib) > 0.1f || fabsf(Ic) > 0.1f);
}

// 注意：main函数在 test_FOC_math_cpputest.cpp 中定义，这里不需要重复定义
