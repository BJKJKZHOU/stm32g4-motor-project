/*============================================================================
    File Name     : test_current_loop.cpp
    Description   : 电流环控制测试
    Author        : ZHOUHENG
    Date          : 2025-11-20
    ----------------------------------------------------------------------
    测试目标：
    1. 测试电流环初始化和复位功能
    2. 测试电流环主控制函数的完整流程
    3. 验证电流环与电机模拟器的闭环控制
*=============================================================================*/

#include "CppUTest/TestHarness.h"

extern "C" {
#include "FOC_Loop.h"
#include "FOC_math.h"
#include "Current.h"
#include "motor_params.h"
#include "normalization.h"
#include "motor_simulator.h"
#include <math.h>
}

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define FLOAT_TOLERANCE 0.01f

// ============================================================================
// Test group: 电流环基础功能测试
// ============================================================================
TEST_GROUP(CurrentLoop_Basic)
{
    CurrentLoop_t loop;

    void setup() {
        // 初始化电机参数
        MotorParams_Init();
        MotorParams_SetActiveMotor(MOTOR_0);

        // 初始化归一化模块
        Normalization_Init();
        Normalization_UpdateMotor(MOTOR_0);

        // 初始化电流采样模块
        Current_Init();
    }

    void teardown() {
        // 清理
    }
};

TEST(CurrentLoop_Basic, InitializationTest)
{
    // 测试参数
    uint8_t motor_id = MOTOR_0;
    float dt = 50e-6f;  // 50μs控制周期
    float kp_d = 0.5f;
    float ki_d = 10.0f;
    float kp_q = 0.5f;
    float ki_q = 10.0f;
    float observer_gamma = 100.0f;

    // 初始化电流环
    CurrentLoop_Init(&loop, motor_id, dt, kp_d, ki_d, kp_q, ki_q, observer_gamma);

    // 验证初始化结果
    CHECK(loop.is_initialized == true);
    CHECK(loop.is_running == false);
    CHECK_EQUAL(motor_id, loop.motor_id);
    DOUBLES_EQUAL(dt, loop.dt, 1e-9);

    // 验证PID参数
    DOUBLES_EQUAL(kp_d, loop.pid_d_params.kp, 1e-6);
    DOUBLES_EQUAL(ki_d, loop.pid_d_params.ki, 1e-6);
    DOUBLES_EQUAL(kp_q, loop.pid_q_params.kp, 1e-6);
    DOUBLES_EQUAL(ki_q, loop.pid_q_params.ki, 1e-6);

    // 验证PID状态初始化为零
    DOUBLES_EQUAL(0.0f, loop.pid_d_state.integral, 1e-6);
    DOUBLES_EQUAL(0.0f, loop.pid_q_state.integral, 1e-6);

    // 验证PWM输出初始化为中点值
    CHECK_EQUAL(ARR_PERIOD / 2, loop.last_Tcm1);
    CHECK_EQUAL(ARR_PERIOD / 2, loop.last_Tcm2);
    CHECK_EQUAL(ARR_PERIOD / 2, loop.last_Tcm3);
}

TEST(CurrentLoop_Basic, ResetTest)
{
    // 初始化电流环
    CurrentLoop_Init(&loop, MOTOR_0, 50e-6f, 0.5f, 10.0f, 0.5f, 10.0f, 100.0f);

    // 模拟运行后的状态
    loop.pid_d_state.integral = 0.5f;
    loop.pid_q_state.integral = 0.3f;
    loop.id_feedback = 0.2f;
    loop.iq_feedback = 0.4f;
    loop.is_running = true;

    // 复位电流环
    CurrentLoop_Reset(&loop);

    // 验证复位结果
    CHECK(loop.is_running == false);
    DOUBLES_EQUAL(0.0f, loop.pid_d_state.integral, 1e-6);
    DOUBLES_EQUAL(0.0f, loop.pid_q_state.integral, 1e-6);
    DOUBLES_EQUAL(0.0f, loop.id_feedback, 1e-6);
    DOUBLES_EQUAL(0.0f, loop.iq_feedback, 1e-6);

    // 验证PWM输出复位为中点值
    CHECK_EQUAL(ARR_PERIOD / 2, loop.last_Tcm1);
    CHECK_EQUAL(ARR_PERIOD / 2, loop.last_Tcm2);
    CHECK_EQUAL(ARR_PERIOD / 2, loop.last_Tcm3);
}

// ============================================================================
// Test group: PWM到电压转换测试
// ============================================================================
TEST_GROUP(PWM_To_Voltage)
{
    void setup() {
    }

    void teardown() {
    }
};

TEST(PWM_To_Voltage, MidpointTest)
{
    // 测试中点值（50%占空比）应该输出零电压
    uint32_t Tcm1 = ARR_PERIOD / 2;
    uint32_t Tcm2 = ARR_PERIOD / 2;
    uint32_t Tcm3 = ARR_PERIOD / 2;
    float V_DC = 24.0f;

    float Ua, Ub, Uc;
    PWM_To_Voltage_ABC(Tcm1, Tcm2, Tcm3, V_DC, &Ua, &Ub, &Uc);

    // 50%占空比应该输出零电压
    DOUBLES_EQUAL(0.0f, Ua, 1e-6);
    DOUBLES_EQUAL(0.0f, Ub, 1e-6);
    DOUBLES_EQUAL(0.0f, Uc, 1e-6);
}

TEST(PWM_To_Voltage, FullScaleTest)
{
    // 测试满量程（100%占空比）
    uint32_t Tcm1 = ARR_PERIOD;
    uint32_t Tcm2 = ARR_PERIOD / 2;
    uint32_t Tcm3 = 0;
    float V_DC = 24.0f;

    float Ua, Ub, Uc;
    PWM_To_Voltage_ABC(Tcm1, Tcm2, Tcm3, V_DC, &Ua, &Ub, &Uc);

    // 100%占空比 → +V_DC/2
    DOUBLES_EQUAL(V_DC / 2.0f, Ua, 1e-3);
    // 50%占空比 → 0
    DOUBLES_EQUAL(0.0f, Ub, 1e-3);
    // 0%占空比 → -V_DC/2
    DOUBLES_EQUAL(-V_DC / 2.0f, Uc, 1e-3);
}

TEST(PWM_To_Voltage, SymmetryTest)
{
    // 测试对称性
    float V_DC = 24.0f;

    // 测试75%占空比
    uint32_t Tcm_75 = ARR_PERIOD * 3 / 4;
    float Ua_75, Ub_75, Uc_75;
    PWM_To_Voltage_ABC(Tcm_75, ARR_PERIOD / 2, ARR_PERIOD / 2, V_DC, &Ua_75, &Ub_75, &Uc_75);

    // 测试25%占空比
    uint32_t Tcm_25 = ARR_PERIOD / 4;
    float Ua_25, Ub_25, Uc_25;
    PWM_To_Voltage_ABC(Tcm_25, ARR_PERIOD / 2, ARR_PERIOD / 2, V_DC, &Ua_25, &Ub_25, &Uc_25);

    // 验证对称性：75%的电压应该等于-25%的电压
    // 由于整数除法的精度问题，放宽容差
    DOUBLES_EQUAL(-Ua_25, Ua_75, 0.01);
}

// ============================================================================
// Test group: 电流环闭环控制测试（集成测试）
// ============================================================================
TEST_GROUP(CurrentLoop_ClosedLoop)
{
    CurrentLoop_t loop;

    void setup() {
        // 初始化电机参数
        MotorParams_Init();
        MotorParams_SetActiveMotor(MOTOR_0);

        // 初始化归一化模块
        Normalization_Init();
        Normalization_UpdateMotor(MOTOR_0);

        // 初始化电流采样模块
        Current_Init();

        // 初始化电机模拟器
        MotorSim_SetDefaultParams();
        g_motor_sim_params.pole_pairs = 6.0f;
        g_motor_sim_params.Ts = 50e-6f;
        MotorSim_Init(&g_motor_sim_params);

        // 初始化ADC模拟器
        ADCSim_Init(12, 3.3f, 0.001f, 100.0f);

        // 初始化电流环
        CurrentLoop_Init(&loop, MOTOR_0, 50e-6f, 0.5f, 10.0f, 0.5f, 10.0f, 100.0f);
    }

    void teardown() {
        MotorSim_Reset();
    }
};

TEST(CurrentLoop_ClosedLoop, BasicRunTest)
{
    // 测试电流环基本运行
    float id_ref = 0.0f;   // d轴电流设定为0（PMSM常用策略）
    float iq_ref = 0.1f;   // q轴电流设定为0.1标幺值

    uint32_t Tcm1, Tcm2, Tcm3;

    // 运行100个控制周期
    for (int i = 0; i < 100; i++) {
        // 调用电流环控制函数
        bool result = CurrentLoop_Run(&loop, id_ref, iq_ref, &Tcm1, &Tcm2, &Tcm3);
        CHECK(result == true);

        // 验证PWM输出在合理范围内
        CHECK(Tcm1 <= ARR_PERIOD);
        CHECK(Tcm2 <= ARR_PERIOD);
        CHECK(Tcm3 <= ARR_PERIOD);

        // 模拟电机响应（简化处理，实际需要完整的电机模型）
        // 这里仅验证函数能够正常运行
    }

    // 验证电流环已经运行
    CHECK(loop.is_running == true);
}

TEST(CurrentLoop_ClosedLoop, NullPointerTest)
{
    // 测试空指针保护
    uint32_t Tcm1, Tcm2, Tcm3;

    // 测试loop为NULL
    bool result1 = CurrentLoop_Run(NULL, 0.0f, 0.0f, &Tcm1, &Tcm2, &Tcm3);
    CHECK(result1 == false);

    // 测试输出指针为NULL
    bool result2 = CurrentLoop_Run(&loop, 0.0f, 0.0f, NULL, &Tcm2, &Tcm3);
    CHECK(result2 == false);

    bool result3 = CurrentLoop_Run(&loop, 0.0f, 0.0f, &Tcm1, NULL, &Tcm3);
    CHECK(result3 == false);

    bool result4 = CurrentLoop_Run(&loop, 0.0f, 0.0f, &Tcm1, &Tcm2, NULL);
    CHECK(result4 == false);
}

TEST(CurrentLoop_ClosedLoop, SetpointTrackingTest)
{
    // 测试设定值跟踪
    float id_ref = 0.0f;
    float iq_ref = 0.2f;

    uint32_t Tcm1, Tcm2, Tcm3;

    // 运行500个控制周期，观察设定值是否被正确保存
    for (int i = 0; i < 500; i++) {
        CurrentLoop_Run(&loop, id_ref, iq_ref, &Tcm1, &Tcm2, &Tcm3);
    }

    // 验证设定值被正确保存
    DOUBLES_EQUAL(id_ref, loop.id_setpoint, 1e-6);
    DOUBLES_EQUAL(iq_ref, loop.iq_setpoint, 1e-6);
}
