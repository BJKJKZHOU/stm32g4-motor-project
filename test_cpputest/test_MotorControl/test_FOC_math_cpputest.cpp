
/*============================================================================
    File Name     : test_FOC_math_cpputest.cpp
    Description   : FOC math module unit tests using CppUTest framework
    Author        : ZHOUHENG
    Date          : 2025-11-10
    ----------------------------------------------------------------------       
    Note:
        This is the CppUTest version of FOC_math tests
        Provides advanced testing features like test groups, setup/teardown, etc.
=============================================================================*/

#include "CppUTest/TestHarness.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "FOC_math.h"
#include "stm32_hal_stubs.h"
#include <math.h>
}

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define SQRT3_OVER_TWO (0.8660254037844386f)
#define INV_SQRT3 (0.5773502691896258f)
#define FLOAT_TOLERANCE 0.001f  // 浮点数比较容差

// ============================================================================
// Test group for Clarke Transform (abc -> αβ)
// ============================================================================
TEST_GROUP(ClarkeTransform)
{
    void setup() {
        // Test setup
    }
    
    void teardown() {
        mock().clear();
    }
};

// 测试：零电流输入
TEST(ClarkeTransform, ZeroInput)
{
    float I_alpha = 0.0f, I_beta = 0.0f;
    bool result = Clarke_Transform(0.0f, 0.0f, &I_alpha, &I_beta);
    
    CHECK_TRUE(result);
    DOUBLES_EQUAL(0.0f, I_alpha, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, I_beta, FLOAT_TOLERANCE);
}

// 测试：Clarke变换公式验证
// I_alpha = ia
// I_beta = (ia + 2*ib) / sqrt(3)
TEST(ClarkeTransform, FormulaVerification)
{
    float ia = 0.5f;
    float ib = -0.25f;
    float I_alpha = 0.0f, I_beta = 0.0f;
    
    bool result = Clarke_Transform(ia, ib, &I_alpha, &I_beta);
    
    CHECK_TRUE(result);
    DOUBLES_EQUAL(ia, I_alpha, FLOAT_TOLERANCE);
    DOUBLES_EQUAL((ia + 2.0f * ib) * INV_SQRT3, I_beta, FLOAT_TOLERANCE);
}

// 测试：空指针处理
TEST(ClarkeTransform, NullPointer)
{
    float I_alpha = 0.0f;
    bool result = Clarke_Transform(0.5f, 0.5f, &I_alpha, NULL);
    CHECK_FALSE(result);
}

// 测试：对称三相电流
TEST(ClarkeTransform, BalancedThreePhase)
{
    // ia = I*cos(0°) = I, ib = I*cos(120°) = -0.5*I
    // Clarke变换：I_alpha = ia, I_beta = (ia + 2*ib) / sqrt(3)
    // I_beta = (1 + 2*(-0.5)) / sqrt(3) = 0 / sqrt(3) = 0
    // 这个测试用例实际上不会产生sqrt(3)/2的结果
    
    // 修改为正确的测试用例：ia = 0, ib = I，应该产生I_beta = 2*I/sqrt(3)
    float I_peak = 1.0f;
    float ia = 0.0f;
    float ib = I_peak;
    float I_alpha = 0.0f, I_beta = 0.0f;
    
    bool result = Clarke_Transform(ia, ib, &I_alpha, &I_beta);
    
    CHECK_TRUE(result);
    DOUBLES_EQUAL(0.0f, I_alpha, FLOAT_TOLERANCE);
    // I_beta = (0 + 2*1) / sqrt(3) = 2/sqrt(3) = 1.1547
    float expected_beta = 2.0f * I_peak * INV_SQRT3;
    DOUBLES_EQUAL(expected_beta, I_beta, FLOAT_TOLERANCE);
}

// ============================================================================
// Test group for Park Transform (αβ -> dq)
// ============================================================================
TEST_GROUP(ParkTransform)
{
    void setup() {
    }
    
    void teardown() {
        mock().clear();
    }
};

// 测试：Park变换公式验证 - 0度
// I_d = I_alpha * cos(θ) + I_beta * sin(θ)
// I_q = -I_alpha * sin(θ) + I_beta * cos(θ)
TEST(ParkTransform, FormulaVerification_0deg)
{
    float I_alpha = 1.0f;
    float I_beta = 0.0f;
    float theta = 0.0f;  // 0度
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float I_d = 0.0f, I_q = 0.0f;
    
    Park_Transform(I_alpha, I_beta, sin_theta, cos_theta, &I_d, &I_q);
    
    // 0度时：I_d = I_alpha, I_q = 0
    DOUBLES_EQUAL(I_alpha, I_d, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, I_q, FLOAT_TOLERANCE);
}

// 测试：90度Park变换
TEST(ParkTransform, FormulaVerification_90deg)
{
    float I_alpha = 1.0f;
    float I_beta = 0.0f;
    float theta = PI / 2.0f;  // 90度
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float I_d = 0.0f, I_q = 0.0f;
    
    Park_Transform(I_alpha, I_beta, sin_theta, cos_theta, &I_d, &I_q);
    
    // 90度时：I_d ≈ 0, I_q ≈ -I_alpha
    DOUBLES_EQUAL(0.0f, I_d, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(-I_alpha, I_q, FLOAT_TOLERANCE);
}

// 测试：空指针处理
TEST(ParkTransform, NullPointer)
{
    float I_d = 0.0f;
    Park_Transform(1.0f, 0.0f, 0.0f, 1.0f, &I_d, NULL);
    // 函数应该安全返回，不崩溃
}

// 测试：零输入
TEST(ParkTransform, ZeroInput)
{
    float I_d = 99.0f, I_q = 99.0f;
    Park_Transform(0.0f, 0.0f, 0.5f, 0.866f, &I_d, &I_q);
    
    DOUBLES_EQUAL(0.0f, I_d, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, I_q, FLOAT_TOLERANCE);
}

// ============================================================================
// Test group for Inverse Park Transform (dq -> αβ)
// ============================================================================
TEST_GROUP(InverseParkTransform)
{
    void setup() {
    }
    
    void teardown() {
        mock().clear();
    }
};

// 测试：逆Park变换公式验证 - 0度
// U_alpha = U_d * cos(θ) - U_q * sin(θ)
// U_beta = U_d * sin(θ) + U_q * cos(θ)
TEST(InverseParkTransform, FormulaVerification_0deg)
{
    float U_d = 0.5f;
    float U_q = 0.3f;
    float theta = 0.0f;
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float U_alpha = 0.0f, U_beta = 0.0f;
    
    Inverse_Park_Transform(U_d, U_q, sin_theta, cos_theta, &U_alpha, &U_beta);
    
    // 0度时：U_alpha = U_d, U_beta = U_q
    DOUBLES_EQUAL(U_d, U_alpha, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(U_q, U_beta, FLOAT_TOLERANCE);
}

// 测试：逆Park变换 90度
TEST(InverseParkTransform, FormulaVerification_90deg)
{
    float U_d = 0.5f;
    float U_q = 0.3f;
    float theta = PI / 2.0f;
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float U_alpha = 0.0f, U_beta = 0.0f;
    
    Inverse_Park_Transform(U_d, U_q, sin_theta, cos_theta, &U_alpha, &U_beta);
    
    // 90度时：U_alpha ≈ -U_q, U_beta ≈ U_d
    DOUBLES_EQUAL(-U_q, U_alpha, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(U_d, U_beta, FLOAT_TOLERANCE);
}

// 测试：Park和逆Park的互逆性
TEST(InverseParkTransform, InversePropertyVerification)
{
    // 先做逆Park变换
    float U_d = 0.6f;
    float U_q = 0.4f;
    float theta = PI / 4.0f;  // 45度
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float U_alpha = 0.0f, U_beta = 0.0f;
    
    Inverse_Park_Transform(U_d, U_q, sin_theta, cos_theta, &U_alpha, &U_beta);
    
    // 再做Park变换，应该恢复原值
    float I_d_restored = 0.0f, I_q_restored = 0.0f;
    Park_Transform(U_alpha, U_beta, sin_theta, cos_theta, &I_d_restored, &I_q_restored);
    
    DOUBLES_EQUAL(U_d, I_d_restored, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(U_q, I_q_restored, FLOAT_TOLERANCE);
}

// 测试：空指针处理
TEST(InverseParkTransform, NullPointer)
{
    float U_alpha = 0.0f;
    Inverse_Park_Transform(0.5f, 0.3f, 0.0f, 1.0f, &U_alpha, NULL);
    // 应该安全返回
}

// 测试：零输入
TEST(InverseParkTransform, ZeroInput)
{
    float U_alpha = 99.0f, U_beta = 99.0f;
    Inverse_Park_Transform(0.0f, 0.0f, 0.5f, 0.866f, &U_alpha, &U_beta);
    
    DOUBLES_EQUAL(0.0f, U_alpha, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0f, U_beta, FLOAT_TOLERANCE);
}

// ============================================================================
// Test group for SVPWM (αβ -> PWM duty cycles)
// ============================================================================
TEST_GROUP(SVPWM)
{
    void setup() {
    }
    
    void teardown() {
        mock().clear();
    }
};

// 测试：零电压输入
TEST(SVPWM, ZeroVoltageInput)
{
    uint32_t Tcm1 = 0, Tcm2 = 0, Tcm3 = 0;
    
    SVPWM_minmax(0.0f, 0.0f, &Tcm1, &Tcm2, &Tcm3);
    
    // 零电压应该产生接近50%占空比
    // 由于舍入误差，允许±1的误差
    uint32_t expected = ARR_PERIOD / 2;
    CHECK(Tcm1 >= expected - 1 && Tcm1 <= expected + 1);
    CHECK(Tcm2 >= expected - 1 && Tcm2 <= expected + 1);
    CHECK(Tcm3 >= expected - 1 && Tcm3 <= expected + 1);
}

// 测试：逆Clarke变换正确性验证
TEST(SVPWM, InverseClarkTransformVerification)
{
    // 输入αβ电压
    float U_alpha = 0.4f;
    float U_beta = 0.3f;
    uint32_t Tcm1 = 0, Tcm2 = 0, Tcm3 = 0;
    
    SVPWM_minmax(U_alpha, U_beta, &Tcm1, &Tcm2, &Tcm3);
    
    // 验证输出在合理范围内 [0, ARR_PERIOD]
    CHECK(Tcm1 <= ARR_PERIOD);
    CHECK(Tcm2 <= ARR_PERIOD);
    CHECK(Tcm3 <= ARR_PERIOD);
    CHECK(Tcm1 >= 0);
    CHECK(Tcm2 >= 0);
    CHECK(Tcm3 >= 0);
}

// 测试：马鞍波特性 - 旋转磁场测试
TEST(SVPWM, RotatingFieldTest)
{
    // 测试多个角度
    for (int angle_deg = 0; angle_deg < 360; angle_deg += 30) {
        float theta = angle_deg * PI / 180.0f;
        float U_alpha = 0.5f * cosf(theta);
        float U_beta = 0.5f * sinf(theta);
        
        uint32_t Tcm1 = 0, Tcm2 = 0, Tcm3 = 0;
        SVPWM_minmax(U_alpha, U_beta, &Tcm1, &Tcm2, &Tcm3);
        
        // 所有输出应该在有效范围内
        CHECK(Tcm1 >= 0 && Tcm1 <= ARR_PERIOD);
        CHECK(Tcm2 >= 0 && Tcm2 <= ARR_PERIOD);
        CHECK(Tcm3 >= 0 && Tcm3 <= ARR_PERIOD);
    }
}

// 测试：过调制保护
TEST(SVPWM, OverModulationProtection)
{
    // 输入超出范围的电压
    float U_alpha = 1.5f;  // 超出标幺值范围
    float U_beta = 1.5f;
    uint32_t Tcm1 = 0, Tcm2 = 0, Tcm3 = 0;
    
    SVPWM_minmax(U_alpha, U_beta, &Tcm1, &Tcm2, &Tcm3);
    printf("\nTcm1: %d, Tcm2: %d, Tcm3: %d\n", Tcm1, Tcm2, Tcm3);
    
    // 应该被饱和限制到ARR_PERIOD
    CHECK(Tcm1 <= ARR_PERIOD);
    CHECK(Tcm2 <= ARR_PERIOD);
    CHECK(Tcm3 <= ARR_PERIOD);
}

// 测试：空指针处理
TEST(SVPWM, NullPointer)
{
    uint32_t Tcm1 = 0, Tcm2 = 0;
    SVPWM_minmax(0.5f, 0.3f, &Tcm1, &Tcm2, NULL);
    // 应该安全返回
}

// 测试：三相对称性
TEST(SVPWM, ThreePhaseSymmetry)
{
    // U_alpha = 0.5, U_beta = 0 应该对应A相最大
    uint32_t Tcm1 = 0, Tcm2 = 0, Tcm3 = 0;
    SVPWM_minmax(0.5f, 0.0f, &Tcm1, &Tcm2, &Tcm3);
    
    // A相应该最大
    CHECK(Tcm1 > Tcm2);
    CHECK(Tcm1 > Tcm3);
}

// ============================================================================
// Test group for Low Pass Filter (Float version)
// ============================================================================
TEST_GROUP(LowPassFilter)
{
    LPF_Float_t filter;
    
    void setup() {
        filter.state = 0.0f;
    }
    
    void teardown() {
        mock().clear();
    }
};

// 测试：滤波器阶跃响应
TEST(LowPassFilter, StepResponse)
{
    float cutoff_freq = 100.0f;  // 100 Hz
    float sample_time = 0.001f;  // 1ms
    
    // 应用阶跃输入
    float output = 0.0f;
    for (int i = 0; i < 10; i++) {
        output = LPF_Filter(&filter, 1.0f, cutoff_freq, sample_time, true);
    }
    
    // 输出应该接近1.0，但不会立即达到
    CHECK(output > 0.1f);
    CHECK(output < 1.0f);
}

// 测试：滤波器稳态
TEST(LowPassFilter, SteadyState)
{
    float cutoff_freq = 100.0f;
    float sample_time = 0.001f;
    float input = 1.0f;
    
    // 运行足够长时间达到稳态
    float output = 0.0f;
    for (int i = 0; i < 1000; i++) {
        output = LPF_Filter(&filter, input, cutoff_freq, sample_time, true);
    }
    
    // 稳态输出应该接近输入
    DOUBLES_EQUAL(input, output, 0.01f);
}

// 测试：单位选择 - Hz vs rad/s
TEST(LowPassFilter, UnitSelection)
{
    LPF_Float_t filter_hz, filter_rad;
    filter_hz.state = 0.0f;
    filter_rad.state = 0.0f;
    
    float freq_hz = 100.0f;
    float freq_rad = 2.0f * PI * freq_hz;  // 转换为rad/s
    float sample_time = 0.001f;
    
    float output_hz = LPF_Filter(&filter_hz, 1.0f, freq_hz, sample_time, true);
    float output_rad = LPF_Filter(&filter_rad, 1.0f, freq_rad, sample_time, false);
    
    // 两种单位应该产生相同结果
    DOUBLES_EQUAL(output_hz, output_rad, 0.001f);
}

// ============================================================================
// Test group for Complete FOC Chain
// ============================================================================
TEST_GROUP(FOCCompleteChain)
{
    void setup() {
    }
    
    void teardown() {
        mock().clear();
    }
};

// 测试：完整FOC链路 - Clarke -> Park -> Inverse Park -> SVPWM
TEST(FOCCompleteChain, CompleteTransformationChain)
{
    // 1. 三相电流
    float ia = 0.5f;
    float ib = -0.25f;
    
    // 2. Clarke变换
    float I_alpha = 0.0f, I_beta = 0.0f;
    bool result = Clarke_Transform(ia, ib, &I_alpha, &I_beta);
    CHECK_TRUE(result);
    
    // 3. Park变换
    float theta = PI / 6.0f;  // 30度
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float I_d = 0.0f, I_q = 0.0f;
    Park_Transform(I_alpha, I_beta, sin_theta, cos_theta, &I_d, &I_q);
    
    // 4. 逆Park变换（模拟PID控制后）
    float U_d = I_d * 0.8f;  // 简单的比例控制
    float U_q = I_q * 0.8f;
    float U_alpha = 0.0f, U_beta = 0.0f;
    Inverse_Park_Transform(U_d, U_q, sin_theta, cos_theta, &U_alpha, &U_beta);
    
    // 5. SVPWM
    uint32_t Tcm1 = 0, Tcm2 = 0, Tcm3 = 0;
    SVPWM_minmax(U_alpha, U_beta, &Tcm1, &Tcm2, &Tcm3);
    
    // 验证整个链路没有错误
    CHECK(Tcm1 <= ARR_PERIOD);
    CHECK(Tcm2 <= ARR_PERIOD);
    CHECK(Tcm3 <= ARR_PERIOD);
}

// 测试：FOC零序分量注入验证
TEST(FOCCompleteChain, ZeroSequenceInjection)
{
    // 创建一个旋转的电压矢量
    float theta = PI / 3.0f;  // 60度
    float U_d = 0.0f;
    float U_q = 0.5f;
    
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float U_alpha = 0.0f, U_beta = 0.0f;
    
    Inverse_Park_Transform(U_d, U_q, sin_theta, cos_theta, &U_alpha, &U_beta);
    
    uint32_t Tcm1 = 0, Tcm2 = 0, Tcm3 = 0;
    SVPWM_minmax(U_alpha, U_beta, &Tcm1, &Tcm2, &Tcm3);
    
    // 验证三相PWM值的合理性
    // 马鞍波应该使得max + min = ARR_PERIOD
    uint32_t max_val = (Tcm1 > Tcm2) ? ((Tcm1 > Tcm3) ? Tcm1 : Tcm3) : ((Tcm2 > Tcm3) ? Tcm2 : Tcm3);
    uint32_t min_val = (Tcm1 < Tcm2) ? ((Tcm1 < Tcm3) ? Tcm1 : Tcm3) : ((Tcm2 < Tcm3) ? Tcm2 : Tcm3);
    
    // 对于标幺化SVPWM，max + min应该接近ARR_PERIOD
    CHECK(max_val + min_val >= ARR_PERIOD * 0.8f);
    CHECK(max_val + min_val <= ARR_PERIOD * 1.2f);
}

// ============================================================================
// Main test runner
// ============================================================================
int main(int argc, char** argv)
{
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
