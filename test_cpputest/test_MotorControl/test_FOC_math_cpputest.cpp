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

// Include FOC_math header
extern "C" {
#include "FOC_math.h"
#include "stm32_hal_stubs.h"
}

// Test group for Inverse Park Transform
TEST_GROUP(InverseParkTransform)
{
    void setup() {
        // Test setup - called before each test
    }
    
    void teardown() {
        // Test cleanup - called after each test
        mock().clear();
    }
};

// Test basic case: U_d=1.0, U_q=0.0, theta=0
TEST(InverseParkTransform, BasicCase)
{
    float U_alpha, U_beta;
    
    Inverse_Park_Transform(1.0f, 0.0f, 0.0f, 1.0f, &U_alpha, &U_beta);
    
    DOUBLES_EQUAL(1.0f, U_alpha, 0.001f);
    DOUBLES_EQUAL(0.0f, U_beta, 0.001f);
}

// Test 90 degree case: U_d=1.0, U_q=0.0, theta=90deg
TEST(InverseParkTransform, NinetyDegrees)
{
    float U_alpha, U_beta;
    
    Inverse_Park_Transform(1.0f, 0.0f, 1.0f, 0.0f, &U_alpha, &U_beta);
    
    DOUBLES_EQUAL(0.0f, U_alpha, 0.001f);
    DOUBLES_EQUAL(1.0f, U_beta, 0.001f);
}

// Test 45 degree case: U_d=1.0, U_q=0.0, theta=45deg
TEST(InverseParkTransform, FortyFiveDegrees)
{
    float U_alpha, U_beta;
    float sin_45 = 0.7071067811865476f;
    float cos_45 = 0.7071067811865476f;
    
    Inverse_Park_Transform(1.0f, 0.0f, sin_45, cos_45, &U_alpha, &U_beta);
    
    DOUBLES_EQUAL(0.7071f, U_alpha, 0.001f);
    DOUBLES_EQUAL(0.7071f, U_beta, 0.001f);
}

// Test negative voltage case: U_d=-1.0, U_q=0.0, theta=0
TEST(InverseParkTransform, NegativeVoltage)
{
    float U_alpha, U_beta;
    
    Inverse_Park_Transform(-1.0f, 0.0f, 0.0f, 1.0f, &U_alpha, &U_beta);
    
    DOUBLES_EQUAL(-1.0f, U_alpha, 0.001f);
    DOUBLES_EQUAL(0.0f, U_beta, 0.001f);
}

// Test both d and q components: U_d=1.0, U_q=1.0, theta=30deg
TEST(InverseParkTransform, BothComponents)
{
    float U_alpha, U_beta;
    float sin_30 = 0.5f;
    float cos_30 = 0.8660254037844386f;
    
    Inverse_Park_Transform(1.0f, 1.0f, sin_30, cos_30, &U_alpha, &U_beta);
    
    DOUBLES_EQUAL(0.3660f, U_alpha, 0.001f);
    DOUBLES_EQUAL(1.3660f, U_beta, 0.001f);
}

// Test group for Clarke Transform
TEST_GROUP(ClarkeTransform)
{
    void setup() {
        // Test setup
    }
    
    void teardown() {
        // Test cleanup
        mock().clear();
    }
};

// Test basic case: ia=1.0, ib=-0.5
TEST(ClarkeTransform, BasicCase)
{
    float I_alpha, I_beta;
    
    bool result = Clarke_Transform(1.0f, -0.5f, &I_alpha, &I_beta);
    
    CHECK_TRUE(result);
    DOUBLES_EQUAL(1.0f, I_alpha, 0.001f);
    
    // Calculate expected I_beta: (ia + 2*ib) * 1/sqrt(3)
    float expected_beta = (1.0f + 2.0f * (-0.5f)) * 0.5773502691896258f;
    DOUBLES_EQUAL(expected_beta, I_beta, 0.001f);
}

// Test zero input case
TEST(ClarkeTransform, ZeroInput)
{
    float I_alpha, I_beta;
    
    bool result = Clarke_Transform(0.0f, 0.0f, &I_alpha, &I_beta);
    
    CHECK_TRUE(result);
    DOUBLES_EQUAL(0.0f, I_alpha, 0.001f);
    DOUBLES_EQUAL(0.0f, I_beta, 0.001f);
}

// Test group for Park Transform
TEST_GROUP(ParkTransform)
{
    void setup() {
        // Test setup
    }
    
    void teardown() {
        // Test cleanup
        mock().clear();
    }
};

// Test basic case: I_alpha=1.0, I_beta=0.0, theta=0
TEST(ParkTransform, BasicCase)
{
    float I_d, I_q;
    
    Park_Transform(1.0f, 0.0f, 0.0f, 1.0f, &I_d, &I_q);
    
    DOUBLES_EQUAL(1.0f, I_d, 0.001f);
    DOUBLES_EQUAL(0.0f, I_q, 0.001f);
}

// Test group for SVPWM
TEST_GROUP(SVPWM)
{
    void setup() {
        // Test setup
    }
    
    void teardown() {
        // Test cleanup
        mock().clear();
    }
};

// Test zero input case
TEST(SVPWM, ZeroInput)
{
    uint32_t Tcm1, Tcm2, Tcm3;
    
    SVPWM(0.0f, 0.0f, &Tcm1, &Tcm2, &Tcm3);
    
    // All PWM values should be around 50% duty cycle
    uint32_t expected_ticks = ARR_PERIOD / 2;
    CHECK(Tcm1 >= expected_ticks - 10 && Tcm1 <= expected_ticks + 10);
    CHECK(Tcm2 >= expected_ticks - 10 && Tcm2 <= expected_ticks + 10);
    CHECK(Tcm3 >= expected_ticks - 10 && Tcm3 <= expected_ticks + 10);
}

// Test group for Sine Cosine calculation
TEST_GROUP(SineCosine)
{
    void setup() {
        // Test setup
    }
    
    void teardown() {
        // Test cleanup
        mock().clear();
    }
};

// Test zero angle case
TEST(SineCosine, ZeroAngle)
{
    float sin_val, cos_val;
    
    Sine_Cosine(0.0f, &sin_val, &cos_val);
    
    DOUBLES_EQUAL(0.0f, sin_val, 0.001f);
    DOUBLES_EQUAL(1.0f, cos_val, 0.001f);
}

// Test group for Low Pass Filter
TEST_GROUP(LowPassFilter)
{
    void setup() {
        // Test setup
    }
    
    void teardown() {
        // Test cleanup
        mock().clear();
    }
};

// Test basic filter functionality
TEST(LowPassFilter, BasicFunctionality)
{
    LPF_Float_t filter = {0};
    
    float output1 = LPF_Filter(&filter, 1.0f, 10.0f, 0.001f, true);
    float output2 = LPF_Filter(&filter, 1.0f, 10.0f, 0.001f, true);
    
    // Second output should be closer to input value
    CHECK(output2 > output1);
    CHECK(output2 <= 1.0f);
}

// Main function for CppUTest
int main(int argc, char** argv)
{
    return CommandLineTestRunner::RunAllTests(argc, argv);
}