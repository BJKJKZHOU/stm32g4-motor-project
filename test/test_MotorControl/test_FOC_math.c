/*============================================================================
    File Name     : test_FOC_math_simple.c
    Description   : Simple unit tests for FOC math module (without CppUTest)
    Author        : ZHOUHENG
    Date          : 2025-11-10
    ----------------------------------------------------------------------       
    Note:
        This is a simplified test version that does not depend on CppUTest
        Used to verify basic functionality of the test environment
=============================================================================*/

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Include FOC_math header
#include "FOC_math.h"
#include "stm32_hal_stubs.h"

// Test statistics
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

// Test macros
#define TEST_ASSERT(condition, message) \
    do { \
        tests_run++; \
        if (condition) { \
            tests_passed++; \
            printf("PASS: %s\n", message); \
        } else { \
            tests_failed++; \
            printf("FAIL: %s\n", message); \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_EQUAL(expected, actual, tolerance, message) \
    do { \
        tests_run++; \
        if (fabsf((expected) - (actual)) < (tolerance)) { \
            tests_passed++; \
            printf("PASS: %s (expected: %.6f, actual: %.6f)\n", message, expected, actual); \
        } else { \
            tests_failed++; \
            printf("FAIL: %s (expected: %.6f, actual: %.6f)\n", message, expected, actual); \
        } \
    } while(0)

// Helper function: Check if two Q31 numbers are approximately equal
static bool q31_equals(q31_t a, q31_t b, q31_t tolerance)
{
    q31_t diff = (a > b) ? (a - b) : (b - a);
    return diff < tolerance;
}

#define TEST_ASSERT_Q31_EQUAL(expected, actual, tolerance, message) \
    do { \
        tests_run++; \
        if (q31_equals((expected), (actual), (tolerance))) { \
            tests_passed++; \
            printf("PASS: %s\n", message); \
        } else { \
            tests_failed++; \
            printf("FAIL: %s (expected: %d, actual: %d)\n", message, expected, actual); \
        } \
    } while(0)

// Test function declarations
void test_inverse_park_transform_basic(void);
void test_inverse_park_transform_90_degrees(void);
void test_clarke_transform_basic(void);
void test_clarke_transform_zero_input(void);
void test_park_transform_basic(void);
void test_svpwm_zero_input(void);
void test_sine_cosine_zero_angle(void);
void test_lpf_filter_basic(void);

int main(void)
{
    printf("========================================\n");
    printf("FOC_math Module Unit Tests (Simple Version)\n");
    printf("========================================\n\n");
    
    // Run all tests
    test_inverse_park_transform_basic();
    test_inverse_park_transform_90_degrees();
    test_clarke_transform_basic();
    test_clarke_transform_zero_input();
    test_park_transform_basic();
    test_svpwm_zero_input();
    test_sine_cosine_zero_angle();
    test_lpf_filter_basic();
    
    // Output test results
    printf("\n========================================\n");
    printf("Test Results Summary:\n");
    printf("Total Tests: %d\n", tests_run);
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("Success Rate: %.1f%%\n", tests_run > 0 ? (100.0f * tests_passed / tests_run) : 0.0f);
    printf("========================================\n");
    
    return (tests_failed == 0) ? 0 : 1;
}

void test_inverse_park_transform_basic(void)
{
    printf("\n--- Test Inverse Park Transform (Basic) ---\n");
    float U_alpha, U_beta;
    
    // Test basic case: U_d=1.0, U_q=0.0, theta=0
    Inverse_Park_Transform(1.0f, 0.0f, 0.0f, 1.0f, &U_alpha, &U_beta);
    
    TEST_ASSERT_FLOAT_EQUAL(1.0f, U_alpha, 0.001f, "U_alpha = 1.0 when U_d=1.0, U_q=0.0, theta=0");
    TEST_ASSERT_FLOAT_EQUAL(0.0f, U_beta, 0.001f, "U_beta = 0.0 when U_d=1.0, U_q=0.0, theta=0");
}

void test_inverse_park_transform_90_degrees(void)
{
    printf("\n--- Test Inverse Park Transform (90 degrees) ---\n");
    float U_alpha, U_beta;
    
    // Test 90 degree case: U_d=1.0, U_q=0.0, theta=90deg
    Inverse_Park_Transform(1.0f, 0.0f, 1.0f, 0.0f, &U_alpha, &U_beta);
    
    TEST_ASSERT_FLOAT_EQUAL(0.0f, U_alpha, 0.001f, "U_alpha = 0.0 when U_d=1.0, U_q=0.0, theta=90deg");
    TEST_ASSERT_FLOAT_EQUAL(1.0f, U_beta, 0.001f, "U_beta = 1.0 when U_d=1.0, U_q=0.0, theta=90deg");
}

void test_clarke_transform_basic(void)
{
    printf("\n--- Test Clarke Transform (Basic) ---\n");
    float I_alpha, I_beta;
    
    // Test basic case: ia=1.0, ib=-0.5
    bool result = Clarke_Transform(1.0f, -0.5f, &I_alpha, &I_beta);
    
    TEST_ASSERT(result, "Clarke_Transform returns true for valid input");
    TEST_ASSERT_FLOAT_EQUAL(1.0f, I_alpha, 0.001f, "I_alpha = ia = 1.0");
    
    // Calculate expected I_beta: (ia + 2*ib) * 1/sqrt(3)
    float expected_beta = (1.0f + 2.0f * (-0.5f)) * 0.5773502691896258f;
    TEST_ASSERT_FLOAT_EQUAL(expected_beta, I_beta, 0.001f, "I_beta calculation");
}

void test_clarke_transform_zero_input(void)
{
    printf("\n--- Test Clarke Transform (Zero Input) ---\n");
    float I_alpha, I_beta;
    
    bool result = Clarke_Transform(0.0f, 0.0f, &I_alpha, &I_beta);
    
    TEST_ASSERT(result, "Clarke_Transform returns true for zero input");
    TEST_ASSERT_FLOAT_EQUAL(0.0f, I_alpha, 0.001f, "I_alpha = 0.0 for zero input");
    TEST_ASSERT_FLOAT_EQUAL(0.0f, I_beta, 0.001f, "I_beta = 0.0 for zero input");
}

void test_park_transform_basic(void)
{
    printf("\n--- Test Park Transform (Basic) ---\n");
    float I_d, I_q;
    
    // Test basic case: I_alpha=1.0, I_beta=0.0, theta=0
    Park_Transform(1.0f, 0.0f, 0.0f, 1.0f, &I_d, &I_q);
    
    TEST_ASSERT_FLOAT_EQUAL(1.0f, I_d, 0.001f, "I_d = 1.0 when I_alpha=1.0, I_beta=0.0, theta=0");
    TEST_ASSERT_FLOAT_EQUAL(0.0f, I_q, 0.001f, "I_q = 0.0 when I_alpha=1.0, I_beta=0.0, theta=0");
}

void test_svpwm_zero_input(void)
{
    printf("\n--- Test SVPWM (Zero Input) ---\n");
    uint32_t Tcm1, Tcm2, Tcm3;
    
    // Test zero input
    SVPWM(0.0f, 0.0f, &Tcm1, &Tcm2, &Tcm3);
    
    // All PWM values should be around 50% duty cycle
    uint32_t expected_ticks = ARR_PERIOD / 2;
    TEST_ASSERT(Tcm1 >= expected_ticks - 10 && Tcm1 <= expected_ticks + 10, 
                "Tcm1 should be around 50%% duty cycle for zero input");
    TEST_ASSERT(Tcm2 >= expected_ticks - 10 && Tcm2 <= expected_ticks + 10, 
                "Tcm2 should be around 50%% duty cycle for zero input");
    TEST_ASSERT(Tcm3 >= expected_ticks - 10 && Tcm3 <= expected_ticks + 10, 
                "Tcm3 should be around 50%% duty cycle for zero input");
}

void test_sine_cosine_zero_angle(void)
{
    printf("\n--- Test Sine Cosine Calculation (Zero Angle) ---\n");
    float sin_val, cos_val;
    
    Sine_Cosine(0.0f, &sin_val, &cos_val);
    
    TEST_ASSERT_FLOAT_EQUAL(0.0f, sin_val, 0.001f, "sin(0) = 0.0");
    TEST_ASSERT_FLOAT_EQUAL(1.0f, cos_val, 0.001f, "cos(0) = 1.0");
}

void test_lpf_filter_basic(void)
{
    printf("\n--- Test Low Pass Filter (Basic) ---\n");
    LPF_Float_t filter = {0};
    
    // Test basic filter functionality
    float output1 = LPF_Filter(&filter, 1.0f, 10.0f, 0.001f, true);
    float output2 = LPF_Filter(&filter, 1.0f, 10.0f, 0.001f, true);
    
    // Second output should be closer to input value
    TEST_ASSERT(output2 > output1, "LPF output should converge to input value");
    TEST_ASSERT(output2 <= 1.0f, "LPF output should not exceed input value");
}