/*============================================================================
    File Name     : test_Positioning_cpputest.cpp
    Description   : Position observer unit tests using CppUTest
    Author        : Codex
    Date          : 2025-11-17
*============================================================================*/

#include "CppUTest/TestHarness.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "Positioning.h"
#include "motor_params.h"
}

#include "normalization_stub.h"

#include <string.h>

#define FLOAT_TOLERANCE 1e-5f

static void ConfigureMotorAndBases(float inductance,
                                   float resistance,
                                   float flux,
                                   float time_base)
{
    normalization_base_values_t base = {0};
    base.voltage_base    = 1.0f;
    base.current_base    = 1.0f;
    base.omega_base      = 1.0f;
    base.flux_base       = flux;
    base.torque_base     = 1.0f;
    base.power_base      = 1.0f;
    base.impedance_base  = (resistance != 0.0f) ? resistance : 1.0f;
    base.inductance_base = inductance;
    base.time_base       = time_base;
    base.friction_base   = 1.0f;
    base.inertia_base    = 1.0f;
    NormalizationStub_SetBases(MOTOR_0, &base);

    Motor_Params_t params;
    memset(&params, 0, sizeof(params));
    params.Ld   = inductance;
    params.Lq   = inductance;
    params.Rs   = resistance;
    params.Flux = flux;
    params.Pn   = 1.0f;
    motor_params[MOTOR_0] = params;
}

static void SetTestWindow(NonlinearObs_Position_t *obs, uint16_t window)
{
    obs->stable_required = window;
    obs->stable_counter = 0;
}

TEST_GROUP(NonlinearObserver)
{
    NonlinearObs_Position_t observer;

    void setup()
    {
        memset(&observer, 0, sizeof(observer));
        memset(motor_params, 0, sizeof(motor_params));
        NormalizationStub_ClearBases();
        MotorParams_SetActiveMotor(MOTOR_0);
    }

    void teardown()
    {
        mock().clear();
        NormalizationStub_ClearBases();
    }

    void InitObserver(float gamma = 0.0f)
    {
        NonlinearObs_Position_Init(&observer, MOTOR_0, gamma);
        CHECK_TRUE(observer.is_initialized);
        SetTestWindow(&observer, 5U);
    }
};

TEST(NonlinearObserver, AppliesPerUnitTimeStep)
{
    ConfigureMotorAndBases(1e-3f, 0.0f, 0.05f, 1e-3f);
    InitObserver(0.0f);

    // dt_pu = 0.0001 / 0.001 = 0.1, y_alpha = 1 => Δx_hat_alpha = 0.1
    NonlinearObs_Position_Update(&observer,
                                 0.0f, 0.0f,
                                 1.0f, 0.0f,
                                 1e-4f);

    DOUBLES_EQUAL(0.1f, observer.x_hat_alpha_pu, FLOAT_TOLERANCE);
}

static void RunIdleStep(NonlinearObs_Position_t *obs, float v_alpha_pu = 1e-4f)
{
    NonlinearObs_Position_Update(obs,
                                 0.0f, 0.0f,
                                 v_alpha_pu, 0.0f,
                                 1e-4f);
}

TEST(NonlinearObserver, RequiresStableWindowBeforeConvergence)
{
    ConfigureMotorAndBases(1e-3f, 0.0f, 0.05f, 1e-3f);
    InitObserver(0.0f);

    for (uint16_t i = 0; i < observer.stable_required - 1; ++i) {
        RunIdleStep(&observer);
    }
    CHECK_FALSE(observer.is_converged);

    RunIdleStep(&observer);

    CHECK_TRUE(observer.is_converged);
}

TEST(NonlinearObserver, ResetsConvergenceWhenErrorSpikes)
{
    ConfigureMotorAndBases(1e-3f, 0.0f, 0.05f, 1e-3f);
    InitObserver(0.0f);

    for (uint16_t i = 0; i < observer.stable_required; ++i) {
        RunIdleStep(&observer);
    }
    CHECK_TRUE(observer.is_converged);

    NonlinearObs_Position_Update(&observer,
                                 0.0f, 0.0f,
                                 5.0f, 0.0f,
                                 1e-4f);

    CHECK_FALSE(observer.is_converged);
}
