/*============================================================================
    File Name     : motor_params_stub.c
    Description   : Minimal motor parameter stubs for unit testing
    Author        : Codex
    Date          : 2025-11-17
    ----------------------------------------------------------------------
    Provides bare definitions required by unit tests so we don't need the
    full motor parameter module implementation.
*============================================================================*/

#include "motor_params.h"

#include <stdio.h>

Motor_Params_t motor_params[motors_number];
Motor_LimitParams_t motor_limit_params[motors_number];

uint8_t g_active_motor_id = MOTOR_0;

void MotorParams_Init(void) {}

void MotorParams_SetParam(uint8_t motor_id, const char* param_name, float value)
{
    (void)motor_id;
    (void)param_name;
    (void)value;
}

void MotorParams_PrintAll(uint8_t motor_id)
{
    (void)motor_id;
}

bool MotorParams_IsMotorEnabled(uint8_t motor_id)
{
    (void)motor_id;
    return true;
}

void MotorParams_SetActiveMotor(uint8_t motor_id)
{
    if (motor_id < motors_number) {
        g_active_motor_id = motor_id;
    }
}

void MotorParams_DisableMotor(uint8_t motor_id)
{
    (void)motor_id;
}

uint8_t MotorParams_GetActiveMotor(void)
{
    return g_active_motor_id;
}

bool MotorParams_IsAnyMotorActive(void)
{
    return true;
}

void format_float_value(char* buffer, int buffer_size, float value)
{
    if (buffer == NULL || buffer_size <= 0) {
        return;
    }
    if (buffer_size < 2) {
        buffer[0] = '\0';
        return;
    }
    (void)snprintf(buffer, (size_t)buffer_size, "%.3f", (double)value);
}
