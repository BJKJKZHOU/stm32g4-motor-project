/*============================================================================
    File Name     : normalization_stub.c
    Description   : Simplified normalization interface for unit tests
    Author        : Codex
    Date          : 2025-11-17
*============================================================================*/

#include "normalization_stub.h"
#include "motor_params.h"  // 添加对motor_params.h的包含

#include <string.h>

static normalization_base_values_t s_bases[motors_number];
static bool s_valid[motors_number];

void NormalizationStub_SetBases(uint8_t motor_id,
                                const normalization_base_values_t *bases)
{
    if (motor_id >= motors_number || bases == NULL) {
        return;
    }

    s_bases[motor_id] = *bases;
    s_valid[motor_id] = true;
}

void NormalizationStub_ClearBases(void)
{
    memset(s_bases, 0, sizeof(s_bases));
    memset(s_valid, 0, sizeof(s_valid));
}

const normalization_base_values_t *Normalization_GetBases(uint8_t motor_id)
{
    if (motor_id < motors_number && s_valid[motor_id]) {
        return &s_bases[motor_id];
    }
    return NULL;
}

void Normalization_UpdateMotor(uint8_t motor_id)
{
    (void)motor_id;
}