/*============================================================================
    File Name     : normalization_stub.h
    Description   : Helpers for configuring normalization bases in tests
    Author        : Codex
    Date          : 2025-11-17
*============================================================================*/

#ifndef NORMALIZATION_STUB_H
#define NORMALIZATION_STUB_H

#include <stdbool.h>
#include <stdint.h>

#include "normalization.h"

#ifdef __cplusplus
extern "C" {
#endif

void NormalizationStub_SetBases(uint8_t motor_id,
                                const normalization_base_values_t *bases);
void NormalizationStub_ClearBases(void);

#ifdef __cplusplus
}
#endif

#endif /* NORMALIZATION_STUB_H */
