/*============================================================================
    File Name     : normalization.c
    Description   : 归一化模块 - 包含归一化相关的函数
    Author        : ZHOUHENG
    Date          : 2025-11-04
    ----------------------------------------------------------------------       

*=============================================================================
*/


#include "normalization.h"

#include <math.h>
#include <string.h>

#define NORMALIZATION_CLAMP_MIN (-1.0f)
#define NORMALIZATION_CLAMP_MAX (1.0f)
#define SQRT3_F (1.7320508075688772f)

typedef struct {
    normalization_base_values_t base;
    normalization_base_values_t inv;
    bool valid;
} normalization_motor_ctx_t;

static normalization_motor_ctx_t s_norm_ctx[motors_number];

static void clear_motor_ctx(normalization_motor_ctx_t *ctx)
{
    if (ctx != NULL) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

static bool compute_bases(uint8_t motor_id, normalization_base_values_t *out)
{
    const Motor_Params_t *params = &motor_params[motor_id];
    const Motor_LimitParams_t *limits = &motor_limit_params[motor_id];

    const float voltage_base = params->V_DC / SQRT3_F;
    const float current_base =
        (limits->I_limit_actual > 0.0f) ? limits->I_limit_actual :
        (limits->I_limit_max   > 0.0f) ? limits->I_limit_max   :
        (params->I_rated       > 0.0f) ? params->I_rated       : 0.0f;
    const float rpm = (limits->speed_limit_actual > 0.0f) ?
                      limits->speed_limit_actual : params->RPM_rated;
    const float omega_base = (2.0f * PI * rpm / 60.0f) * params->Pn;

    if (voltage_base <= 0.0f || current_base <= 0.0f ||
        omega_base <= 0.0f || params->Flux <= 0.0f || params->Pn <= 0.0f) {
        return false;
    }

    out->voltage_base    = voltage_base;
    out->current_base    = current_base;
    out->omega_base      = omega_base;
    out->flux_base       = voltage_base / omega_base;
    out->torque_base     = 1.5f * params->Pn * params->Flux * current_base;
    out->power_base      = 1.5f * current_base * voltage_base;
    out->impedance_base  = voltage_base / current_base;
    out->inductance_base = out->flux_base / current_base;
    out->time_base       = 1.0f / omega_base;
    out->friction_base   = out->power_base * (params->Pn * params->Pn) / (out->omega_base * out->omega_base);
    out->inertia_base    = 2.0f * out->power_base * (params->Pn * params->Pn) / (out->omega_base * out->omega_base);
    return true;
}

static void compute_inverses(const normalization_base_values_t *base,
                             normalization_base_values_t *inv)
{
    inv->voltage_base    = base->voltage_base    ? 1.0f / base->voltage_base    : 0.0f;
    inv->current_base    = base->current_base    ? 1.0f / base->current_base    : 0.0f;
    inv->omega_base      = base->omega_base      ? 1.0f / base->omega_base      : 0.0f;
    inv->flux_base       = base->flux_base       ? 1.0f / base->flux_base       : 0.0f;
    inv->torque_base     = base->torque_base     ? 1.0f / base->torque_base     : 0.0f;
    inv->power_base      = base->power_base      ? 1.0f / base->power_base      : 0.0f;
    inv->impedance_base  = base->impedance_base  ? 1.0f / base->impedance_base  : 0.0f;
    inv->inductance_base = base->inductance_base ? 1.0f / base->inductance_base : 0.0f;
    inv->time_base       = base->time_base       ? 1.0f / base->time_base       : 0.0f;
    inv->friction_base   = base->friction_base   ? 1.0f / base->friction_base   : 0.0f;
    inv->inertia_base    = base->inertia_base    ? 1.0f / base->inertia_base    : 0.0f;
}

static bool get_base_pair(uint8_t motor_id,
                          normalization_quantity_t quantity,
                          float *base,
                          float *inv)
{
    if (motor_id >= motors_number) {
        return false;
    }

    if (!MotorParams_IsMotorEnabled(motor_id)) {
        return false;
    }

    const normalization_motor_ctx_t *ctx = &s_norm_ctx[motor_id];
    if (!ctx->valid) {
        return false;
    }

    switch (quantity) {
    case NORMALIZE_VOLTAGE:
        *base = ctx->base.voltage_base;
        *inv  = ctx->inv.voltage_base;
        return true;
    case NORMALIZE_CURRENT:
        *base = ctx->base.current_base;
        *inv  = ctx->inv.current_base;
        return true;
    case NORMALIZE_SPEED_ELECTRICAL:
        *base = ctx->base.omega_base;
        *inv  = ctx->inv.omega_base;
        return true;
    case NORMALIZE_FLUX:
        *base = ctx->base.flux_base;
        *inv  = ctx->inv.flux_base;
        return true;
    case NORMALIZE_TORQUE:
        *base = ctx->base.torque_base;
        *inv  = ctx->inv.torque_base;
        return true;
    case NORMALIZE_POWER:
        *base = ctx->base.power_base;
        *inv  = ctx->inv.power_base;
        return true;
    case NORMALIZE_IMPEDANCE:
        *base = ctx->base.impedance_base;
        *inv  = ctx->inv.impedance_base;
        return true;
    case NORMALIZE_INDUCTANCE:
        *base = ctx->base.inductance_base;
        *inv  = ctx->inv.inductance_base;
        return true;
    case NORMALIZE_TIME:
        *base = ctx->base.time_base;
        *inv  = ctx->inv.time_base;
        return true;
    case NORMALIZE_FRICTION:
        *base = ctx->base.friction_base;
        *inv  = ctx->inv.friction_base;
        return true;
    case NORMALIZE_INERTIA:
        *base = ctx->base.inertia_base;
        *inv  = ctx->inv.inertia_base;
        return true;
    default:
        return false;
    }
}

void Normalization_Init(void)
{
    for (uint8_t i = 0; i < motors_number; ++i) {
        clear_motor_ctx(&s_norm_ctx[i]);
    }
}

void Normalization_UpdateMotor(uint8_t motor_id)
{
    if (motor_id >= motors_number) {
        return;
    }

    // 只对激活的电机参数套更新归一化基值
    if (MotorParams_IsMotorEnabled(motor_id)) {
        normalization_motor_ctx_t *ctx = &s_norm_ctx[motor_id];
        ctx->valid = compute_bases(motor_id, &ctx->base);
        if (ctx->valid) {
            compute_inverses(&ctx->base, &ctx->inv);
        } else {
            memset(ctx, 0, sizeof(*ctx));
        }
    }
}

const normalization_base_values_t *Normalization_GetBases(uint8_t motor_id)
{
    // 只返回激活电机的归一化基值
    if (motor_id >= motors_number || !MotorParams_IsMotorEnabled(motor_id) || !s_norm_ctx[motor_id].valid) {
        return NULL;
    }
    return &s_norm_ctx[motor_id].base;
}

float Normalization_ToPerUnit(uint8_t motor_id,
                              normalization_quantity_t quantity,
                              float value)
{
    float base, inv;
    if (!get_base_pair(motor_id, quantity, &base, &inv)) {
        return 0.0f;
    }

    float pu = value * inv;
    if (pu > NORMALIZATION_CLAMP_MAX) pu = NORMALIZATION_CLAMP_MAX;
    if (pu < NORMALIZATION_CLAMP_MIN) pu = NORMALIZATION_CLAMP_MIN;
    return pu;
}

float Normalization_FromPerUnit(uint8_t motor_id,
                                normalization_quantity_t quantity,
                                float pu_value)
{
    float base, inv;
    if (!get_base_pair(motor_id, quantity, &base, &inv)) {
        return 0.0f;
    }

    if (pu_value > NORMALIZATION_CLAMP_MAX) pu_value = NORMALIZATION_CLAMP_MAX;
    if (pu_value < NORMALIZATION_CLAMP_MIN) pu_value = NORMALIZATION_CLAMP_MIN;
    return pu_value * base;
}

q31_t Normalization_ToQ31(uint8_t motor_id,
                          normalization_quantity_t quantity,
                          float value)
{
    float pu = Normalization_ToPerUnit(motor_id, quantity, value);
    q31_t qvalue;
    arm_float_to_q31(&pu, &qvalue, 1);
    return qvalue;
}

float Normalization_FromQ31(uint8_t motor_id,
                            normalization_quantity_t quantity,
                            q31_t value)
{
    float pu;
    arm_q31_to_float(&value, &pu, 1);
    return Normalization_FromPerUnit(motor_id, quantity, pu);
}

 
float Normalization_ToPerUnitWithBase(float value, float base)
{
    if (base == 0.0f) {
        return 0.0f;
    }
    
    float pu = value / base;
    if (pu > NORMALIZATION_CLAMP_MAX) pu = NORMALIZATION_CLAMP_MAX;
    if (pu < NORMALIZATION_CLAMP_MIN) pu = NORMALIZATION_CLAMP_MIN;
    return pu;
}

q31_t Normalization_ToQ31WithBase(float value, float base)
{
    float pu = Normalization_ToPerUnitWithBase(value, base);
    q31_t qvalue;
    arm_float_to_q31(&pu, &qvalue, 1);
    return qvalue;
}

float Normalization_FromPerUnitWithBase(float pu_value, float base)
{
    if (pu_value > NORMALIZATION_CLAMP_MAX) pu_value = NORMALIZATION_CLAMP_MAX;
    if (pu_value < NORMALIZATION_CLAMP_MIN) pu_value = NORMALIZATION_CLAMP_MIN;
    return pu_value * base;
}

float Normalization_FromQ31WithBase(q31_t value, float base)
{
    float pu;
    arm_q31_to_float(&value, &pu, 1);
    return Normalization_FromPerUnitWithBase(pu, base);
}
