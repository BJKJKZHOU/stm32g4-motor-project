/* ======================================================================
 *  Q31格式支持: 使用arm_math库的q31_t数据类型
 *  自动归一化: 将各种电机参数归一化到[-1,1]或[0,1]范围
 *  可配置范围: 支持自定义各参数的最大值
 *  双向转换: 支持浮点到Q31的归一化转换和反向反归一化
 *  范围保护: 自动限制输入值在有效范围内，防止溢出
 *  可自定义归一化格式如，Q12,Q31等
 * ====================================================================== */

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
}

static bool get_base_pair(uint8_t motor_id,
                          normalization_quantity_t quantity,
                          float *base,
                          float *inv)
{
    if (motor_id >= motors_number) {
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
    default:
        return false;
    }
}

void Normalization_Init(void)
{
    for (uint8_t i = 0; i < motors_number; ++i) {
        Normalization_UpdateMotor(i);
    }
}

void Normalization_UpdateMotor(uint8_t motor_id)
{
    if (motor_id >= motors_number) {
        return;
    }

    normalization_motor_ctx_t *ctx = &s_norm_ctx[motor_id];
    ctx->valid = compute_bases(motor_id, &ctx->base);
    if (ctx->valid) {
        compute_inverses(&ctx->base, &ctx->inv);
    } else {
        memset(ctx, 0, sizeof(*ctx));
    }
}

const normalization_base_values_t *Normalization_GetBases(uint8_t motor_id)
{
    if (motor_id >= motors_number || !s_norm_ctx[motor_id].valid) {
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
