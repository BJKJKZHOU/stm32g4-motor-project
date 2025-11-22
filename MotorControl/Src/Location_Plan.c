/*============================================================================
    File Name     : Location_Plan.c
    Description   : 位置规划模块 - 速度模式实现
    Author        : ZHOUHENG
    Date          : 2025-11-22
    ----------------------------------------------------------------------
    混合版接口：核心2函数 + 扩展查询函数
    - VelocityMode_Init：初始化并预计算所有参数
    - VelocityMode_Run：每个控制周期调用，自动状态转换
    - VelocityMode_GetPhase：获取当前阶段
    - VelocityMode_IsFinished：是否完成
    - VelocityMode_GetCurrentVelocity：获取当前速度
*=============================================================================*/

#include "Location_Plan.h"

// ============================================================================
// 内部辅助函数
// ============================================================================

/**
 * @brief 限幅函数
 */
static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 三角型判断并计算峰值速度
 * @param accel 加速度 (rad/s²)
 * @param decel 减速度 (rad/s²)
 * @param v_target 目标速度 (rad/s)
 * @param run_time 运行时间 (s)
 * @param peak_vel 输出：峰值速度 (rad/s)
 * @return true=三角型, false=梯形
 */
static bool is_triangular_profile(float accel, float decel, float v_target,
                                   float run_time, float *peak_vel) {
    float t_accel = v_target / accel;
    float t_decel = v_target / decel;

    // 定时模式：检查时间是否足够
    if (run_time > 0.0f && (t_accel + t_decel) >= run_time) {
        // 三角型：计算峰值速度
        // 推导：v_peak = a * t_accel = d * t_decel
        //       t_accel + t_decel = run_time
        // 解方程：v_peak = (a * d * run_time) / (a + d)
        *peak_vel = (accel * decel * run_time) / (accel + decel);
        return true;
    }

    // 持续模式或时间充足：梯形
    *peak_vel = v_target;
    return false;
}

/**
 * @brief T型曲线速度计算
 * @param vm 速度模式实例指针
 * @param t 当前阶段累计时间 (s)
 * @return 当前速度 (rad/s)
 */
static float compute_trapezoidal_velocity(const VelocityMode_t *vm, float t) {
    if (vm->is_triangular) {
        // 三角型
        if (vm->phase == 0) {
            // 加速段：v = a * t
            return vm->acceleration * t;
        } else if (vm->phase == 2) {
            // 减速段：v = v_peak - d * t
            float v = vm->peak_velocity - vm->deceleration * t;
            return clamp(v, 0.0f, vm->peak_velocity);
        }
    } else {
        // 梯形
        if (vm->phase == 0) {
            // 加速段：v = a * t
            return vm->acceleration * t;
        } else if (vm->phase == 1) {
            // 匀速段：v = v_target
            return vm->target_velocity;
        } else if (vm->phase == 2) {
            // 减速段：v = v_target - d * t
            float v = vm->target_velocity - vm->deceleration * t;
            return clamp(v, 0.0f, vm->target_velocity);
        }
    }

    return 0.0f;
}

/**
 * @brief S型曲线速度计算（简化三次多项式）
 * @param vm 速度模式实例指针
 * @param t 当前阶段累计时间 (s)
 * @return 当前速度 (rad/s)
 */
static float compute_scurve_velocity(const VelocityMode_t *vm, float t) {
    if (vm->is_triangular) {
        // 三角型S曲线
        if (vm->phase == 0) {
            // 加速段：五次多项式平滑
            float t_norm = t / vm->accel_time;
            float t3 = t_norm * t_norm * t_norm;
            float t4 = t3 * t_norm;
            float t5 = t4 * t_norm;
            return vm->peak_velocity * (10.0f * t3 - 15.0f * t4 + 6.0f * t5);
        } else if (vm->phase == 2) {
            // 减速段：镜像对称
            float t_norm = t / vm->decel_time;
            float t3 = t_norm * t_norm * t_norm;
            float t4 = t3 * t_norm;
            float t5 = t4 * t_norm;
            float v = vm->peak_velocity * (1.0f - (10.0f * t3 - 15.0f * t4 + 6.0f * t5));
            return clamp(v, 0.0f, vm->peak_velocity);
        }
    } else {
        // 梯形S曲线
        if (vm->phase == 0) {
            // 加速段：S曲线
            float t_norm = t / vm->accel_time;
            float t3 = t_norm * t_norm * t_norm;
            float t4 = t3 * t_norm;
            float t5 = t4 * t_norm;
            return vm->target_velocity * (10.0f * t3 - 15.0f * t4 + 6.0f * t5);
        } else if (vm->phase == 1) {
            // 匀速段
            return vm->target_velocity;
        } else if (vm->phase == 2) {
            // 减速段：S曲线
            float t_norm = t / vm->decel_time;
            float t3 = t_norm * t_norm * t_norm;
            float t4 = t3 * t_norm;
            float t5 = t4 * t_norm;
            float v = vm->target_velocity * (1.0f - (10.0f * t3 - 15.0f * t4 + 6.0f * t5));
            return clamp(v, 0.0f, vm->target_velocity);
        }
    }

    return 0.0f;
}

// ============================================================================
// 核心接口实现
// ============================================================================

/**
 * @brief 速度模式初始化
 */
void VelocityMode_Init(VelocityMode_t *vm, const VelocityMode_Config_t *config) {
    // 保存配置参数
    // 参数验证和钳位，防止除以零或不合理的值
    vm->acceleration = (config->acceleration > 0.001f) ? config->acceleration : 0.001f; // 最小加速度
    vm->deceleration = (config->deceleration > 0.001f) ? config->deceleration : 0.001f; // 最小减速度
    vm->target_velocity = (config->target_velocity >= 0.0f) ? config->target_velocity : 0.0f; // 目标速度不能为负
    vm->run_time = (config->run_time >= 0.0f) ? config->run_time : 0.0f; // 运行时间不能为负

    vm->use_scurve = config->use_scurve;

    // 预计算时间参数
    vm->accel_time = vm->target_velocity / vm->acceleration;
    vm->decel_time = vm->target_velocity / vm->deceleration;

    // 判断三角型/梯形
    vm->is_triangular = is_triangular_profile(
        vm->acceleration, vm->deceleration, vm->target_velocity,
        vm->run_time, &vm->peak_velocity
    );

    // 计算匀速时间
    if (vm->is_triangular) {
        // 三角型：无匀速段
        vm->cruise_time = 0.0f;
        // 重新计算加速/减速时间（基于峰值速度）
        vm->accel_time = vm->peak_velocity / vm->acceleration;
        vm->decel_time = vm->peak_velocity / vm->deceleration;
    } else {
        // 梯形：计算匀速时间
        if (vm->run_time > 0.0f) {
            // 定时模式：匀速时间 = 总时间 - 加速时间 - 减速时间
            vm->cruise_time = vm->run_time - vm->accel_time - vm->decel_time;
        } else {
            // 持续模式：匀速时间未知（等待停止信号）
            vm->cruise_time = 0.0f;
        }
    }

    // 初始化运行时状态
    if (vm->target_velocity == 0.0f) {
        vm->elapsed_time = 0.0f;
        vm->current_velocity = 0.0f;
        vm->phase = 3;  // 目标速度为0，直接进入停止阶段
    } else {
        vm->elapsed_time = 0.0f;
        vm->current_velocity = 0.0f;
        vm->phase = 0;  // 从加速阶段开始
    }
    vm->stop_requested = false;
    vm->stop_velocity = 0.0f;
}

/**
 * @brief 速度模式运行
 */
float VelocityMode_Run(VelocityMode_t *vm, float dt, bool stop_request) {
    // 阶段3：已停止，直接返回0
    if (vm->phase == 3) {
        vm->current_velocity = 0.0f;
        return 0.0f;
    }

    // 处理停止请求（持续模式）
    if (vm->run_time == 0.0f && stop_request && !vm->stop_requested) {
        vm->stop_requested = true;

        // 立即切换到减速阶段
        if (vm->phase < 2) {
            vm->stop_velocity = vm->current_velocity;  // 保存当前速度
            vm->phase = 2;
            vm->elapsed_time = 0.0f;
            // 重新计算减速时间（从当前速度减速到0）
            vm->decel_time = vm->stop_velocity / vm->deceleration;
        }
    }

    // 更新时间
    vm->elapsed_time += dt;

    // 根据曲线类型计算速度
    if (vm->use_scurve) {
        vm->current_velocity = compute_scurve_velocity(vm, vm->elapsed_time);
    } else {
        vm->current_velocity = compute_trapezoidal_velocity(vm, vm->elapsed_time);
    }

    // 自动状态转换
    if (vm->is_triangular) {
        // 三角型：加速→减速→停止
        if (vm->phase == 0 && vm->elapsed_time >= vm->accel_time) {
            vm->phase = 2;  // 跳过匀速，直接减速
            vm->elapsed_time = 0.0f;
        } else if (vm->phase == 2 && vm->elapsed_time >= vm->decel_time) {
            vm->phase = 3;  // 停止
            vm->current_velocity = 0.0f;
        }
    } else {
        // 梯形：加速→匀速→减速→停止
        if (vm->phase == 0 && vm->elapsed_time >= vm->accel_time) {
            vm->phase = 1;  // 匀速
            vm->elapsed_time = 0.0f;
        } else if (vm->phase == 1) {
            // 定时模式：检查匀速时间
            if (vm->run_time > 0.0f && vm->elapsed_time >= vm->cruise_time) {
                vm->phase = 2;  // 减速
                vm->elapsed_time = 0.0f;
            }
            // 持续模式：等待停止信号（已在上面处理）
        } else if (vm->phase == 2 && vm->elapsed_time >= vm->decel_time) {
            vm->phase = 3;  // 停止
            vm->current_velocity = 0.0f;
        }
    }

    return vm->current_velocity;
}

// ============================================================================
// 扩展查询接口实现
// ============================================================================

/**
 * @brief 获取当前阶段
 */
uint8_t VelocityMode_GetPhase(const VelocityMode_t *vm) {
    return vm->phase;
}

/**
 * @brief 是否完成
 */
bool VelocityMode_IsFinished(const VelocityMode_t *vm) {
    return (vm->phase == 3);
}

/**
 * @brief 获取当前速度
 */
float VelocityMode_GetCurrentVelocity(const VelocityMode_t *vm) {
    return vm->current_velocity;
}



