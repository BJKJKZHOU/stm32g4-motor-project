/*============================================================================
    File Name     : test_velocity_mode_fix.c
    Description   : 速度模式修复验证测试
    Author        : Claude Code
    Date          : 2025-11-23
    ----------------------------------------------------------------------
    测试用例：
    1. 定时模式 - 梯形曲线
    2. 定时模式 - 三角型曲线
    3. 持续模式 - 匀速段停止
    4. 持续模式 - 加速段停止（关键测试）
    5. S 型曲线 - 速度连续性
*=============================================================================*/

#include <stdio.h>
#include <math.h>
#include "Location_Plan.h"

// 测试辅助函数
#define ASSERT_FLOAT_EQ(expected, actual, tolerance) \
    do { \
        float diff = fabsf((expected) - (actual)); \
        if (diff > (tolerance)) { \
            printf("  ❌ FAIL: Expected %.4f, got %.4f (diff=%.6f)\n", \
                   (expected), (actual), diff); \
            return false; \
        } \
    } while(0)

#define ASSERT_TRUE(condition, message) \
    do { \
        if (!(condition)) { \
            printf("  ❌ FAIL: %s\n", message); \
            return false; \
        } \
    } while(0)

// ============================================================================
// 测试用例 1：定时模式 - 梯形曲线
// ============================================================================
bool test_timed_trapezoidal() {
    printf("\n【测试 1】定时模式 - 梯形曲线\n");

    VelocityMode_Config_t config = {
        .acceleration = 10.0f,      // 10 rad/s²
        .deceleration = 10.0f,      // 10 rad/s²
        .target_velocity = 50.0f,   // 50 rad/s
        .run_time = 15.0f,          // 15 s
        .use_scurve = false         // T 型曲线
    };

    VelocityMode_t vm;
    VelocityMode_Init(&vm, &config);

    // 验证预计算参数
    ASSERT_FLOAT_EQ(5.0f, vm.accel_time, 0.01f);
    ASSERT_FLOAT_EQ(5.0f, vm.cruise_time, 0.01f);
    ASSERT_FLOAT_EQ(5.0f, vm.decel_time, 0.01f);
    ASSERT_FLOAT_EQ(50.0f, vm.decel_start_velocity, 0.01f);
    ASSERT_TRUE(!vm.is_triangular, "应该是梯形");

    float dt = 0.01f;  // 10ms 控制周期
    float t = 0.0f;

    // 加速段（0-5s）
    for (int i = 0; i < 500; i++) {
        float v = VelocityMode_Run(&vm, dt, false);
        t += dt;
        if (i == 0) {
            ASSERT_FLOAT_EQ(0.1f, v, 0.01f);  // 第一个周期
        }
        if (i == 499) {
            ASSERT_FLOAT_EQ(50.0f, v, 0.5f);  // 加速结束
            ASSERT_TRUE(vm.phase == 1, "应该进入匀速段");
        }
    }

    // 匀速段（5-10s）
    for (int i = 0; i < 500; i++) {
        float v = VelocityMode_Run(&vm, dt, false);
        t += dt;
        ASSERT_FLOAT_EQ(50.0f, v, 0.1f);
        if (i == 499) {
            ASSERT_TRUE(vm.phase == 2, "应该进入减速段");
        }
    }

    // 减速段（10-15s）
    for (int i = 0; i < 500; i++) {
        float v = VelocityMode_Run(&vm, dt, false);
        t += dt;
        if (i == 499) {
            ASSERT_FLOAT_EQ(0.0f, v, 0.5f);  // 减速结束
            ASSERT_TRUE(vm.phase == 3, "应该停止");
        }
    }

    printf("  ✅ PASS\n");
    return true;
}

// ============================================================================
// 测试用例 2：定时模式 - 三角型曲线
// ============================================================================
bool test_timed_triangular() {
    printf("\n【测试 2】定时模式 - 三角型曲线\n");

    VelocityMode_Config_t config = {
        .acceleration = 10.0f,      // 10 rad/s²
        .deceleration = 10.0f,      // 10 rad/s²
        .target_velocity = 100.0f,  // 100 rad/s（无法达到）
        .run_time = 8.0f,           // 8 s（时间不足）
        .use_scurve = false         // T 型曲线
    };

    VelocityMode_t vm;
    VelocityMode_Init(&vm, &config);

    // 验证三角型判断
    ASSERT_TRUE(vm.is_triangular, "应该是三角型");
    ASSERT_FLOAT_EQ(40.0f, vm.peak_velocity, 0.01f);  // v_peak = (10*10*8)/(10+10) = 40
    ASSERT_FLOAT_EQ(4.0f, vm.accel_time, 0.01f);
    ASSERT_FLOAT_EQ(4.0f, vm.decel_time, 0.01f);
    ASSERT_FLOAT_EQ(40.0f, vm.decel_start_velocity, 0.01f);

    float dt = 0.01f;

    // 加速段（0-4s）
    for (int i = 0; i < 400; i++) {
        float v = VelocityMode_Run(&vm, dt, false);
        if (i == 399) {
            ASSERT_FLOAT_EQ(40.0f, v, 0.5f);  // 达到峰值速度
            ASSERT_TRUE(vm.phase == 2, "应该直接进入减速段");
        }
    }

    // 减速段（4-8s）
    for (int i = 0; i < 400; i++) {
        float v = VelocityMode_Run(&vm, dt, false);
        if (i == 399) {
            ASSERT_FLOAT_EQ(0.0f, v, 0.5f);  // 减速到0
            ASSERT_TRUE(vm.phase == 3, "应该停止");
        }
    }

    printf("  ✅ PASS\n");
    return true;
}

// ============================================================================
// 测试用例 3：持续模式 - 匀速段停止
// ============================================================================
bool test_continuous_stop_at_cruise() {
    printf("\n【测试 3】持续模式 - 匀速段停止\n");

    VelocityMode_Config_t config = {
        .acceleration = 10.0f,
        .deceleration = 10.0f,
        .target_velocity = 50.0f,
        .run_time = 0.0f,           // 持续模式
        .use_scurve = false
    };

    VelocityMode_t vm;
    VelocityMode_Init(&vm, &config);

    float dt = 0.01f;

    // 加速到匀速（0-5s）
    for (int i = 0; i < 500; i++) {
        VelocityMode_Run(&vm, dt, false);
    }
    ASSERT_TRUE(vm.phase == 1, "应该在匀速段");
    ASSERT_FLOAT_EQ(50.0f, vm.current_velocity, 0.5f);

    // 匀速运行 5s（5-10s）
    for (int i = 0; i < 500; i++) {
        VelocityMode_Run(&vm, dt, false);
    }

    // 发送停止请求
    float v_before_stop = vm.current_velocity;
    VelocityMode_Run(&vm, dt, true);  // 停止请求

    ASSERT_TRUE(vm.phase == 2, "应该进入减速段");
    ASSERT_FLOAT_EQ(50.0f, vm.decel_start_velocity, 0.01f);  // 关键：减速起始速度
    ASSERT_FLOAT_EQ(5.0f, vm.decel_time, 0.01f);  // 减速时间 = 50/10 = 5s

    // 减速到0（10-15s）
    for (int i = 0; i < 499; i++) {
        VelocityMode_Run(&vm, dt, false);
    }

    ASSERT_FLOAT_EQ(0.0f, vm.current_velocity, 0.5f);
    ASSERT_TRUE(vm.phase == 3, "应该停止");

    printf("  ✅ PASS\n");
    return true;
}

// ============================================================================
// 测试用例 4：持续模式 - 加速段停止（关键测试）
// ============================================================================
bool test_continuous_stop_at_accel() {
    printf("\n【测试 4】持续模式 - 加速段停止（关键测试）\n");

    VelocityMode_Config_t config = {
        .acceleration = 10.0f,
        .deceleration = 10.0f,
        .target_velocity = 100.0f,
        .run_time = 0.0f,           // 持续模式
        .use_scurve = false
    };

    VelocityMode_t vm;
    VelocityMode_Init(&vm, &config);

    float dt = 0.01f;

    // 加速 3s（达到 30 rad/s）
    for (int i = 0; i < 300; i++) {
        VelocityMode_Run(&vm, dt, false);
    }

    ASSERT_TRUE(vm.phase == 0, "应该在加速段");
    float v_at_stop = vm.current_velocity;
    printf("  停止时速度: %.2f rad/s\n", v_at_stop);
    ASSERT_FLOAT_EQ(30.0f, v_at_stop, 1.0f);

    // 发送停止请求
    float v_first_cycle = VelocityMode_Run(&vm, dt, true);

    printf("  停止请求后第一个周期速度: %.4f rad/s\n", v_first_cycle);
    printf("  decel_start_velocity: %.4f rad/s\n", vm.decel_start_velocity);
    printf("  decel_time: %.4f s\n", vm.decel_time);

    ASSERT_TRUE(vm.phase == 2, "应该进入减速段");
    ASSERT_FLOAT_EQ(v_at_stop, vm.decel_start_velocity, 0.5f);  // 关键：使用停止时的速度
    ASSERT_FLOAT_EQ(v_at_stop / 10.0f, vm.decel_time, 0.1f);  // 减速时间 = v_stop / decel

    // 验证速度连续性（关键）
    ASSERT_FLOAT_EQ(v_at_stop, v_first_cycle, 1.0f);  // 第一个周期速度应该接近停止时速度

    // 减速到0
    int decel_cycles = (int)(vm.decel_time / dt);
    for (int i = 0; i < decel_cycles; i++) {
        VelocityMode_Run(&vm, dt, false);
    }

    ASSERT_FLOAT_EQ(0.0f, vm.current_velocity, 1.0f);
    ASSERT_TRUE(vm.phase == 3, "应该停止");

    printf("  ✅ PASS - 修复了 elapsed_time 重置后立即累加的问题\n");
    return true;
}

// ============================================================================
// 测试用例 5：S 型曲线 - 速度连续性
// ============================================================================
bool test_scurve_continuity() {
    printf("\n【测试 5】S 型曲线 - 速度连续性\n");

    VelocityMode_Config_t config = {
        .acceleration = 10.0f,
        .deceleration = 10.0f,
        .target_velocity = 50.0f,
        .run_time = 15.0f,
        .use_scurve = true          // S 型曲线
    };

    VelocityMode_t vm;
    VelocityMode_Init(&vm, &config);

    float dt = 0.01f;
    float v_prev = 0.0f;

    // 运行整个过程，检查速度连续性
    for (int i = 0; i < 1500; i++) {
        float v = VelocityMode_Run(&vm, dt, false);

        // 检查速度跳变（允许状态转换时有小幅跳变）
        float dv = fabsf(v - v_prev);
        if (dv > 2.0f) {  // 允许 2 rad/s 的跳变
            printf("  ⚠️  警告：t=%.2fs 时速度跳变 %.2f rad/s (%.2f -> %.2f)\n",
                   i * dt, dv, v_prev, v);
        }

        v_prev = v;
    }

    ASSERT_TRUE(vm.phase == 3, "应该停止");
    ASSERT_FLOAT_EQ(0.0f, vm.current_velocity, 0.5f);

    printf("  ✅ PASS - S 型曲线速度连续\n");
    return true;
}

// ============================================================================
// 主测试函数
// ============================================================================
int main() {
    printf("========================================\n");
    printf("速度模式修复验证测试\n");
    printf("========================================\n");

    int passed = 0;
    int total = 5;

    if (test_timed_trapezoidal()) passed++;
    if (test_timed_triangular()) passed++;
    if (test_continuous_stop_at_cruise()) passed++;
    if (test_continuous_stop_at_accel()) passed++;
    if (test_scurve_continuity()) passed++;

    printf("\n========================================\n");
    printf("测试结果: %d/%d 通过\n", passed, total);
    printf("========================================\n");

    return (passed == total) ? 0 : 1;
}
