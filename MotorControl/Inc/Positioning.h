/*============================================================================
    File Name     : Positioning.h
    Description   : 电机位置相关 
    Author        : ZHOUHENG
    Date          : 2025-11-12
    ----------------------------------------------------------------------       
    ## 主要是根据电流信号来判断电机的位置
    1、转子初始位置估计，磁极，相位。
    2、位置观测。
    3、位置传感器的上层模块，暂时没有位置传感器。

    ### 脉冲法(IPD)实现转子初始定位

        **原理说明：**
        IPD（Initial Position Detection）通过在不同角度施加短时电压脉冲，
        测量电流响应来确定转子磁极位置。由于磁阻效应，当脉冲方向与转子
        N极对齐时，电流响应最大。

        **实现步骤：**
        1、在 d 轴发送特定角度的电压脉冲，12个脉冲，每个脉冲角度差为 30 度。
            电压脉冲幅值：0.1-0.3 标幺值（可配置）
            持续时间：100-500us（可配置）
            衰减时间：500-2000us（可配置）

        2、记录每个脉冲对应的电流值。
            发一次脉冲后，下一次发脉冲在对角位置（相差180°）。

            顺序      角度       脉冲对标记       采样值
            1          0°         P1+           I1
            2          180°       P1-           I2
            3          30°        P2+           I3
            4          210°       P2-           I4
            5          60°        P3+           I5
            6          240°       P3-           I6
            7          90°        P4+           I7
            8          270°       P4-           I8
            9          120°       P5+           I9
            10         300°       P5-           I10
            11         150°       P6+           I11
            12         330°       P6-           I12

        3、根据电流值，判断电机的磁极和相位。
            - 在一对测试中，电流响应峰值大的为N极方向
            - 多个方向的测试中，电流响应峰值最大的角度为转子位置
            - 如有编码器，此角度与编码器角度有差异，需要校准

        **使用示例：**

        **方式 1：简化接口（推荐）**
        ```c
        // 配置 IPD 参数
        IPD_Config_t config = {
            .pulse_voltage_pu = 0.2f,      // 脉冲电压 20% 标幺值
            .pulse_duration_us = 200,      // 脉冲持续 200us
            .decay_time_us = 1000          // 衰减等待 1000us
        };

        // 一步到位：检测转子位置
        float rotor_angle;
        if (IPD_DetectRotorPosition(&config, &rotor_angle)) {
            printf("转子初始位置: %.1f 度\n", rotor_angle);
        } else {
            printf("IPD 定位失败\n");
        }
        ```

        **方式 2：带详细数据输出（调试用）**
        ```c
        IPD_Config_t config = {  同上  };

        float rotor_angle;
        IPD_Pulse_t pulses[12];  // 获取详细数据

        if (IPD_DetectRotorPositionEx(&config, &rotor_angle, pulses)) {
            printf("转子位置: %.1f 度\n", rotor_angle);

            // 分析每个脉冲的响应（用于调试）
            for (int i = 0; i < 12; i++) {
                printf("角度 %.0f°: 电流 %.3f pu\n",
                       pulses[i].angle_elec,
                       pulses[i].current_sample);
            }
        }
        ```

        **方式 3：分步调用（高级用法）**
        ```c
        IPD_Config_t config = {  同上  };
        IPD_Pulse_t pulses[12];

        // 步骤 1：执行脉冲序列
        if (IPD_ExecutePulseSequence(pulses, &config)) {
            // 步骤 2：自定义分析或记录数据
            // ... 用户自定义处理 ...

            // 步骤 3：计算转子位置
            float rotor_angle = IPD_CalculateRotorPosition(pulses);
            if (rotor_angle >= 0) {
                printf("转子位置: %.1f 度\n", rotor_angle);
            }
        }
        ```

        **注意事项：**
        - IPD 需要在电机静止状态下执行
        - 脉冲电压不宜过大，避免电机抖动
        - 脉冲持续时间需根据电机电感和电阻调整
        - 衰减时间需足够长，确保电流完全衰减

        **角度类型说明：**⚠️重要
        - IPD 检测的是**电角度**（electrical angle），不是机械角度
        - 电角度 = 机械角度 × 极对数
        - 例如：4极电机（2对极）
          * 电角度 0° = 机械角度 0°
          * 电角度 180° = 机械角度 90°
          * 电角度 360° = 机械角度 180°（完成一个电周期）
        - 电角度可直接用于 FOC 控制（Park/Clarke 变换）
        - 如需机械角度：`机械角度 = 电角度 / 极对数`

    ### 转子位置估计(无位置传感器)
        论文：Sensorless Control of Surface-Mount  Permanent-Magnet  
                Synchronous Motors  Based on a Nonlinear Observer


*=============================================================================
*/


#ifndef POSITIONING_H
#define POSITIONING_H


#include <stdbool.h>
#include <stdint.h>

#include "normalization.h"

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief IPD 脉冲数据结构
 * @note 角度说明：
 *       - angle_elec 是**电角度**（electrical angle），不是机械角度
 *       - 电角度 = 机械角度 × 极对数
 *       - IPD 检测的是转子磁极位置，对应电角度
 *       - 该电角度可直接用于 FOC 控制（Park/Clarke 变换）
 */
typedef struct {
    float angle_elec;       // 电角度 (度)，范围 [0, 360)
    float current_sample;   // 电流采样值（电流矢量幅值，单位：标幺值或A）
} IPD_Pulse_t;

// IPD 脉冲参数配置
typedef struct {
    float pulse_voltage_pu;     // 脉冲电压幅值（标幺值，建议 0.1-0.3）
    uint32_t pulse_duration_us; // 脉冲持续时间（微秒，建议 100-500us）
    uint32_t decay_time_us;     // 电流衰减等待时间（微秒，建议 500-2000us）
} IPD_Config_t;

// ============================================================================
// IPD 函数声明
// ============================================================================


/**
 * @brief 检测转子初始位置（简化接口）
 * @param config IPD 配置参数
 * @param angle_deg 输出：转子**电角度**位置（度），范围 [0, 360)
 * @return true=成功, false=失败
 *
 * @note 角度类型说明：
 *       - 输出的是**电角度**
 */

bool IPD_DetectRotorPosition(const IPD_Config_t *config, float *angle_deg);


/**
 * @brief 执行 IPD 脉冲序列（详细接口）
 * @param pulses 输出：12个脉冲的角度和电流采样值
 * @param config IPD 配置参数
 * @return true=成功, false=失败
 */

bool IPD_ExecutePulseSequence(IPD_Pulse_t pulses[12], const IPD_Config_t *config);

/**
 * @brief 计算转子位置（基于脉冲响应）
 * @param pulses 12个脉冲的角度和电流采样值
 * @return 转子**电角度**位置（度），范围 [0, 360)；失败返回 -999.0f
 *
 * @note 算法原理：
 *       - 对比每对对角脉冲（相差180°电角度）的电流响应
 *       - 电流响应大的为 N 极方向
 *       - 在 6 个候选角度中选择电流响应最大的
 *
 * @note 角度类型：
 *       - 返回值是**电角度**，不是机械角度
 *       - 可直接用于 FOC 控制（Park/Clarke 变换）
 *       - 如需机械角度：机械角度 = 电角度 / 极对数
 */
 
float IPD_CalculateRotorPosition(IPD_Pulse_t pulses[12]);


/**
 * @brief 检测转子初始位置（带详细数据输出）
 * @param config IPD 配置参数
 * @param angle_deg 输出：转子**电角度**位置（度），范围 [0, 360)
 * @param pulses 输出：12个脉冲的详细数据（可选，传 NULL 则不输出）
 * @return true=成功, false=失败
 *
 * @note 适用场景：
 *       - 需要同时获取位置和详细数据
 *       - 调试和数据记录
 *
 * @note 角度类型说明：
 *       - 输出的是**电角度**，不是机械角度
 *       - pulses 数组中的 angle_elec 也是电角度
 */

bool IPD_DetectRotorPositionEx(const IPD_Config_t *config, float *angle_deg, IPD_Pulse_t pulses[12]);

//=============================================================================
// 非线性观测器结构体 (使用项目归一化参数)
typedef struct {
    uint8_t motor_id;                    // 电机ID，用于获取对应参数
    
    // 观测器增益参数 (归一化值)
    float gamma;                         // 观测器增益 > 0
    
    // 状态估计变量 (归一化值)
    float x_hat_alpha_pu;               // 状态估计值 x_alpha (标幺值)
    float x_hat_beta_pu;                // 状态估计值 x_beta (标幺值)
    
    // 位置估计输出
    float theta_hat_rad;                // 转子位置估计 (电角度, 弧度)
    float theta_hat_deg;                // 转子位置估计 (电角度, 度)
    
    // 观测器状态标志
    bool is_initialized;                // 初始化标志
    bool is_converged;                  // 收敛标志
    
    // 归一化基值缓存（用于在电机未激活时维持标幺运算）
    normalization_base_values_t base_values;
    bool base_valid;
    
    // 误差统计，用于收敛判定
    float eta_norm_sq_filtered;
    float eta_norm_sq_last;
    uint16_t stable_counter;
    uint16_t stable_required;
    bool error_metrics_valid;
    
} NonlinearObs_Position_t;

// 非线性观测器函数声明
void NonlinearObs_Position_Init(NonlinearObs_Position_t *obs, uint8_t motor_id, float gamma);
void NonlinearObs_Position_Reset(NonlinearObs_Position_t *obs);
void NonlinearObs_Position_Update(NonlinearObs_Position_t *obs, 
                                  float i_alpha_pu, float i_beta_pu, 
                                  float v_alpha_pu, float v_beta_pu, 
                                  float dt);

// 获取位置估计结果 (弧度)
float NonlinearObs_Position_GetThetaRad(NonlinearObs_Position_t *obs);
// 获取位置估计结果 (度)  
float NonlinearObs_Position_GetThetaDeg(NonlinearObs_Position_t *obs);




#ifdef __cplusplus
}
#endif

#endif /* POSITIONING_H */
