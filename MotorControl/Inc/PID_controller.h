/*============================================================================
    File Name     : PID_controller.h
    Description   : PID控制器头文件
    Author        : ZHOUHENG 
    Date          : 2025-11-5
    ----------------------------------------------------------------------       
  pid控制器，电流环和速度环的基础控制器是pi控制。
  控制器需要有抗积分保护功能
  速度环的另一方案是PDFF控制器，调整参数可变为pi控制
  位置环的控制器是PD控制，
            
  电流环参数
  name     HMI代码     单位        描述
  Kip       PXXXX       -       电流环比例系数
  Kii       PXXXX       -       电流环积分系数

  电流环解耦
  q 轴: id_feedback反馈的电流和 ws 电转数(rad/s) 
        ws*( Ld * id_feedback + Flux  )
  d 轴: iq_feedback反馈的电流和 ws 电转数(rad/s) 
        -ws*( Lq * iq_feedback)


  直接电压前馈 V_ff = Ke × ω //低速下禁用，禁用范围待测试
  V_ff：前馈电压
  ω ：  机械转数
  将 V_ff 直接加到电流环PI控制器的输出上
  
  速度环参数
  name     HMI代码     单位        描述
  Kvp       PXXXX       -       速度环比例系数
  Kvi       PXXXX       -       速度环积分系数
  Kvfr      PXXXX       -       速度环前馈
  
  位置环参数 - 待定，暂时不设置位置环
  
  需要原始的KP,KI参数和部分基值共同计算从归一化后的参数
  
*=============================================================================
*/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "main.h"
#include "motor_params.h"
#include "normalization.h"
#include "arm_math.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PID控制器类型定义 */
typedef enum {
    PID_TYPE_CURRENT_D = 0,    // d轴电流环PI控制器
    PID_TYPE_CURRENT_Q,        // q轴电流环PI控制器
    PID_TYPE_SPEED,            // 速度环PI控制器
    PID_TYPE_POSITION,         // 位置环PD控制器
    PID_TYPE_COUNT
} pid_controller_type_t;

/* 抗积分饱和模式 */
typedef enum {
    ANTI_WINDUP_NONE = 0,      // 无抗积分饱和
    ANTI_WINDUP_CLAMP,         // 积分限幅
    ANTI_WINDUP_BACK_CALC      // 反向计算
} anti_windup_mode_t;

/* PID控制器结构体 */
typedef struct {
    /* 控制器参数 */
    float kp;                  // 比例系数
    float ki;                  // 积分系数
    float kd;                  // 微分系数 (设为0时为PI控制器)
    
    /* 控制器状态 */
    float integral;            // 积分项累积
    float prev_error;          // 上一次误差
    float prev_measurement;    // 上一次测量值（用于微分作用在测量值）
    float prev_derivative;     // 上一次微分项（用于微分滤波）
    float output;              // 控制器输出
    
    /* 限幅参数 */
    float output_limit;        // 输出限幅
    float integral_limit;      // 积分限幅
    
    /* 抗积分饱和 */
    anti_windup_mode_t anti_windup_mode;
    float back_calc_gain;      // 反向计算增益
    
    /* 滤波参数 */
    float derivative_filter_coeff; // 微分滤波系数 (0-1, 0表示无滤波)
    
    /* 控制器配置 */
    bool enable;               // 控制器使能
    bool derivative_on_measurement; // 微分作用在测量值而非误差
    
} pid_controller_t;

/* 电流环解耦参数结构体 */
typedef struct {
    float ws;                  // 电角速度 (rad/s)
    float Ld;                  // d轴电感
    float Lq;                  // q轴电感
    float Flux;                // 转子磁链
    bool enable;               // 解耦使能
} current_decoupling_t;

/* 速度前馈参数结构体 */
typedef struct {
    float feed_forward_gain;   // 前馈增益 (Kvfr)
    float omega_threshold;     // 启用前馈的角速度阈值
    bool enable;               // 前馈使能
} speed_feed_forward_t;

/* PID控制器组结构体 - 每个电机一个组 */
typedef struct {
    pid_controller_t current_d;    // d轴电流环控制器
    pid_controller_t current_q;    // q轴电流环控制器
    pid_controller_t speed;        // 速度环控制器
    pid_controller_t position;     // 位置环控制器
    
    current_decoupling_t decoupling; // 电流环解耦参数
    speed_feed_forward_t feed_forward; // 速度前馈参数
    
    float dt;                     // 采样时间 (s)
    uint8_t motor_id;             // 关联的电机ID
} pid_controller_group_t;

/* 全局PID控制器组声明 */
extern pid_controller_group_t pid_groups[motors_number];

/* ========================================================================
 * 核心PID控制函数
 * ======================================================================== */

/**
 * @brief 初始化PID控制器组
 * @param motor_id 电机ID
 */
void PID_Init(uint8_t motor_id);

/**
 * @brief 通用PID控制器计算函数
 * @param pid PID控制器结构体指针
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @param dt 采样时间 (s)
 * @return 控制器输出
 */
float PID_Calculate(pid_controller_t *pid, float setpoint, float feedback, float dt);

/**
 * @brief 重置PID控制器状态
 * @param motor_id 电机ID
 * @param type 控制器类型
 */
void PID_ResetController(uint8_t motor_id, pid_controller_type_t type);

/* ========================================================================
 * 参数设置函数
 * ======================================================================== */

/**
 * @brief 设置PID控制器参数
 * @param motor_id 电机ID
 * @param type 控制器类型
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数 (PI控制器设为0)
 */
void PID_SetParameters(uint8_t motor_id, pid_controller_type_t type,
                      float kp, float ki, float kd);

/**
 * @brief 设置PID控制器输出限幅
 * @param motor_id 电机ID
 * @param type 控制器类型
 * @param limit 输出限幅
 */
void PID_SetOutputLimit(uint8_t motor_id, pid_controller_type_t type, float limit);

/**
 * @brief 设置PID控制器积分限幅
 * @param motor_id 电机ID
 * @param type 控制器类型
 * @param limit 积分限幅
 */
void PID_SetIntegralLimit(uint8_t motor_id, pid_controller_type_t type, float limit);

/**
 * @brief 设置抗积分饱和模式
 * @param motor_id 电机ID
 * @param type 控制器类型
 * @param mode 抗积分饱和模式
 * @param back_calc_gain 反向计算增益（仅用于BACK_CALC模式）
 */
void PID_SetAntiWindup(uint8_t motor_id, pid_controller_type_t type,
                      anti_windup_mode_t mode, float back_calc_gain);

/**
 * @brief 设置采样时间
 * @param motor_id 电机ID
 * @param dt 采样时间 (s)
 */
void PID_SetSampleTime(uint8_t motor_id, float dt);

/* ========================================================================
 * 电流环控制函数
 * ======================================================================== */

/**
 * @brief d轴电流环PI控制器计算
 * @param motor_id 电机ID
 * @param id_ref d轴电流设定值
 * @param id_feedback d轴电流反馈值
 * @param iq_feedback q轴电流反馈值（用于解耦）
 * @return d轴电压输出
 */
float PID_CurrentD_Calculate(uint8_t motor_id, float id_ref, float id_feedback, float iq_feedback);

/**
 * @brief q轴电流环PI控制器计算
 * @param motor_id 电机ID
 * @param iq_ref q轴电流设定值
 * @param iq_feedback q轴电流反馈值
 * @param id_feedback d轴电流反馈值（用于解耦）
 * @return q轴电压输出
 */
float PID_CurrentQ_Calculate(uint8_t motor_id, float iq_ref, float iq_feedback, float id_feedback);

/**
 * @brief 设置电流环解耦参数
 * @param motor_id 电机ID
 * @param ws 电角速度 (rad/s)
 * @param Ld d轴电感
 * @param Lq q轴电感
 * @param Flux 转子磁链
 */
void PID_SetDecoupling(uint8_t motor_id, float ws, float Ld, float Lq, float Flux);

/**
 * @brief 启用/禁用电流环解耦
 * @param motor_id 电机ID
 * @param enable true启用，false禁用
 */
void PID_EnableDecoupling(uint8_t motor_id, bool enable);

/* ========================================================================
 * 速度环控制函数
 * ======================================================================== */

/**
 * @brief 速度环PI控制器计算
 * @param motor_id 电机ID
 * @param speed_ref 速度设定值
 * @param speed_feedback 速度反馈值
 * @return 电流设定值输出
 */
float PID_Speed_Calculate(uint8_t motor_id, float speed_ref, float speed_feedback);

/**
 * @brief 设置速度前馈参数
 * @param motor_id 电机ID
 * @param feed_forward_gain 前馈增益 (Kvfr)
 * @param omega_threshold 启用阈值
 */
void PID_SetSpeedFeedForward(uint8_t motor_id, float feed_forward_gain, float omega_threshold);

/**
 * @brief 启用/禁用速度前馈
 * @param motor_id 电机ID
 * @param enable true启用，false禁用
 */
void PID_EnableSpeedFeedForward(uint8_t motor_id, bool enable);

/* ========================================================================
 * 位置环控制函数
 * ======================================================================== */

/**
 * @brief 位置环PD控制器计算
 * @param motor_id 电机ID
 * @param position_ref 位置设定值
 * @param position_feedback 位置反馈值
 * @return 速度设定值输出
 */
float PID_Position_Calculate(uint8_t motor_id, float position_ref, float position_feedback);

/* ========================================================================
 * 控制器管理函数
 * ======================================================================== */

/**
 * @brief 启用/禁用PID控制器
 * @param motor_id 电机ID
 * @param type 控制器类型
 * @param enable true启用，false禁用
 */
void PID_EnableController(uint8_t motor_id, pid_controller_type_t type, bool enable);

/**
 * @brief 获取PID控制器输出
 * @param motor_id 电机ID
 * @param type 控制器类型
 * @return 控制器输出值
 */
float PID_GetOutput(uint8_t motor_id, pid_controller_type_t type);

/**
 * @brief 获取PID控制器参数
 * @param motor_id 电机ID
 * @param type 控制器类型
 * @param kp 输出比例系数指针
 * @param ki 输出积分系数指针
 * @param kd 输出微分系数指针
 */
void PID_GetParameters(uint8_t motor_id, pid_controller_type_t type,
                      float *kp, float *ki, float *kd);

/* ========================================================================
 * 默认参数定义
 * ======================================================================== */

/* 默认控制参数 */
#define PID_DEFAULT_KP_CURRENT_D    0.5f    // d轴电流环比例系数
#define PID_DEFAULT_KI_CURRENT_D    10.0f   // d轴电流环积分系数
#define PID_DEFAULT_KD_CURRENT_D    0.0f    // d轴电流环微分系数 (PI控制)

#define PID_DEFAULT_KP_CURRENT_Q    0.5f    // q轴电流环比例系数
#define PID_DEFAULT_KI_CURRENT_Q    10.0f   // q轴电流环积分系数
#define PID_DEFAULT_KD_CURRENT_Q    0.0f    // q轴电流环微分系数 (PI控制)

#define PID_DEFAULT_KP_SPEED        0.1f    // 速度环比例系数
#define PID_DEFAULT_KI_SPEED        1.0f    // 速度环积分系数
#define PID_DEFAULT_KD_SPEED        0.0f    // 速度环微分系数 (PI控制)

#define PID_DEFAULT_KP_POSITION     5.0f    // 位置环比例系数
#define PID_DEFAULT_KI_POSITION     0.0f    // 位置环积分系数 (无积分)
#define PID_DEFAULT_KD_POSITION     0.1f    // 位置环微分系数 (PD控制)

/* 默认限幅参数 */
#define PID_DEFAULT_OUTPUT_LIMIT_CURRENT     1.0f    // 电流环输出限幅 (标幺值)
#define PID_DEFAULT_OUTPUT_LIMIT_SPEED       1.0f    // 速度环输出限幅 (标幺值)
#define PID_DEFAULT_OUTPUT_LIMIT_POSITION    1.0f    // 位置环输出限幅 (标幺值)

#define PID_DEFAULT_INTEGRAL_LIMIT_CURRENT   0.5f    // 电流环积分限幅
#define PID_DEFAULT_INTEGRAL_LIMIT_SPEED     0.5f    // 速度环积分限幅
#define PID_DEFAULT_INTEGRAL_LIMIT_POSITION  0.5f    // 位置环积分限幅

/* 默认前馈参数 */
#define PID_DEFAULT_SPEED_FEED_FORWARD_GAIN  0.1f    // 速度前馈增益
#define PID_DEFAULT_SPEED_FEED_FORWARD_THRESHOLD  10.0f  // 速度前馈启用阈值 (rad/s)

/* 默认滤波参数 */
#define PID_DEFAULT_DERIVATIVE_FILTER_COEFF  0.1f    // 微分滤波系数

/* 默认抗积分饱和参数 */
#define PID_DEFAULT_BACK_CALC_GAIN           1.0f    // 反向计算增益

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H