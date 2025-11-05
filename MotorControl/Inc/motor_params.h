// motor_params.h
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_COUNT 9

enum Motor_Index
{
    MOTOR_0 = 0,
    MOTOR_1 = 1,
    motors_number = 2,
};


// 电机参数结构体
typedef struct {
    float Pn;                   // 极对数
    float RPM_rated;            // 额定转速
    float I_rated;        // 额定电流
    float Rs;                   // 电阻
    float Lq;                   // q轴电感
    float Ld;                   // d轴电感
    float J;                    // 转动惯量
    float B;                    // 摩擦系数
    float lamaf;                // 转子磁链
} Motor_Params_t;

//限值参数结构体，取对应限制最小的作为最终限制
typedef struct {
    float I_limit_user;         // 用户最大电流限值
    float I_limit_max;          // 硬件最大电流限值
    float speed_limit_user;     // 用户转数限值
    float speed_limit_max;      // 电机最大转数
    float I_limit_actual;       // 实际电流限值
    float speed_limit_actual;   // 实际转数限值

} Motor_LimitParams_t;

// 参数映射结构体 
typedef struct {
    float *value_ptr;      // 变量指针
    uint8_t param_idx;     // 参数索引（0-8）
} ParamMap_t;

// 参数描述结构体
typedef struct {
    const char* hmi_code;  // HMI代码
    const char* name;      // 参数名
    const char* unit;      // 单位
    const char* desc;      // 参数描述
} ParamDesc_t;


// 声明全局电机参数数组 
extern Motor_Params_t motor_params[motors_number];  
extern Motor_LimitParams_t motor_limit_params[motors_number];

// 函数声明
void MotorParams_Init(void);
void MotorParams_SetParam(uint8_t motor_id, const char* param_name, float value); //设置电机参数

void MotorParams_PrintAll(uint8_t motor_id); // 输出指定电机的所有参数和描述

// 浮点数格式化函数声明
void format_float_value(char* buffer, int buffer_size, float value);

#ifdef __cplusplus
}

#endif

#endif // MOTOR_PARAMS_H