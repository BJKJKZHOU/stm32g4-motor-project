/*============================================================================
    File Name     : motor_params.h
    Description   : 电机参数模块 - 包含电机参数相关的函数
    Author        : ZHOUHENG
    Date          : 2025-11-03
    ----------------------------------------------------------------------       

*=============================================================================
*/
#include "motor_params.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>

#include <string.h>


// 定义电机参数数组，暂定两个电机
Motor_Params_t motor_params[motors_number];
Motor_LimitParams_t motor_limit_params[motors_number];

// 定义激活电机ID全局变量，初始值为无效值
uint8_t g_active_motor_id = 0xFF;


void MotorParams_Init(void)
{
    // 初始化0号电机参数
    motor_params[MOTOR_0].V_DC = 12.0f;        // 直流母线电压
    motor_params[MOTOR_0].I_rated = 11.6f;      // 额定电流
    motor_params[MOTOR_0].Rs = 0.027f;           // 电阻
    motor_params[MOTOR_0].Lq = 0.045e-3f;         // q轴电感
    motor_params[MOTOR_0].Ld = 0.001f;          // d轴电感（对于表贴式永磁同步电机，Ld=Lq）
    motor_params[MOTOR_0].RPM_rated = 5300.0f; // 额定转速（电机参数，不代表最终转数限制）
    motor_params[MOTOR_0].Pn = 6.0f;           // 极对数
    motor_params[MOTOR_0].Ke = 1.84f;           // 反电动势常数
    motor_params[MOTOR_0].Flux = motor_params[MOTOR_0].Ke * FLUX_CONSTANT / motor_params[MOTOR_0].Pn;         // 转子磁链
    motor_params[MOTOR_0].J = 4.56e-5f;            // 转动惯量
    motor_params[MOTOR_0].B = 0.0f;            // 摩擦系数（可以忽略不记，就是0）

    // 初始化0号电机控制系统限值
    motor_limit_params[MOTOR_0].I_limit_user = 11.6;//用户希望的最大电流
    motor_limit_params[MOTOR_0].I_limit_max = 18.0f;//硬件支持的最大电流,一旦硬件确定,就不能再改变
    motor_limit_params[MOTOR_0].speed_limit_user = 2700.0f;//用户希望的最大转速,一般是实际的转数限制
    motor_limit_params[MOTOR_0].speed_limit_max = motor_params[MOTOR_0].RPM_rated;//默认电机额定转数
    motor_limit_params[MOTOR_0].I_limit_actual =  fmin(motor_limit_params[MOTOR_0].I_limit_user, motor_limit_params[MOTOR_0].I_limit_max);//最终实际最大电流
    motor_limit_params[MOTOR_0].speed_limit_actual = fmin(motor_limit_params[MOTOR_0].speed_limit_user, motor_limit_params[MOTOR_0].speed_limit_max);//最终实际最大转速
    
    // 初始化1号电机参数
    motor_params[MOTOR_1].V_DC = 0.0f;        // 直流母线电压
    motor_params[MOTOR_1].I_rated = 0.0f;      // 额定电流
    motor_params[MOTOR_1].Rs = 0.0f;           // 电阻
    motor_params[MOTOR_1].Lq = 0.000f;         // q轴电感
    motor_params[MOTOR_1].Ld = 0.000f;          // d轴电感（对于表贴式永磁同步电机，Ld=Lq）
    motor_params[MOTOR_1].RPM_rated = 0000.0f; // 额定转速（电机参数，不代表最终转数限制）
    motor_params[MOTOR_1].Pn = 0.0f;           // 极对数
    motor_params[MOTOR_1].Ke = 0.0f;           // 反电动势常数
    motor_params[MOTOR_1].Flux = motor_params[MOTOR_1].Ke * FLUX_CONSTANT / motor_params[MOTOR_1].Pn;         // 转子磁链
    motor_params[MOTOR_1].J = 0.00f;           // 转动惯量
    motor_params[MOTOR_1].B = 0.0f;            // 摩擦系数（可以忽略不记，就是0）

    // 初始化1号电机控制系统限值
    motor_limit_params[MOTOR_1].I_limit_user = 8.0f;//用户希望的最大电流
    motor_limit_params[MOTOR_1].I_limit_max = 15.0f;//硬件支持的最大电流,一旦硬件确定,就不能再改变
    motor_limit_params[MOTOR_1].speed_limit_user = 2700.0f;//用户希望的最大转速,一般是实际的转数限制
    motor_limit_params[MOTOR_1].speed_limit_max = motor_params[MOTOR_1].RPM_rated;//默认电机额定转数
    motor_limit_params[MOTOR_1].I_limit_actual =  fmin(motor_limit_params[MOTOR_1].I_limit_user, motor_limit_params[MOTOR_1].I_limit_max);//最终实际最大电流
    motor_limit_params[MOTOR_1].speed_limit_actual = fmin(motor_limit_params[MOTOR_1].speed_limit_user, motor_limit_params[MOTOR_1].speed_limit_max);//最终实际最大转速
    
}

// 参数描述数组
const ParamDesc_t param_descs[PARAM_COUNT] = { 
    // hmi_code, name, unit, desc
    {"P1001", "V_DC", "V", "直流母线电压"},          // 索引0
    {"P1002", "I_rated", "A", "额定电流"},           // 索引1  
    {"P1003", "Rs", "ohm", "定子绕组相电阻"},         // 索引2
    {"P1004", "Lq", "mH", "q轴电感"},                // 索引3
    {"P1005", "Ld", "mH", "d轴电感"},                // 索引4
    {"P1006", "RPM_rated", "rpm", "额定转速"},       // 索引5
    {"P1007", "Pn", "-", "极对数"},                   // 索引6
    {"P1008", "Ke", "Vpk_LL/krpm", "线线峰值测的反电动势常数"},   // 索引7
    {"P1009", "Flux", "Wb", "转子磁链"},              // 索引8
    {"P1010", "J", "kg·m²*10^-3", "转动惯量"},       // 索引9
    {"P1011", "B", "N·m·s/rad", "摩擦系数"}          // 索引10
};

const ParamMap_t param_maps[motors_number][PARAM_COUNT] = {
    // 电机0的参数映射
    {
        {&motor_params[MOTOR_0].V_DC, 0},      // P1001 - 直流母线电压 - 索引0
        {&motor_params[MOTOR_0].I_rated, 1},   // P1002 - 额定电流 - 索引1
        {&motor_params[MOTOR_0].Rs, 2},        // P1003 - 电阻 - 索引2
        {&motor_params[MOTOR_0].Lq, 3},        // P1004 - q轴电感 - 索引3
        {&motor_params[MOTOR_0].Ld, 4},        // P1005 - d轴电感 - 索引4
        {&motor_params[MOTOR_0].RPM_rated, 5}, // P1006 - 额定转速 - 索引5
        {&motor_params[MOTOR_0].Pn, 6},        // P1007 - 极对数 - 索引6
        {&motor_params[MOTOR_0].Ke, 7},        // P1008 - 反电动势常数 - 索引7
        {&motor_params[MOTOR_0].Flux, 8},      // P1009 - 转子磁链 - 索引8
        {&motor_params[MOTOR_0].J, 9},         // P1010 - 转动惯量 - 索引9
        {&motor_params[MOTOR_0].B, 10}         // P1011 - 摩擦系数 - 索引10
    },

    // 电机1的参数映射
    {
        {&motor_params[MOTOR_1].V_DC, 0},      // P1001 - 直流母线电压 - 索引0
        {&motor_params[MOTOR_1].I_rated, 1},   // P1002 - 额定电流 - 索引1
        {&motor_params[MOTOR_1].Rs, 2},        // P1003 - 电阻 - 索引2
        {&motor_params[MOTOR_1].Lq, 3},        // P1004 - q轴电感 - 索引3
        {&motor_params[MOTOR_1].Ld, 4},        // P1005 - d轴电感 - 索引4
        {&motor_params[MOTOR_1].RPM_rated, 5}, // P1006 - 额定转速 - 索引5
        {&motor_params[MOTOR_1].Pn, 6},        // P1007 - 极对数 - 索引6
        {&motor_params[MOTOR_1].Ke, 7},        // P1008 - 反电动势常数 - 索引7
        {&motor_params[MOTOR_1].Flux, 8},      // P1009 - 转子磁链 - 索引8
        {&motor_params[MOTOR_1].J, 9},         // P1010 - 转动惯量 - 索引9
        {&motor_params[MOTOR_1].B, 10}         // P1011 - 摩擦系数 - 索引10
    }
};


void MotorParams_PrintAll(uint8_t motor_id)// 输出指定电机的所有参数和描述
{
    if (motor_id >= motors_number) {
        printf("错误：电机ID %d 无效\n", motor_id);
        return;
    }
    
    printf("======== 电机 %d 参数列表 ========\n", motor_id);
    printf("%-12s %-20s %-10s %-20s %-10s\n", 
           "HMI代码", "参数名", "值", "单位", "描述");
    printf("------------------------------------------------------------\n");
    
    // 遍历所有参数
    for (int i = 0; i < PARAM_COUNT; i++) {
        // 使用二维数组直接访问参数指针
        float *value_ptr = param_maps[motor_id][i].value_ptr;
        
        if (value_ptr) {
            char value_str[16]; // 缓冲区用于存储格式化后的值
            format_float_value(value_str, sizeof(value_str), *value_ptr);
            
            printf("%-10s %-15s %-10s %-20s %-10s\n",
                   param_descs[i].hmi_code,
                   param_descs[i].name,
                   value_str,
                   param_descs[i].unit,
                   param_descs[i].desc);
        } else {
            printf("%-10s %-15s %-10s %-20s %-10s\n",
                   param_descs[i].hmi_code,
                   param_descs[i].name,
                   "NULL",
                   param_descs[i].unit,
                   param_descs[i].desc);
        }
    }
    printf("============================================================\n");

} 

// 内部辅助函数：根据HMI代码获取参数ID
static int8_t GetParamIdByHMICode(const char* hmi_code)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(param_descs[i].hmi_code, hmi_code) == 0) {
            return i;
        }
    }
    return -1; // 无效HMI代码
}

void MotorParams_SetParam(uint8_t motor_id, const char* param_name, float value) // 设置电机参数
{
    if (motor_id >= motors_number) {
        return;
    }
    
    // 首先尝试将param_name作为HMI代码（Pxxxx格式）处理
    int8_t param_id = GetParamIdByHMICode(param_name);
    if (param_id >= 0 && param_id < PARAM_COUNT) {
        // 使用HMI代码查找参数ID成功，直接设置值
        float* param_ptr = param_maps[motor_id][param_id].value_ptr;
        if (param_ptr) {
            *param_ptr = value;
            // 输出确认信息
            char value_str[16];
            format_float_value(value_str, sizeof(value_str), value);
            printf("OK new %s = %s\n", param_descs[param_id].name, value_str);
            return;
        }
    }
            
    // 如果不是HMI代码格式，使用参数名称设置逻辑
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(param_descs[i].name, param_name) == 0) {
            // 直接使用param_maps数组获取参数指针并设置值
            float* param_ptr = param_maps[motor_id][i].value_ptr;
            if (param_ptr) {
                *param_ptr = value;
                // 输出确认信息
                char value_str[16];
                format_float_value(value_str, sizeof(value_str), value);
                printf("OK new %s = %s\n", param_name, value_str);
                return;
            }
        }
    }
    
    // 如果两种方式都找不到参数，输出错误信息
    printf("错误：未找到参数 '%s'\n", param_name);
}

// 激活状态管理函数实现

bool MotorParams_IsMotorEnabled(uint8_t motor_id)
{
    if (motor_id >= motors_number) {
        return false;
    }
    return (g_active_motor_id == motor_id);
}

void MotorParams_SetActiveMotor(uint8_t motor_id)
{
    if (motor_id >= motors_number) {
        printf("错误：无效的电机ID %d\n", motor_id);
        return;
    }
    
    g_active_motor_id = motor_id;
    printf("OK 电机 %d 已激活\n", motor_id);
}

void MotorParams_DisableMotor(uint8_t motor_id)
{
    if (motor_id >= motors_number) {
        printf("错误：无效的电机ID %d\n", motor_id);
        return;
    }
    
    if (g_active_motor_id == motor_id) {
        g_active_motor_id = 0xFF;  // 设置为无效值
        printf("OK 电机 %d 已停用\n", motor_id);
    } else {
        printf("警告：电机 %d 未处于激活状态\n", motor_id);
    }
}

uint8_t MotorParams_GetActiveMotor(void)
{
    return g_active_motor_id;
}

bool MotorParams_IsAnyMotorActive(void)
{
    return (g_active_motor_id < motors_number);
}
