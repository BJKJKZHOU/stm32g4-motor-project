/*============================================================================
    File Name     : motor_params.c
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

    // 初始化0号电机位置限位参数（默认不启用）
    motor_limit_params[MOTOR_0].position_limit_enable = false;//默认不启用位置限位
    motor_limit_params[MOTOR_0].position_limit_min = 0.0f;//最小位置限值 (rad)
    motor_limit_params[MOTOR_0].position_limit_max = 0.0f;//最大位置限值 (rad)
    
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

    // 初始化1号电机位置限位参数（默认不启用）
    motor_limit_params[MOTOR_1].position_limit_enable = false;//默认不启用位置限位
    motor_limit_params[MOTOR_1].position_limit_min = 0.0f;//最小位置限值 (rad)
    motor_limit_params[MOTOR_1].position_limit_max = 0.0f;//最大位置限值 (rad)
    
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

// 限值参数描述数组
const ParamDesc_t limit_param_descs[LIMIT_PARAM_COUNT] = {
    // hmi_code, name, unit, desc
    {"P2001", "I_limit_user", "A", "用户最大电流限值"},           // 索引0
    {"P2002", "speed_limit_user", "rpm", "用户最大转速限值"},     // 索引1
    {"P2003", "position_limit_enable", "-", "位置限位使能"},      // 索引2
    {"P2004", "position_limit_min", "rad", "最小位置限值"},       // 索引3
    {"P2005", "position_limit_max", "rad", "最大位置限值"},       // 索引4
    {"P2006", "I_limit_max", "A", "硬件最大电流限值（只读）"}     // 索引5
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

// 限值参数映射数组（注意：position_limit_enable 是 bool 类型，需要特殊处理）
const ParamMap_t limit_param_maps[motors_number][LIMIT_PARAM_COUNT] = {
    // 电机0的限值参数映射
    {
        {&motor_limit_params[MOTOR_0].I_limit_user, 0},        // P2001 - 用户最大电流限值 - 索引0
        {&motor_limit_params[MOTOR_0].speed_limit_user, 1},    // P2002 - 用户最大转速限值 - 索引1
        {NULL, 2},                                              // P2003 - 位置限位使能（bool类型，特殊处理）- 索引2
        {&motor_limit_params[MOTOR_0].position_limit_min, 3},  // P2004 - 最小位置限值 - 索引3
        {&motor_limit_params[MOTOR_0].position_limit_max, 4},  // P2005 - 最大位置限值 - 索引4
        {&motor_limit_params[MOTOR_0].I_limit_max, 5}          // P2006 - 硬件最大电流限值（只读）- 索引5
    },

    // 电机1的限值参数映射
    {
        {&motor_limit_params[MOTOR_1].I_limit_user, 0},        // P2001 - 用户最大电流限值 - 索引0
        {&motor_limit_params[MOTOR_1].speed_limit_user, 1},    // P2002 - 用户最大转速限值 - 索引1
        {NULL, 2},                                              // P2003 - 位置限位使能（bool类型，特殊处理）- 索引2
        {&motor_limit_params[MOTOR_1].position_limit_min, 3},  // P2004 - 最小位置限值 - 索引3
        {&motor_limit_params[MOTOR_1].position_limit_max, 4},  // P2005 - 最大位置限值 - 索引4
        {&motor_limit_params[MOTOR_1].I_limit_max, 5}          // P2006 - 硬件最大电流限值（只读）- 索引5
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

void MotorParams_PrintLimits(uint8_t motor_id)
{
    if (motor_id >= motors_number) {
        printf("错误：电机ID %d 无效\n", motor_id);
        return;
    }

    Motor_LimitParams_t *limit = &motor_limit_params[motor_id];

    printf("======== 电机 %d 限值参数 ========\n", motor_id);
    printf("%-10s %-25s %-15s %-15s %-15s %-10s\n",
           "HMI代码", "参数名", "用户限值", "硬件限值", "实际限值", "单位");
    printf("--------------------------------------------------------------------------------\n");

    // 电流限值
    char i_user_str[16], i_max_str[16], i_actual_str[16];
    format_float_value(i_user_str, sizeof(i_user_str), limit->I_limit_user);
    format_float_value(i_max_str, sizeof(i_max_str), limit->I_limit_max);
    format_float_value(i_actual_str, sizeof(i_actual_str), limit->I_limit_actual);
    printf("%-10s %-25s %-15s %-15s %-15s %-10s\n",
           "P2001", "I_limit_user", i_user_str, "-", "-", "A");
    printf("%-10s %-25s %-15s %-15s %-15s %-10s\n",
           "P2006", "I_limit_max (只读)", "-", i_max_str, "-", "A");
    printf("%-10s %-25s %-15s %-15s %-15s %-10s\n",
           "-", "I_limit_actual", "-", "-", i_actual_str, "A");

    // 速度限值
    char speed_user_str[16], speed_max_str[16], speed_actual_str[16];
    format_float_value(speed_user_str, sizeof(speed_user_str), limit->speed_limit_user);
    format_float_value(speed_max_str, sizeof(speed_max_str), limit->speed_limit_max);
    format_float_value(speed_actual_str, sizeof(speed_actual_str), limit->speed_limit_actual);
    printf("%-10s %-25s %-15s %-15s %-15s %-10s\n",
           "P2002", "speed_limit_user", speed_user_str, "-", "-", "rpm");
    printf("%-10s %-25s %-15s %-15s %-15s %-10s\n",
           "-", "speed_limit_max", "-", speed_max_str, "-", "rpm");
    printf("%-10s %-25s %-15s %-15s %-15s %-10s\n",
           "-", "speed_limit_actual", "-", "-", speed_actual_str, "rpm");

    printf("--------------------------------------------------------------------------------\n");

    // 位置限位
    printf("%-10s %-25s %s\n",
           "P2003", "position_limit_enable",
           limit->position_limit_enable ? "启用" : "禁用");

    if (limit->position_limit_enable) {
        char pos_min_str[16], pos_max_str[16];
        format_float_value(pos_min_str, sizeof(pos_min_str), limit->position_limit_min);
        format_float_value(pos_max_str, sizeof(pos_max_str), limit->position_limit_max);
        printf("%-10s %-25s %-15s rad (%.2f°)\n",
               "P2004", "position_limit_min",
               pos_min_str, limit->position_limit_min * 180.0f / PI);
        printf("%-10s %-25s %-15s rad (%.2f°)\n",
               "P2005", "position_limit_max",
               pos_max_str, limit->position_limit_max * 180.0f / PI);

        // 计算位置范围
        float range_rad = limit->position_limit_max - limit->position_limit_min;
        float range_deg = range_rad * 180.0f / PI;
        printf("%-10s %-25s %.4f rad (%.2f°)\n",
               "-", "位置范围", range_rad, range_deg);
    } else {
        printf("%-10s %-25s %s\n",
               "P2004", "position_limit_min", "未设置（限位未启用）");
        printf("%-10s %-25s %s\n",
               "P2005", "position_limit_max", "未设置（限位未启用）");
    }

    printf("====================================================================\n");
}

// 内部辅助函数：根据HMI代码获取限值参数ID
static int8_t GetLimitParamIdByHMICode(const char* hmi_code)
{
    for (int i = 0; i < LIMIT_PARAM_COUNT; i++) {
        if (strcmp(limit_param_descs[i].hmi_code, hmi_code) == 0) {
            return i;
        }
    }
    return -1; // 无效HMI代码
}

void MotorParams_SetLimitParam(uint8_t motor_id, const char* param_name, float value)
{
    if (motor_id >= motors_number) {
        return;
    }

    // 首先尝试将param_name作为HMI代码（Pxxxx格式）处理
    int8_t param_id = GetLimitParamIdByHMICode(param_name);
    if (param_id >= 0 && param_id < LIMIT_PARAM_COUNT) {
        // 使用HMI代码查找参数ID成功

        // 特殊处理：P2003 (position_limit_enable) 是 bool 类型
        if (param_id == 2) {
            bool enable_value = (value != 0.0f);
            motor_limit_params[motor_id].position_limit_enable = enable_value;
            printf("OK new %s = %s\n", limit_param_descs[param_id].name, enable_value ? "true" : "false");
            return;
        }

        // 特殊处理：P2006 (I_limit_max) 是只读参数
        if (param_id == 5) {
            printf("错误：参数 '%s' 是只读参数，不能修改\n", limit_param_descs[param_id].name);
            return;
        }

        // 处理其他 float 类型参数
        float* param_ptr = limit_param_maps[motor_id][param_id].value_ptr;
        if (param_ptr) {
            *param_ptr = value;

            // 更新实际限值
            motor_limit_params[motor_id].I_limit_actual = fmin(
                motor_limit_params[motor_id].I_limit_user,
                motor_limit_params[motor_id].I_limit_max
            );
            motor_limit_params[motor_id].speed_limit_actual = fmin(
                motor_limit_params[motor_id].speed_limit_user,
                motor_limit_params[motor_id].speed_limit_max
            );

            // 输出确认信息
            char value_str[16];
            format_float_value(value_str, sizeof(value_str), value);

            // 对于位置限值，同时显示度数
            if (param_id == 3 || param_id == 4) {
                printf("OK new %s = %s %s (%.2f°)\n",
                       limit_param_descs[param_id].name,
                       value_str,
                       limit_param_descs[param_id].unit,
                       value * 180.0f / PI);
            } else {
                printf("OK new %s = %s %s\n",
                       limit_param_descs[param_id].name,
                       value_str,
                       limit_param_descs[param_id].unit);
            }
            return;
        }
    }

    // 如果不是HMI代码格式，使用参数名称设置逻辑
    for (int i = 0; i < LIMIT_PARAM_COUNT; i++) {
        if (strcmp(limit_param_descs[i].name, param_name) == 0) {
            // 特殊处理：position_limit_enable 是 bool 类型
            if (i == 2) {
                bool enable_value = (value != 0.0f);
                motor_limit_params[motor_id].position_limit_enable = enable_value;
                printf("OK new %s = %s\n", param_name, enable_value ? "true" : "false");
                return;
            }

            // 特殊处理：I_limit_max 是只读参数
            if (i == 5) {
                printf("错误：参数 '%s' 是只读参数，不能修改\n", param_name);
                return;
            }

            // 处理其他 float 类型参数
            float* param_ptr = limit_param_maps[motor_id][i].value_ptr;
            if (param_ptr) {
                *param_ptr = value;

                // 更新实际限值
                motor_limit_params[motor_id].I_limit_actual = fmin(
                    motor_limit_params[motor_id].I_limit_user,
                    motor_limit_params[motor_id].I_limit_max
                );
                motor_limit_params[motor_id].speed_limit_actual = fmin(
                    motor_limit_params[motor_id].speed_limit_user,
                    motor_limit_params[motor_id].speed_limit_max
                );

                // 输出确认信息
                char value_str[16];
                format_float_value(value_str, sizeof(value_str), value);

                // 对于位置限值，同时显示度数
                if (i == 3 || i == 4) {
                    printf("OK new %s = %s %s (%.2f°)\n",
                           param_name,
                           value_str,
                           limit_param_descs[i].unit,
                           value * 180.0f / PI);
                } else {
                    printf("OK new %s = %s %s\n",
                           param_name,
                           value_str,
                           limit_param_descs[i].unit);
                }
                return;
            }
        }
    }

    // 如果两种方式都找不到参数，输出错误信息
    printf("错误：未找到限值参数 '%s'\n", param_name);
}
