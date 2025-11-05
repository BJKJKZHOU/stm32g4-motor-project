// motor_params.c
#include "motor_params.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 定义电机参数数组，暂定两个电机
Motor_Params_t motor_params[motors_number];
Motor_LimitParams_t motor_limit_params[motors_number];



void MotorParams_Init(void)
{
    // 初始化0号电机参数
    motor_params[MOTOR_0].Pn = 4.0f;
    motor_params[MOTOR_0].RPM_rated = 3000.0f;    //电机参数，不代表最终转数限制
    motor_params[MOTOR_0].I_rated = 5.0f;
    motor_params[MOTOR_0].Rs = 0.5f;
    motor_params[MOTOR_0].Ld = 0.001f; //对于表贴式永磁同步电机，Ld=Lq
    motor_params[MOTOR_0].Lq = 0.001f;
    motor_params[MOTOR_0].J = 0.01f;
    motor_params[MOTOR_0].B = 0.0f;  //可以忽略不记，就是0
    motor_params[MOTOR_0].lamaf =  2.0f;   // 转子磁链

    // 初始化0号电机控制系统限值
    motor_limit_params[MOTOR_0].I_limit_user = 8.0f;//用户希望的最大电流
    motor_limit_params[MOTOR_0].I_limit_max = 15.0f;//硬件支持的最大电流,一旦硬件确定,就不能再改变
    motor_limit_params[MOTOR_0].speed_limit_user = 2700.0f;//用户希望的最大转速,一般是实际的转数限制
    motor_limit_params[MOTOR_0].speed_limit_max = motor_params[MOTOR_0].RPM_rated;//默认电机额定转数
    motor_limit_params[MOTOR_0].I_limit_actual =  fmin(motor_limit_params[MOTOR_0].I_limit_user, motor_limit_params[MOTOR_0].I_limit_max);//最终实际最大电流
    motor_limit_params[MOTOR_0].speed_limit_actual = fmin(motor_limit_params[MOTOR_0].speed_limit_user, motor_limit_params[MOTOR_0].speed_limit_max);//最终实际最大转速
    
    // 初始化1号电机参数
    motor_params[MOTOR_1].Pn = 4.0f;
    motor_params[MOTOR_1].RPM_rated = 3000.0f;    //电机参数，不代表最终转数限制
    motor_params[MOTOR_1].I_rated = 5.0f;
    motor_params[MOTOR_1].Rs = 0.5f;
    motor_params[MOTOR_1].Ld = 0.001f; //对于表贴式永磁同步电机，Ld=Lq
    motor_params[MOTOR_1].Lq = 0.001f;
    motor_params[MOTOR_1].J = 0.01f;
    motor_params[MOTOR_1].B = 0.0f;  //可以忽略不记，就是0
    motor_params[MOTOR_1].lamaf =  2.0f;   // 转子磁链

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
    {"P1001", "Rs", "ohm", "定子绕组电阻"},        // 索引0
    {"P1002", "Lq", "mH", "q轴电感"},              // 索引1
    {"P1003", "Ld", "mH", "d轴电感"},              // 索引2
    {"P1004", "Flux", "Vpk_LL/krpm", "反电动势"},  // 索引3
    {"P1005", "Pn", "-", "极对数"},             // 索引4
    {"P1006", "RPM_rated", "rpm", "额定转速"},          // 索引5
    {"P1007", "I_rated", "A", "额定电流"},         // 索引6
    {"P1008", "J", "kg·m²", "转动惯量"},          // 索引7
    {"P1009", "B", "N·m·s/rad", "摩擦系数"},      // 索引8
};

const ParamMap_t param_maps[motors_number][PARAM_COUNT] = {
    // 电机0的参数映射
    {
        {&motor_params[MOTOR_0].Rs, 0},      // 电阻 - 索引0
        {&motor_params[MOTOR_0].Lq, 1},      // q轴电感 - 索引1
        {&motor_params[MOTOR_0].Ld, 2},      // d轴电感 - 索引2
        {&motor_params[MOTOR_0].lamaf, 3},   // 反电动势 - 索引3
        {&motor_params[MOTOR_0].Pn, 4},      // 极对数 - 索引4
        {&motor_params[MOTOR_0].RPM_rated, 5}, // 额定转速 - 索引5
        {&motor_params[MOTOR_0].I_rated, 6},  // 额定电流 - 索引6
        {&motor_params[MOTOR_0].J, 7},       // 转动惯量 - 索引7
        {&motor_params[MOTOR_0].B, 8}        // 摩擦系数 - 索引8
    },
    
    // 电机1的参数映射
    {
        {&motor_params[MOTOR_1].Rs, 0},      // 电阻 - 索引0
        {&motor_params[MOTOR_1].Lq, 1},      // q轴电感 - 索引1
        {&motor_params[MOTOR_1].Ld, 2},      // d轴电感 - 索引2
        {&motor_params[MOTOR_1].lamaf, 3},   // 反电动势 - 索引3
        {&motor_params[MOTOR_1].Pn, 4},      // 极对数 - 索引4
        {&motor_params[MOTOR_1].RPM_rated, 5}, // 额定转速 - 索引5
        {&motor_params[MOTOR_1].I_rated, 6},  // 额定电流 - 索引6
        {&motor_params[MOTOR_1].J, 7},       // 转动惯量 - 索引7
        {&motor_params[MOTOR_1].B, 8}        // 摩擦系数 - 索引8
    }
};

// 自定义浮点数格式化函数（避免依赖printf的浮点数支持）
void format_float_value(char* buffer, int buffer_size, float value)
{
    if (value == 0.0f) {
        snprintf(buffer, buffer_size, "0.00000");
        return;
    }
    
    // 处理整数部分
    int integer_part = (int)value;
    
    // 处理小数部分（保留5位小数）
    float fractional = value - integer_part;
    if (fractional < 0) fractional = -fractional; // 处理负数
    
    int fractional_part = (int)(fractional * 100000 + 0.5f); // 四舍五入到5位小数
    
    // 处理负数
    if (value < 0 && integer_part == 0) {
        snprintf(buffer, buffer_size, "-%d.%05d", integer_part, fractional_part);
    } else {
        snprintf(buffer, buffer_size, "%d.%05d", integer_part, fractional_part);
    }
}

void MotorParams_PrintAll(uint8_t motor_id)// 输出指定电机的所有参数和描述
{
    if (motor_id >= motors_number) {
        printf("错误：电机ID %d 无效\n", motor_id);
        return;
    }
    
    printf("=== 电机 %d 参数列表 ===\n", motor_id);
    printf("%-10s %-15s %-10s %-20s %-10s\n", 
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
    
    // 检查是否是"Pxxxx=value"格式
    char* equal_sign = strchr(param_name, '=');
    if (equal_sign != NULL) {
        // 解析HMI代码格式："Pxxxx=value"
        char hmi_code[10];
        float parsed_value;
        
        // 提取HMI代码部分（等号前的部分）
        int code_length = equal_sign - param_name;
        if (code_length > 0 && code_length < (int)sizeof(hmi_code)) {
            strncpy(hmi_code, param_name, code_length);
            hmi_code[code_length] = '\0';
            
            // 解析数值部分
            parsed_value = atof(equal_sign + 1);
            
            // 使用HMI代码查找参数ID
            int8_t param_id = GetParamIdByHMICode(hmi_code);
            if (param_id >= 0 && param_id < PARAM_COUNT) {
                // 直接使用param_maps数组获取参数指针并设置值
                float* param_ptr = param_maps[motor_id][param_id].value_ptr;
                if (param_ptr) {
                    *param_ptr = parsed_value;
                    // 输出确认信息
                    char value_str[16];
                    format_float_value(value_str, sizeof(value_str), parsed_value);
                    printf("OK new %s = %s\n", param_descs[param_id].name, value_str);
                    return;
                }
            }
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
    
    // 特殊处理：Flux参数名称映射到lamaf
    if (strcmp(param_name, "Flux") == 0) {
        float* param_ptr = param_maps[motor_id][3].value_ptr; // lamaf对应索引3
        if (param_ptr) {
            *param_ptr = value;
            // 输出确认信息
            char value_str[16];
            format_float_value(value_str, sizeof(value_str), value);
            printf("OK new %s = %s\n", param_name, value_str);
        }
    }
}