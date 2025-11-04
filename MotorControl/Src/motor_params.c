// motor_params.c
#include "motor_params.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// 定义电机参数数组，暂定两个电机


Motor_Params_t motor_params[motors_number];
Motor_LimitParams_t motor_limit_params[motors_number];

// 内部辅助函数：获取参数指针
static float* GetParamPointer(uint8_t motor_id, uint8_t param_id)
{
    if (motor_id >= motors_number) {
        return NULL;
    }
    
    switch (param_id) {
        case 0: return &motor_params[motor_id].Rs;
        case 1: return &motor_params[motor_id].Lq;
        case 2: return &motor_params[motor_id].Ld;
        case 3: return &motor_params[motor_id].lamaf;
        case 4: return &motor_params[motor_id].Pn;
        case 5: return &motor_params[motor_id].RPM_rated;
        case 6: return &motor_params[motor_id].I_rated;
        case 7: return &motor_params[motor_id].J;
        case 8: return &motor_params[motor_id].B;
        default: return NULL;
    }
}

// 内部辅助函数：根据参数名称获取参数ID
static int8_t GetParamIdByName(const char* param_name)
{
    if (strcmp(param_name, "Rs") == 0) return 0;
    else if (strcmp(param_name, "Lq") == 0) return 1;
    else if (strcmp(param_name, "Ld") == 0) return 2;
    else if (strcmp(param_name, "lamaf") == 0) return 3;
    else if (strcmp(param_name, "Pn") == 0) return 4;
    else if (strcmp(param_name, "RPM_rated") == 0) return 5;
    else if (strcmp(param_name, "I_rated") == 0) return 6;
    else if (strcmp(param_name, "J") == 0) return 7;
    else if (strcmp(param_name, "B") == 0) return 8;
    else return -1; // 无效参数名称
}

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
    
}

// 参数描述数组
const ParamDesc_t param_descs[PARAM_COUNT] = { 
    // hmi_code, name, unit, desc
    {"P1001", "Rs", "ohm", "定子绕组电阻"},        // 索引0
    {"P1002", "Lq", "mH", "q轴电感"},              // 索引1
    {"P1003", "Ld", "mH", "d轴电感"},              // 索引2
    {"P1004", "Flux", "Vpk_LL/krpm", "反电动势"},  // 索引3
    {"P1005", "Pn", "  ", "极对数"},             // 索引4
    {"P1006", "RPM_rated", "rpm", "额定转速"},          // 索引5
    {"P1007", "I_rated", "A", "额定电流"},         // 索引6
    {"P1008", "J", "kg·m²", "转动惯量"},          // 索引7
    {"P1009", "B", "N·m·s/rad", "摩擦系数"},      // 索引8
};

const ParamMap_t param_maps[PARAM_COUNT] = {
    {&motor_params[MOTOR_0].Rs, 0},      // 电阻 - 索引0
    {&motor_params[MOTOR_0].Lq, 1},      // q轴电感 - 索引1
    {&motor_params[MOTOR_0].Ld, 2},      // d轴电感 - 索引2
    {&motor_params[MOTOR_0].lamaf, 3},   // 反电动势 - 索引3
    {&motor_params[MOTOR_0].Pn, 4},      // 极对数 - 索引4
    {&motor_params[MOTOR_0].RPM_rated, 5}, // 额定转速 - 索引5
    {&motor_params[MOTOR_0].I_rated, 6},  // 额定电流 - 索引6
    {&motor_params[MOTOR_0].J, 7},       // 转动惯量 - 索引7
    {&motor_params[MOTOR_0].B, 8}        // 摩擦系数 - 索引8
};

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
        // 使用辅助函数获取参数值
        float *value_ptr = GetParamPointer(motor_id, i);
        
        if (value_ptr) {
            printf("%-10s %-15s %-10.3f %-20s %-10s\n",
                   param_descs[i].hmi_code,
                   param_descs[i].name,
                   *value_ptr,
                   param_descs[i].unit,
                   param_descs[i].desc);
        }
    }
    printf("============================================================\n");

} 





void MotorParams_SetParam(uint8_t motor_id, const char* param_name, float value) // 设置电机参数
{
    if (motor_id < motors_number)
    {
        // 使用辅助函数获取参数ID
        int8_t param_id = GetParamIdByName(param_name);
        if (param_id >= 0) {
            // 使用辅助函数获取参数指针并设置值
            float* param_ptr = GetParamPointer(motor_id, param_id);
            if (param_ptr) {
                *param_ptr = value;
            }
        }
    }
}