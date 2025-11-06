/*============================================================================
    File Name     : command.c
    Description   : 负责解析上位机发送的命令，根据命令执行相应的操作。
    Author        : 
    Date          : 2025-11-04
=============================================================================
 */

#include "command.h"
#include "motor_params.h"  // 包含电机参数定义，包括motors_number

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// 外部函数声明
extern void Vofa_Plot_Start(void);
extern void Vofa_Plot_Stop(void);
extern void MotorParams_PrintAll(uint8_t motor_id);
extern void MotorParams_SetParam(uint8_t motor_id, const char* param_name, float value);

static void handle_plot_command(char* args)
{
    if (args != NULL && strcmp(args, "stop") == 0) {
        Vofa_Plot_Stop();
    } else {
        Vofa_Plot_Start();
    }
}

static void handle_motor_command(char* args)
{
    if (args == NULL || strlen(args) == 0) {
        printf("Usage: motor <motor_id>\r\n");
        printf("Available motors: 0, 1\r\n");
        return;
    }
    
    // 解析电机ID（支持格式：motor0, motor1, 0, 1）
    uint8_t motor_id;
    
    // 如果参数以"motor"开头，跳过"motor"部分
    if (strncmp(args, "motor", 5) == 0) {
        args += 5; // 跳过"motor"
    }

    // 检查参数是否为空或无效
    if (strlen(args) == 0) {
        printf("Error: Missing motor ID\r\n");
        printf("Usage: motor <motor_id>\r\n");
        printf("Available motors: 0, 1\r\n");
        return;
    }
    
    // 将字符串转换为电机ID
    char* endptr;
    motor_id = strtoul(args, &endptr, 10);
    
    // 检查转换是否成功
    if (endptr == args || *endptr != '\0') {
        printf("Error: Invalid motor ID '%s'\r\n", args);
        printf("Available motors: 0, 1\r\n");
        return;
    }
    
    // 检查电机ID是否在有效范围内
    if (motor_id >= motors_number) {
        printf("Error: Motor %d not found\r\n", motor_id);
        printf("Available motors: 0, 1\r\n");
        return;
    }
    
    // 打印指定电机的参数信息
    MotorParams_PrintAll(motor_id);
}

static void handle_set_command(char* args)
{
    if (args == NULL || strlen(args) == 0) {
        printf("Usage: set <motor_id> <param_name> = <value>\r\n");
        printf("       set <param_name> = <value>\r\n");
        return;
    }
    
    // 解析参数设置命令
    char* motor_id = NULL;
    char* param_name = args;
    char* value_str = NULL;
    
    // 首先检查参数中是否包含电机ID（在等号分割前处理）
    char* space_pos = strchr(args, ' ');
    if (space_pos != NULL) {
        // 检查空格后面是否包含等号
        char* equal_pos = strchr(space_pos + 1, '=');
        if (equal_pos != NULL) {
            // 分割电机ID和参数部分
            *space_pos = '\0';
            motor_id = args;
            param_name = space_pos + 1;
        }
    }
    
    // 检查是否包含等号（支持有空格和无空格格式）
    char* equal_pos = strchr(param_name, '=');
    if (equal_pos == NULL) {
        printf("Error: Invalid set command format - missing '='\r\n");
        return;
    }
    
    // 分割参数名和值
    *equal_pos = '\0';
    value_str = equal_pos + 1;
    
    // 去除参数名末尾的空格
    char* param_end = param_name + strlen(param_name) - 1;
    while (param_end > param_name && (*param_end == ' ' || *param_end == '\t')) {
        *param_end = '\0';
        param_end--;
    }
    
    // 去除值开头的空格
    while (*value_str == ' ' || *value_str == '\t') value_str++;
    
    // 转换数值
    float value = atof(value_str);
    
    // 设置参数
    int motor_index = 0; // 默认使用电机0
    if (motor_id != NULL) {
        if (strcmp(motor_id, "motor0") == 0) {
            motor_index = 0;
        } else if (strcmp(motor_id, "motor1") == 0) {
            motor_index = 1;
        } else {
            printf("Error: Motor '%s' not found\r\n", motor_id);
            return;
        }
    }
    
    // 调用参数设置函数
    MotorParams_SetParam(motor_index, param_name, value);
 
}

void Command_Parse(char* command_line)
{
    if (command_line == NULL || strlen(command_line) == 0) {
        return;
    }
    
    // 去除首尾空白字符
    char* cmd_start = command_line;
    while (*cmd_start == ' ' || *cmd_start == '\t') cmd_start++;
    
    char* cmd_end = cmd_start + strlen(cmd_start) - 1;
    while (cmd_end > cmd_start && (*cmd_end == ' ' || *cmd_end == '\t' || *cmd_end == '\r' || *cmd_end == '\n')) cmd_end--;
    *(cmd_end + 1) = '\0';
    
    if (strlen(cmd_start) == 0) {
        return;
    }
    
    // 检查是否为motorX格式（motor1, motor0等）
    if (strncmp(cmd_start, "motor", 5) == 0) {
        char* motor_id_str = cmd_start + 5; // 跳过"motor"
        
        // 检查motor后面是否直接跟着数字
        if (strlen(motor_id_str) > 0 && motor_id_str[0] >= '0' && motor_id_str[0] <= '9') {
            // 直接调用handle_motor_command处理motorX格式
            handle_motor_command(motor_id_str);
            return;
        }
    }
    
    // 分割命令和参数
    char* cmd = cmd_start;
    char* args = NULL;
    
    char* space_pos = strchr(cmd_start, ' ');
    if (space_pos != NULL) {
        *space_pos = '\0';
        args = space_pos + 1;
        
        // 去除参数前的空白
        while (*args == ' ' || *args == '\t') args++;
    }
    
    // 命令分发
    if (strcmp(cmd, "plot") == 0) {
        handle_plot_command(args);
    } else if (strcmp(cmd, "motor") == 0) {
        handle_motor_command(args);
    } else if (strcmp(cmd, "set") == 0) {
        handle_set_command(args);
    } else {
        printf("Error: Unknown command '%s'\r\n", cmd);
        printf("Available commands: plot, motor, set\r\n");
    }
}