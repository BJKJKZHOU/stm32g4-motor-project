/*============================================================================
    File Name     : command.h
    Description   : 命令解析模块 - 基于状态机的命令解析器
    Author        : 
    Date          : 2025-11-04
*=============================================================================
*/

#ifndef COMMAND_H
#define COMMAND_H

#include "motor_params.h"
#include "Vofa_STM32G474.h"
#include "normalization.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
    命令解析模块 - 支持的命令列表
    
    示例：
     - plot         // 启动VOFA数据绘图
     - plot stop    // 停止VOFA数据绘图

     - 使用需要显示信息的命令时需要将vofa的协议切换到 FireWater
     - MCU 接收命令不需要切换协议
     
     - motor 0      // 显示电机0参数
     - motor0       // 显示电机0参数（简写）
     - m0           // 显示电机0参数（简写）
     - set motor0 Rs = 0.5  // 设置电机0的Rs参数
     - set m1 Lq=0.003      // 设置电机1的Lq参数
     - set Flux = 2.5       // 设置默认电机0的Flux参数

    特性：
     - 不区分大小写
     - 支持motor命令简写（m0, m1）
     - 灵活的set命令格式（支持空格和等号的不同组合）
=============================================================================
 */

/**
 * @brief 命令解析模块初始化
 */
void Command_Init(void);

/**
 * @brief 解析并执行命令字符串
 * @param command_line 命令字符串（以'\0'结尾）
 */

void Command_Parse(const char* command_line);

/**
 * @brief 获取最后一条错误信息
 * @return 错误信息字符串
 */
const char* Command_GetLastError(void);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_H