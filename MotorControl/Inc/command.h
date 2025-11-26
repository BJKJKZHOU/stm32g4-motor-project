/*============================================================================
    File Name     : command.h
    Description   : 命令解析模块 - 基于状态机的命令解析器
    Author        : 
    Date          : 2025-11-03
    ----------------------------------------------------------------------      
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
     - set motor0 enable  //11-8新增，设置电机为使用状态，后续计算使用此电机参数    
     - set motor0 disable

    特性：
     - 不区分大小写
     - 支持motor命令简写（m0, m1）
     - 灵活的set命令格式（支持空格和等号的不同组合） 

     - TODO 1、待添加，电机启动命令，m0 run
     - TODO 2、待添加，相关参数打印系统，不与电机参数系统一起打印，是单独的命令，打印激活电机的
        PI控制器的控制参数，电流速度环的指令参数，位置和速度规划的相关输入参数
        (加/减速度，目标速度/位置，什么轨迹，是否是持续运行等)
     - TODO 3、待添加，第二点的参数需要集成到set命令中，可以通过set命令设置修改
        修改后仍然需要返回修改后的参数
     
*=============================================================================
*/

#ifndef COMMAND_H
#define COMMAND_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


void Command_Init(void);

void Command_Parse(const char* command_line);


const char* Command_GetLastError(void);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_H
