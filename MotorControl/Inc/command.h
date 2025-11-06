/*============================================================================
    负责解析上位机发送的命令，根据命令执行相应的操作。
    上位机发送的是字符串命令，通过串口接收。
    调用vofa的数据接收功能，解析协议自定义。
 =============================================================================
 * @brief 命令解析模块 - 支持的命令列表
 * 命令不区分大小写
 *  * 示例：
 *   - plot         //命令检查通过，可以使用
 *   - plot stop    //命令检查通过，可以使用
 *   - motor 0或motor0  //命令检查通过，可以使用
 *   - set motor0 Rs = 0.5 //命令检查通过，可以使用
 *   - set motor1 Lq=0.003  //命令检查通过，可以使用
 *   - set motor1 Flux = 2.5 //命令检查通过，可以使用
 *   - set P1001=0.8
 *     set P1001 = 0.8
 * 1. plot命令 - 控制VOFA数据绘图
 *    - plot             启动VOFA数据绘图
 *    - plot stop        停止VOFA数据绘图
 * 
 * 2. motor命令 - 显示电机参数信息
 *    - motor <motor_id> 显示指定电机的所有参数
 *    - 支持格式：motor0, motor1,m1,M1
 *    - 可用电机：0, 1
 * 
 * 3. set命令 - 设置电机参数值
 *    - set <motor_id> <param_name> = <value> 设置指定电机的参数值
 *    - set <param_name> = <value>            设置默认电机0的参数值
 *    - 支持无空格格式：set motor1 Rs=0.8
 *    - 支持参数：Rs, Ls, Flux, Flux(别名), P1001(HMI代码)
 =============================================================================
 */



#ifndef COMMAND_H
#define COMMAND_H

#include "Vofa_STM32G474.h"
#include "motor_params.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 解析并执行命令
 * @param command_line 命令字符串
 */
void Command_Parse(char* command_line);

#ifdef __cplusplus
}

#endif

#endif // COMMAND_H