/*============================================================================
    File Name     : MotorControl.h
    Description   : 电机控制相关线程定义
    Author        : ZHOUHENG
    Date          : 2025-11-11
    ----------------------------------------------------------------------      
          
     
*=============================================================================
*/

#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"
#include <stdint.h>
#include "tim.h"

/* 电机控制线程配置 */
#define MOTOR_CONTROL_THREAD_STACK_SIZE                  512
#define MOTOR_CONTROL_THREAD_PRIO                        8

#ifndef MOTOR_CONTROL_THREAD_PREEMPTION_THRESHOLD
#define MOTOR_CONTROL_THREAD_PREEMPTION_THRESHOLD        MOTOR_CONTROL_THREAD_PRIO
#endif

#ifndef MOTOR_CONTROL_THREAD_TIME_SLICE
#define MOTOR_CONTROL_THREAD_TIME_SLICE                  TX_NO_TIME_SLICE
#endif

#ifndef MOTOR_CONTROL_THREAD_AUTO_START
#define MOTOR_CONTROL_THREAD_AUTO_START                  TX_AUTO_START
#endif

/* 导出函数原型 */
UINT MOTOR_ThreadX_Init(VOID *memory_ptr);
void motor_control_thread_entry(ULONG thread_input);

/* 开环测试数据 - 供中断和线程间共享 */
extern volatile uint32_t g_Tcm1;
extern volatile uint32_t g_Tcm2;
extern volatile uint32_t g_Tcm3;

#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_CONTROL_H__ */