/*============================================================================
    File Name     : can_comm_threadx.h
    Description   : CAN 通信线程头文件
    Author        : ZHOUHENG
    Date          : 2025-11-27
*=============================================================================
*/

#ifndef CAN_COMM_THREADX_H
#define CAN_COMM_THREADX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"

/* 线程配置参数 */
#define CAN_COMM_STACK_SIZE                 1024
#define CAN_COMM_THREAD_PRIO                15    // 低优先级（高于 VOFA，低于电机控制）
#define CAN_COMM_THREAD_PREEMPTION_THRESHOLD 15
#define CAN_COMM_THREAD_TIME_SLICE          10
#define CAN_COMM_THREAD_AUTO_START          TX_AUTO_START

/* 线程句柄 */
extern TX_THREAD can_comm_thread;


UINT CAN_Comm_ThreadX_Init(VOID *memory_ptr);
void can_comm_thread_entry(ULONG thread_input);

#ifdef __cplusplus
}
#endif

#endif /* CAN_COMM_THREADX_H */
