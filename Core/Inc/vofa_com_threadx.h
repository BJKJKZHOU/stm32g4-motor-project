/* ======================================================================
  File Name          : vofa_com_threadx.h
  Description        : Header for vofa_com_threadx.c file.
                       This file contains the common defines of the application.
                      vofa串口调试助手的通信线程
   ======================================================================*/

#ifndef __VOFA_COM_THREADX_H__
#define __VOFA_COM_THREADX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"
#include "tim.h"



#include "Vofa_STM32G474.h"


/* VOFA通信线程配置 */
#define VOFA_COM_STACK_SIZE                     1024
#define VOFA_COM_THREAD_PRIO                    8

#ifndef VOFA_COM_THREAD_PREEMPTION_THRESHOLD
#define VOFA_COM_THREAD_PREEMPTION_THRESHOLD    VOFA_COM_THREAD_PRIO
#endif

#ifndef VOFA_COM_THREAD_TIME_SLICE
#define VOFA_COM_THREAD_TIME_SLICE              TX_NO_TIME_SLICE
#endif

#ifndef VOFA_COM_THREAD_AUTO_START
#define VOFA_COM_THREAD_AUTO_START              TX_AUTO_START
#endif

/* 导出函数原型 */
UINT VOFA_Com_ThreadX_Init(VOID *memory_ptr);
void vofa_com_thread_entry(ULONG thread_input);

/* 信号量相关定义*/
extern TX_SEMAPHORE vofa_timer_semaphore;

#ifdef __cplusplus
}
#endif
#endif /* __VOFA_COM_THREADX_H__ */
