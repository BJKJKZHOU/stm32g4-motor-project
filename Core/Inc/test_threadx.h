/*============================================================================
    File Name     : test_threadx.h
    Description   : 测试线程 - 用于测试其他模块
    Author        : ZHOUHENG
    Date          : 2025-11-10
    ----------------------------------------------------------------------      
          
     
*=============================================================================
*/

#ifndef __TEST_THREADX_H__
#define __TEST_THREADX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tx_api.h"

/* 测试线程配置 */
#define TEST_THREAD_STACK_SIZE                  512
#define TEST_THREAD_PRIO                        9

#ifndef TEST_THREAD_PREEMPTION_THRESHOLD
#define TEST_THREAD_PREEMPTION_THRESHOLD        TEST_THREAD_PRIO
#endif

#ifndef TEST_THREAD_TIME_SLICE
#define TEST_THREAD_TIME_SLICE                  TX_NO_TIME_SLICE
#endif

#ifndef TEST_THREAD_AUTO_START
#define TEST_THREAD_AUTO_START                  TX_AUTO_START
#endif

/* 导出函数原型 */
UINT TEST_ThreadX_Init(VOID *memory_ptr);
void test_thread_entry(ULONG thread_input);

#ifdef __cplusplus
}
#endif
#endif /* __TEST_THREADX_H__ */
