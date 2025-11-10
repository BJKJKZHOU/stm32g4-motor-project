/*============================================================================
    File Name     : test_threadx.c
    Description   : 测试线程模块 - 用于测试其他模块
    Author        : ZHOUHENG
    Date          : 2025-11-10
    ----------------------------------------------------------------------      
          
     
*=============================================================================
*/

#include "test_threadx.h"

TX_THREAD test_thread;

UINT TEST_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  CHAR *pointer;

  /* Allocate the stack for test thread.  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       TEST_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  /* Create test thread.  */
  if (tx_thread_create(&test_thread, "test thread", test_thread_entry, 0, pointer,
                       TEST_THREAD_STACK_SIZE, TEST_THREAD_PRIO, TEST_THREAD_PREEMPTION_THRESHOLD,
                       TEST_THREAD_TIME_SLICE, TEST_THREAD_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  return ret;
}

// 测试线程入口
void test_thread_entry(ULONG thread_input)
{
  
  while (1)
  {




  }
}
