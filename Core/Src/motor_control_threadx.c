/*============================================================================
    File Name     : MotorControl.c
    Description   : 电机控制相关线程实现
    Author        : ZHOUHENG
    Date          : 2025-11-11
    ----------------------------------------------------------------------      
          
     
*=============================================================================
*/

#include "motor_control_threadx.h"

TX_THREAD motor_control_thread;

UINT MOTOR_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  CHAR *pointer;

  /* Allocate the stack for motor control thread.  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       MOTOR_CONTROL_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  /* Create motor control thread.  */
  if (tx_thread_create(&motor_control_thread, "motor control thread", motor_control_thread_entry, 0, pointer,
                       MOTOR_CONTROL_THREAD_STACK_SIZE, MOTOR_CONTROL_THREAD_PRIO, MOTOR_CONTROL_THREAD_PREEMPTION_THRESHOLD,
                       MOTOR_CONTROL_THREAD_TIME_SLICE, MOTOR_CONTROL_THREAD_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  return ret;
}

// 电机控制线程入口
void motor_control_thread_entry(ULONG thread_input)
{
  // 启动TIM1的更新中断（用于FOC控制循环）
  

  while (1)
  {
     
    
    
    

    tx_thread_sleep(1);    
  }
}