/*============================================================================
    File Name     : MotorControl.c
    Description   : 电机控制相关线程实现
    Author        : ZHOUHENG
    Date          : 2025-11-11
    ----------------------------------------------------------------------      
          
     
*=============================================================================
*/

#include "motor_control_threadx.h"
#include "FOC_Loop.h"
#include "main.h"


// 定义全局电流环实例
CurrentLoop_t g_CurrentLoop;

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
  UNUSED(thread_input);
  
  // 电流环初始化
  CurrentLoop_Init(&g_CurrentLoop,0, TPWM_PERIOD,   
                  10.0f, 100.0f,  // d轴PID: kp=10, ki=100
                  10.0f, 100.0f,  // q轴PID: kp=10, ki=100
                  100.0f);         // 观测器gamma=100

  // 启动电流环
  g_CurrentLoop.is_running = true;



  while (1)
  {
    // 设置电流环设定值（示例：id_ref=0, iq_ref=0.5）
    g_CurrentLoop.id_setpoint = 0.0f;    // d轴电流设定值
    g_CurrentLoop.iq_setpoint = 0.5f;    // q轴电流设定值
    
    // 线程休眠，等待下一次控制周期
    tx_thread_sleep(1);  // 休眠10个tick，具体时间根据系统配置调整
  }
}

