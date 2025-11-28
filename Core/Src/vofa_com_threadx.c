/*============================================================================
    File Name     : vofa_com_threadx.c
    Description   : vofa串口调试助手的通信线程·
    Author        : ZHOUHENG
    Date          : 2025-11-03  
    ----------------------------------------------------------------------  
    
    
*=============================================================================
*/

#include "vofa_com_threadx.h"
#include "Current.h"
#include "tim.h"
#include "Vofa_STM32G474.h"


// 声明外部全局变量 - 来自main.c中断
extern volatile uint32_t g_Tcm1;
extern volatile uint32_t g_Tcm2;
extern volatile uint32_t g_Tcm3;

// 声明外部全局变量 - 来自FOC_Loop.c
extern volatile float g_debug_angle;
extern volatile float g_debug_sin;
extern volatile float g_debug_cos;

extern volatile uint8_t sector ;

extern volatile uint32_t g_tim1_interrupt_count;
extern volatile float g_tim1_interrupt_freq_hz;

// 声明外部全局变量 - 来自电流采样模块
extern CurrentSample_t g_CurrentSample;


TX_THREAD vofa_com_thread;


/* 信号量定义 */
TX_SEMAPHORE vofa_timer_semaphore;

/* Vofa句柄 */
Vofa_HandleTypedef vofa_handle;

#define TEST_DATA_COUNT 15 //发送到上位机通道数
static float test_data[TEST_DATA_COUNT] = {0};

// 全局状态变量，控制波形是否开启发送
static uint8_t vofa_justfloat_enabled = 0;


void Vofa_UpdateTestData(void);

UINT VOFA_Com_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  CHAR *pointer;

  // 创建信号量
  ret = tx_semaphore_create(&vofa_timer_semaphore, "VOFA Timer Semaphore", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  /* Allocate the stack for vofa com thread.  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       VOFA_COM_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  /* Create vofa com thread.  */
  if (tx_thread_create(&vofa_com_thread, "vofa com thread", vofa_com_thread_entry, 0, pointer,
                       VOFA_COM_STACK_SIZE, VOFA_COM_THREAD_PRIO, VOFA_COM_THREAD_PREEMPTION_THRESHOLD,
                       VOFA_COM_THREAD_TIME_SLICE, VOFA_COM_THREAD_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  return ret;
}
/**
  * @brief  Function implementing the vofa_com_thread_entry thread.
  * @param  thread_input: Hardcoded to 0.
  * @retval None
  */


// Vofa通信线程入口函数
void vofa_com_thread_entry(ULONG thread_input)
{
  UNUSED(thread_input);

  Vofa_STM32G474_Init(&vofa_handle, VOFA_MODE_BLOCK_IF_FIFO_FULL);

  // 设置接收通道名称
  Vofa_SetChannelName(RECEIVING_CHANNEL_10, "TEST_DATA_1");
  Vofa_SetChannelName(RECEIVING_CHANNEL_11, "TEST_DATA_2"); 
  Vofa_SetChannelName(RECEIVING_CHANNEL_12, "TEST_DATA_3");
  
  // 设置电流采样数据通道名称
  Vofa_SetChannelName(RECEIVING_CHANNEL_13, "Ia_Current");
  Vofa_SetChannelName(RECEIVING_CHANNEL_14, "Ib_Current");
  Vofa_SetChannelName(RECEIVING_CHANNEL_15, "Ic_Current");
  

  //HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    
    tx_semaphore_get(&vofa_timer_semaphore, TX_WAIT_FOREVER);
 
    
    Vofa_UpdateTestData();

    if (vofa_justfloat_enabled){
    
      Vofa_JustFloat(&vofa_handle, test_data, TEST_DATA_COUNT);
    }
     

    tx_thread_sleep(1);
  }


}

void Vofa_UpdateTestData(void)
{
  test_data[0] = 0.0f;
  test_data[1] = (float)g_Tcm1;  // FOC_OpenLoopTest 输出 1 - PWM比较值1
  test_data[2] = (float)g_Tcm2;  // FOC_OpenLoopTest 输出 2 - PWM比较值2
  test_data[3] = (float)g_Tcm3;  // FOC_OpenLoopTest 输出 3 - PWM比较值3
  
  test_data[4] = g_debug_angle;
  test_data[5] = g_debug_sin;
  test_data[6] = g_debug_cos;
  test_data[7] = (float)sector;
  test_data[8] = g_CurrentSample.current_abc[0];  // Ia相电流物理值 (A)
  test_data[9] = g_CurrentSample.current_abc[1];  // Ib相电流物理值 (A)
  test_data[10] = g_CurrentSample.current_abc[2];  // Ic相电流物理值 (A)
  test_data[11] = 1002;
}


void Vofa_Plot_Start(void)
{
    vofa_justfloat_enabled = 1;
}

void Vofa_Plot_Stop(void)
{
    vofa_justfloat_enabled = 0;
}

