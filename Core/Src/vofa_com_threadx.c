
/**
  ==============================================================================
  * @file           : vofa_com_threadx.c
  * @brief          : VOFA Communication ThreadX implementation file
  ==============================================================================
**/

#include "vofa_com_threadx.h"

#include <math.h>

TX_THREAD vofa_com_thread;


/* 信号量定义 */
TX_SEMAPHORE vofa_timer_semaphore;

/* Vofa句柄 */
Vofa_HandleTypedef vofa_handle;

#define TEST_DATA_COUNT 15 //发送到上位机通道数
static float test_data[TEST_DATA_COUNT] = {0};


static uint32_t time_counter = 0;

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


  Vofa_STM32G474_Init(&vofa_handle, VOFA_MODE_BLOCK_IF_FIFO_FULL);

  // 设置接收通道名称
  Vofa_SetChannelName(RECEIVING_CHANNEL_10, "TEST_DATA_1");
  Vofa_SetChannelName(RECEIVING_CHANNEL_11, "TEST_DATA_2"); 
  Vofa_SetChannelName(RECEIVING_CHANNEL_12, "TEST_DATA_3");

  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    /* 等待定时器信号量 */
    tx_semaphore_get(&vofa_timer_semaphore, TX_WAIT_FOREVER);
 
    /* 更新测试数据 */
    Vofa_UpdateTestData();
    
    /* 发送数据到VOFA+ */
    Vofa_JustFloat(&vofa_handle, test_data, TEST_DATA_COUNT);

    time_counter++;  //测试数据的时间计数器

    tx_thread_sleep(1);
  }


}



void Vofa_UpdateTestData(void)
{
  float time = time_counter * 0.1f;  // 时间基准，每100ms增加0.1
  
  /* 1. 正弦波 */
  test_data[0] = sinf(2.0f * M_PI * 0.5f * time);
  
  /* 2. 余弦波 */
  test_data[1] = cosf(2.0f * M_PI * 0.5f * time);
  
  /* 3. 三角波 */
  test_data[2] = 2.0f * fabsf(fmodf(time, 2.0f) - 1.0f) - 1.0f;
  
  /* 4. 方波 */
  test_data[3] = (fmodf(time, 2.0f) < 1.0f) ? 1.0f : -1.0f;
  
  /* 5. 锯齿波 */
  test_data[4] = fmodf(time, 2.0f) - 1.0f;
  
  /* 6. 随机噪声 */
  test_data[5] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
  
  /* 7. 指数函数 */
  test_data[6] = expf(-0.5f * fmodf(time, 4.0f));
  
  /* 8. 对数函数 */
  test_data[7] = logf(1.0f + fabsf(sinf(2.0f * M_PI * 0.2f * time)));
  
  /* 9. 脉冲信号 */
  test_data[8] = (fmodf(time, 4.0f) < 0.1f) ? 1.0f : 0.0f;
  
  /* 10. 斜坡信号 */
  test_data[9] = fmodf(time * 0.5f, 2.0f) - 1.0f;

  /* 11. 接收数据1的返回（通道10） */
  test_data[10] = Vofa_GetChannelData(RECEIVING_CHANNEL_0); 
  
  /* 12. 接收数据2的返回（通道11） */
  test_data[11] = Vofa_GetChannelData(RECEIVING_CHANNEL_1); 
  
  /* 13. 接收数据3的返回（通道12） */
  test_data[12] = Vofa_GetChannelData(RECEIVING_CHANNEL_2);   
    
  // /* 14. 接收数据总和 */
  // test_data[13] = ch10_data + ch11_data + ch12_data;
  
  // /* 15. 接收数据平均值 */
  // test_data[14] = (ch10_data + ch11_data + ch12_data) / 3.0f;
}
