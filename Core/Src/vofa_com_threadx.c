/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa_com_threadx.c
  * @author  MCD Application Team
  * @brief   VOFA communication thread implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "vofa_com_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TX_THREAD vofa_com_thread;
/* USER CODE BEGIN PV */

/* 信号量定义 */
TX_SEMAPHORE vofa_timer_semaphore;

/* Vofa句柄 */
Vofa_HandleTypedef vofa_handle;

#define TEST_DATA_COUNT 10
static float test_data[TEST_DATA_COUNT] = {0};
static uint32_t time_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void Vofa_UpdateTestData(void);

/* USER CODE END PFP */

/**
  * @brief  VOFA Communication ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT VOFA_Com_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  /* USER CODE BEGIN VOFA_Com_ThreadX_MEM_POOL */

  /* USER CODE END VOFA_Com_ThreadX_MEM_POOL */
  CHAR *pointer;

  /* USER CODE BEGIN VOFA_Com_ThreadX_Init */

  // 先创建信号量，避免竞态条件
  ret = tx_semaphore_create(&vofa_timer_semaphore, "VOFA Timer Semaphore", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  /* USER CODE END VOFA_Com_ThreadX_Init */

  /* Allocate the stack for vofa com thread  */
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
void vofa_com_thread_entry(ULONG thread_input)
{
  /* USER CODE BEGIN vofa_com_thread_entry */
  Vofa_Init(&vofa_handle, VOFA_MODE_BLOCK_IF_FIFO_FULL);

  Vofa_SetChannelName(0, "Sine_Wave");
  Vofa_SetChannelName(1, "Cosine_Wave");
  Vofa_SetChannelName(2, "Triangle_Wave");
  Vofa_SetChannelName(3, "Square_Wave");
  Vofa_SetChannelName(4, "Sawtooth_Wave");
  Vofa_SetChannelName(5, "Random_Noise");
  Vofa_SetChannelName(6, "Exponential");
  Vofa_SetChannelName(7, "Logarithmic");
  Vofa_SetChannelName(8, "Pulse");
  Vofa_SetChannelName(9, "Ramp");


  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    /* 等待定时器信号量 */
    tx_semaphore_get(&vofa_timer_semaphore, TX_WAIT_FOREVER);

    /* 更新测试数据 */
    Vofa_UpdateTestData();
    
    /* 发送数据到VOFA+ */
    Vofa_JustFloat(&vofa_handle, test_data, TEST_DATA_COUNT);

    time_counter++;

    tx_thread_sleep(1);
  }

  /* USER CODE END vofa_com_thread_entry */
}

/* USER CODE BEGIN 1 */

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
}

/* USER CODE END 1 */