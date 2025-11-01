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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN VOFA_Com_ThreadX_Init */


  ret = tx_semaphore_create(&vofa_timer_semaphore, "VOFA Timer Semaphore", 0);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  /* USER CODE END VOFA_Com_ThreadX_Init */

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

  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    /* 等待定时器信号量 */
    tx_semaphore_get(&vofa_timer_semaphore, TX_WAIT_FOREVER);
    
    /* 这里可以添加VOFA通信处理代码 */
    /* 例如：发送数据到VOFA上位机 */
  }

  /* USER CODE END vofa_com_thread_entry */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */