/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa_com_threadx.h
  * @author  MCD Application Team
  * @brief   VOFA communication thread header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VOFA_COM_THREADX_H__
#define __VOFA_COM_THREADX_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "tx_api.h"
#include "tim.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Vofa_STM32G474.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Private defines -----------------------------------------------------------*/
#define VOFA_COM_STACK_SIZE                     1024
#define VOFA_COM_THREAD_PRIO                    8
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* VOFA communication thread defines ----------------------------------------*/
#ifndef VOFA_COM_THREAD_PREEMPTION_THRESHOLD
#define VOFA_COM_THREAD_PREEMPTION_THRESHOLD    VOFA_COM_THREAD_PRIO
#endif

#ifndef VOFA_COM_THREAD_TIME_SLICE
#define VOFA_COM_THREAD_TIME_SLICE              TX_NO_TIME_SLICE
#endif
#ifndef VOFA_COM_THREAD_AUTO_START
#define VOFA_COM_THREAD_AUTO_START              TX_AUTO_START
#endif
/* USER CODE BEGIN MTD */

/* USER CODE END MTD */

/* Exported macro ------------------------------------------------------------*/

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT VOFA_Com_ThreadX_Init(VOID *memory_ptr);
void vofa_com_thread_entry(ULONG thread_input);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* USER CODE BEGIN 1 */

/* 信号量相关定义 */
extern TX_SEMAPHORE vofa_timer_semaphore;

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __VOFA_COM_THREADX_H__ */