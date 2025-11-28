/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_CLK_MHz 170
#define PWM_FREQUENCY 20000

/* USER CODE BEGIN Private defines */



#define ARR_PERIOD ((uint32_t)((TIM_CLK_MHz * 1000000.0f / PWM_FREQUENCY) / 2.0f)) // =4250

#define TPWM_PERIOD (1.0f/PWM_FREQUENCY) // 1/20000 = 0.00005s

#define PI 3.14159265358979323846f

// ADC电流采样触发点配置
#define TIM1_DEADTIME       85      // 死区时间（时钟周期）
#define ADC_SAMPLE_TIME     415     // ADC 16倍过采样时间（约2.44us @ 170MHz）

// ADC触发点配置（中心对齐模式，触发后采样可跨越ARR峰值）
#define TRIGGER_MARGIN      300     // 触发点距ARR的裕量，提供600个计数的采样窗口（向上+向下）
#define TRIGGER_HIGH_SIDE   ((uint32_t)(ARR_PERIOD - TRIGGER_MARGIN))  // 高侧触发点（ARR-300）

// 占空比阈值（基于ARR_PERIOD=4249）
#define DUTY_THRESHOLD_HIGH ((uint32_t)(TRIGGER_HIGH_SIDE - TIM1_DEADTIME))  // 约3864，90%占空比
#define DUTY_THRESHOLD_LOW  ((uint32_t)(ARR_PERIOD * 0.10f))  // 10%占空比阈值，约425
#define DUTY_CRITICAL_HIGH  ((uint32_t)(ARR_PERIOD * 0.98f))  // 98%占空比阈值，约4164

/* 用于在CCM中执行的代码 */
#define CCMRAM_FUNC __attribute__((section(".ccmtext")))

/* 用于存放在CCM中的数据 */
#define CCMRAM_DATA __attribute__((section(".ccmram")))

/* 用于存放在CCM中的常量数据 */
#define CCMRAM_CONST __attribute__((section(".ccmram"))) const

/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */

extern volatile uint32_t uwLEDToggleTime;
extern volatile uint8_t LED_Toggle_Flag;
/* USER CODE END Private variables */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
