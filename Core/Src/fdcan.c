/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;  // 临时改为外部回环模式测试
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  // 配置接收滤波器（接受所有标准ID）
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;  // 滤波器ID
  sFilterConfig.FilterID2 = 0x000;  // 滤波器掩码（0=接受所有标准ID）

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 配置全局滤波器（拒绝不匹配的帧和远程帧）
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                    FDCAN_REJECT,        // 拒绝不匹配的标准ID
                                    FDCAN_REJECT,        // 拒绝不匹配的扩展ID
                                    FDCAN_FILTER_REMOTE, // 过滤远程标准帧
                                    FDCAN_FILTER_REMOTE) // 过滤远程扩展帧
                                    != HAL_OK)
  {
    Error_Handler();
  }

  // 启动 FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // 激活接收中断（FIFO 0 新消息）
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                      0) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief  FDCAN 接收 FIFO 0 回调函数
 * @param  hfdcan: FDCAN 句柄
 * @param  RxFifo0ITs: 中断标志
 * @note   当 FIFO 0 接收到新消息时触发
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    // 从 FIFO 0 读取消息
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      // 调用 CAN 通信模块处理接收到的消息
      extern void CAN_ProcessReceivedMessage(uint32_t id, uint8_t *data, uint8_t len);

      // 计算数据长度（从 DLC 转换为字节数）
      uint8_t data_len = (RxHeader.DataLength >> 16) & 0x0F;
      if (data_len > 8) data_len = 8;  // 限制最大长度为8字节

      CAN_ProcessReceivedMessage(RxHeader.Identifier, RxData, data_len);
    }
  }
}

/**
 * @brief  FDCAN 发送消息
 * @param  id: CAN ID（标准ID，11位）
 * @param  data: 数据指针
 * @param  len: 数据长度（0-8字节）
 * @retval HAL_StatusTypeDef: HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef FDCAN_Transmit(uint32_t id, uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;

  // 限制数据长度为0-8字节
  if (len > 8) len = 8;

  // 将字节长度转换为 FDCAN DLC 值
  uint32_t dlc_value;
  switch (len)
  {
    case 0:  dlc_value = FDCAN_DLC_BYTES_0; break;
    case 1:  dlc_value = FDCAN_DLC_BYTES_1; break;
    case 2:  dlc_value = FDCAN_DLC_BYTES_2; break;
    case 3:  dlc_value = FDCAN_DLC_BYTES_3; break;
    case 4:  dlc_value = FDCAN_DLC_BYTES_4; break;
    case 5:  dlc_value = FDCAN_DLC_BYTES_5; break;
    case 6:  dlc_value = FDCAN_DLC_BYTES_6; break;
    case 7:  dlc_value = FDCAN_DLC_BYTES_7; break;
    case 8:  dlc_value = FDCAN_DLC_BYTES_8; break;
    default: dlc_value = FDCAN_DLC_BYTES_8; break;
  }

  // 配置发送头
  TxHeader.Identifier = id;                      // CAN ID
  TxHeader.IdType = FDCAN_STANDARD_ID;           // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;       // 数据帧
  TxHeader.DataLength = dlc_value;               // 数据长度（使用正确的DLC值）
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // 经典CAN不使用BRS
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;         // 经典CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/* USER CODE END 1 */
