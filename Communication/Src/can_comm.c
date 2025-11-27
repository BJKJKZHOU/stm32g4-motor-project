/*============================================================================
    File Name     : can_comm.c
    Description   : CAN 通信模块实现 - 应用层CAN消息处理
    Author        : ZHOUHENG
    Date          : 2025-11-27
    ----------------------------------------------------------------------
    说明：
    - CAN 硬件初始化在 fdcan.c 的 MX_FDCAN1_Init() 中完成
    - CAN 发送使用 fdcan.c 中的 FDCAN_Transmit() 函数
    - 本模块只负责应用层的消息封装和解析
*=============================================================================
*/

#include "can_comm.h"
#include "fdcan.h"


extern HAL_StatusTypeDef FDCAN_Transmit(uint32_t id, uint8_t *data, uint8_t len);



static uint16_t can_test_counter = 0;  // CAN 测试计数器



HAL_StatusTypeDef CAN_SendButtonTest(void)
{
  uint8_t tx_data[8];

  // 填充测试消息
  tx_data[0] = 0x01;                                     // 消息类型：按键测试
  tx_data[1] = 0x01;                                     // 按键状态：按下
  tx_data[2] = (uint8_t)(can_test_counter & 0xFF);       // 计数器低字节
  tx_data[3] = (uint8_t)((can_test_counter >> 8) & 0xFF); // 计数器高字节

  uint32_t timestamp = HAL_GetTick();
  tx_data[4] = (uint8_t)(timestamp & 0xFF);              // 时间戳字节0
  tx_data[5] = (uint8_t)((timestamp >> 8) & 0xFF);       // 时间戳字节1
  tx_data[6] = (uint8_t)((timestamp >> 16) & 0xFF);      // 时间戳字节2
  tx_data[7] = (uint8_t)((timestamp >> 24) & 0xFF);      // 时间戳字节3

  can_test_counter++;  // 递增计数器

  // 调用 fdcan.c 中的发送函数
  return FDCAN_Transmit(CAN_ID_MCU_TO_HOST, tx_data, 8);
}

/**
 * @brief  处理接收到的 CAN 消息
 * @param  id: CAN ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @note   回环模式：接收到自己发送的消息时翻转LED
 */
void CAN_ProcessReceivedMessage(uint32_t id, uint8_t *data, uint8_t len)
{
  // 回环模式：收到消息后翻转LED
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

  // 检查是否是来自上位机的 LED 控制命令（正常模式使用）
  if (id == CAN_ID_HOST_TO_MCU && len >= 2)
  {
    if (data[0] == 0x10 && data[1] == 0x10)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
  }
}

