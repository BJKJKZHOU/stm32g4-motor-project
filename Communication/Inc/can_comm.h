/*============================================================================
    File Name     : can_comm.h
    Description   : CAN 通信模块 - 用于电机控制系统的 CAN 总线通信
    Author        : ZHOUHENG
    Date          : 2025-11-27
    ----------------------------------------------------------------------
    CAN的发送放在 vofa_com_threadx.c 线程，这个是通讯线程
    TODO  待实现 及测试
*=============================================================================
*/

#ifndef CAN_COMM_H
#define CAN_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
   CAN ID 定义
** ============================================================================
*/

// MCU 发送到上位机的 CAN ID（按键测试消息）
#define CAN_ID_MCU_TO_HOST      0x100

// 上位机发送到 MCU 的 CAN ID（LED 控制命令）
#define CAN_ID_HOST_TO_MCU      0x001  // 修改为 0x001，与上位机一致

/* ============================================================================
   数据结构定义
** ============================================================================
*/

// CAN 测试消息结构（MCU -> 上位机）
typedef struct {
  uint8_t msg_type;       // 消息类型：0x01 = 按键测试
  uint8_t button_state;   // 按键状态：0x01 = 按下
  uint16_t counter;       // 计数器（用于测试）
  uint32_t timestamp;     // 时间戳（毫秒）
} CAN_TestMsg_t;

// CAN 响应消息结构（上位机 -> MCU）
typedef struct {
  uint8_t msg_type;       // 消息类型：0x02 = 响应
  uint8_t ack;            // 确认：0x01 = 收到
  uint8_t led_cmd;        // LED 命令：0x01 = 翻转
  uint8_t reserved[5];    // 保留字节
} CAN_ResponseMsg_t;

/* ============================================================================
   函数声明
** ============================================================================
*/

/**
 * @brief  发送按键测试消息到上位机
 * @note   CAN 硬件初始化在 main.c 的 MX_FDCAN1_Init() 中完成
 * @retval HAL_StatusTypeDef: HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef CAN_SendButtonTest(void);

/**
 * @brief  处理接收到的 CAN 消息
 * @param  id: CAN ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @note   此函数在 fdcan.c 的 HAL_FDCAN_RxFifo0Callback() 中被调用
 */
void CAN_ProcessReceivedMessage(uint32_t id, uint8_t *data, uint8_t len);



#ifdef __cplusplus
}
#endif

#endif /* CAN_COMM_H */
