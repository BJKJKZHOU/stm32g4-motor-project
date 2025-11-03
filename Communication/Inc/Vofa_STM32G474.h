#ifndef VOFA_STM32G474_H
#define VOFA_STM32G474_H

#include "Vofa.h"
#include "stm32g4xx_hal.h"
#include <math.h>

// 双缓冲接收配置
#define RX_BUFFER_SIZE 128
extern uint8_t rx_buffer[2][RX_BUFFER_SIZE]; // 双缓冲
extern volatile uint8_t active_buffer;       // 当前活动缓冲区索引
extern volatile uint16_t rx_data_length[2];  // 两个缓冲区的数据长度

void Vofa_STM32G474_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode);

// 获取接收到的原始数据
uint16_t Vofa_GetReceivedData(uint8_t *buffer, uint16_t max_length);

// 任务上下文中的协议解析函数
void Vofa_ParseReceivedData(void);

// 直接从指定缓冲区解析数据
void Vofa_ParseReceivedDataFromBuffer(uint8_t buffer_index);


float Vofa_GetChannelData(Vofa_ChannelTypeDef channel);

// 自定义协议解析函数
void Vofa_ParseCustomProtocol(uint8_t *data, uint16_t length);

uint8_t Vofa_SetChannelName(uint8_t channel_id, const char *new_name);
const char *Vofa_GetChannelName(uint8_t channel_id);

#endif // VOFA_STM32G474_H