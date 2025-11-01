/*
	MIT License
	Copyright (c) 2021 Jelin
*/

#include "Vofa.h"
#include "Vofa_STM32G474.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// static const uint8_t cmdTail[] = VOFA_CMD_TAIL; //帧头未使用

static const uint8_t justFloatTail[4] = {0x00, 0x00, 0x80, 0x7f};

void Vofa_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode)
{
    if (handle == NULL)
        return;
        
    // 初始化FIFO缓冲区
    memset(handle->rxBuffer.buffer, 0, VOFA_BUFFER_SIZE);
    handle->rxBuffer.rp = handle->rxBuffer.buffer;
    handle->rxBuffer.wp = handle->rxBuffer.buffer;
    handle->rxBuffer.overflow = false;
    
    // 设置模式
    handle->mode = mode;
    
    // 初始化TX缓冲区
    memset(handle->txBuffer, 0, VOFA_BUFFER_SIZE);
    
    // 调用硬件初始化
    Vofa_STM32G474_Init();
}

void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num)
{
    if (handle == NULL || data == NULL || num == 0)
        return;
        
    uint16_t data_size = num * sizeof(float);
    
    // 检查缓冲区大小
    if (data_size > VOFA_BUFFER_SIZE - 4) {  // 减去尾帧大小
        // 处理溢出情况
        return;
    }
    
    // 复制数据到TX缓冲区
    memcpy(handle->txBuffer, data, data_size);
    memcpy(handle->txBuffer + data_size, justFloatTail, 4);
    
    // 发送数据
    Vofa_SendDataCallBack(handle, handle->txBuffer, data_size + 4);
}

/* Commented out other send protocols - moved to end of file */

// 简化的数据读取函数 - 使用双缓冲DMA机制
uint16_t Vofa_ReadData(uint8_t *buffer, uint16_t bufferLen)
{
	// 调用硬件抽象层的双缓冲数据获取函数
	return Vofa_GetReceivedData(buffer, bufferLen);
}

/* Commented out other receive protocols - moved to end of file */

#ifdef __GNUC__
__attribute__((weak)) void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length)
{
	(void)handle;
	(void)data;
	(void)length;
	return;
}

__attribute__((weak))
uint8_t
Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
	(void)handle;
	return 0;
}
#endif