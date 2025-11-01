/*
	MIT License
	Copyright (c) 2021 Jelin
*/

#include "Vofa.h"
#include "Vofa_STM32G474.h"
#include <stdarg.h>
#include <stdio.h>

static const uint8_t cmdTail[] = VOFA_CMD_TAIL;
static const uint8_t justFloatTail[4] = {0x00, 0x00, 0x80, 0x7f};

void Vofa_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode)
{
	(void)handle;
	(void)mode;

}

void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num)
{
	Vofa_SendDataCallBack(handle, (uint8_t *)data, num * sizeof(float));
	Vofa_SendDataCallBack(handle, (uint8_t *)justFloatTail, 4);
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