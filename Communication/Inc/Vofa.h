/*
	MIT License
	Copyright (c) 2021 Jelin
*/
#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* User configuration params */
#define VOFA_BUFFER_SIZE 128	   // Send/Receive data buffer length
#define VOFA_CMD_TAIL {0x00, 0x00, 0x00, 0x00} // Command frame tail

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct
	{
		uint8_t buffer[VOFA_BUFFER_SIZE]; // Data buffer
		uint8_t *rp;					  // Read data buffer pointer
		uint8_t *wp;					  // Write data buffer pointer
		bool overflow;					  // FIFO overflow sign bit
	} Vofa_FIFOTypeDef;

	typedef enum
	{
		VOFA_MODE_SKIP,				 // Do not block
		VOFA_MODE_BLOCK_IF_FIFO_FULL // Wait until there is space in the buffer.
	} Vofa_ModeTypeDef;

	typedef struct
	{
		uint8_t txBuffer[VOFA_BUFFER_SIZE];
		Vofa_FIFOTypeDef rxBuffer;
		Vofa_ModeTypeDef mode;
	} Vofa_HandleTypedef;

	/* initialization/configuration functions */
	void Vofa_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode);

	/* send data functions */
	void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num);


	/* user call back functions */
	void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length);
	uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle);

	/* channel data management functions */
	typedef enum
	{
		RECEIVING_CHANNEL_0 = 0,
		RECEIVING_CHANNEL_1 = 1,
		RECEIVING_CHANNEL_2 = 2,
		RECEIVING_CHANNEL_3 = 3,
		RECEIVING_CHANNEL_4 = 4,
		RECEIVING_CHANNEL_5 = 5,
		RECEIVING_CHANNEL_6 = 6,
		RECEIVING_CHANNEL_7 = 7,
		RECEIVING_CHANNEL_8 = 8,
		RECEIVING_CHANNEL_9 = 9,
		RECEIVING_CHANNEL_10 = 10,
		RECEIVING_CHANNEL_11 = 11,
		RECEIVING_CHANNEL_12 = 12,
		RECEIVING_CHANNEL_13 = 13,
		RECEIVING_CHANNEL_14 = 14,
		RECEIVING_CHANNEL_15 = 15,
		MAX_RECEIVING_CHANNELS = 16
	} Vofa_ChannelTypeDef;


#ifdef __cplusplus
}
#endif

#endif // VOFA_H