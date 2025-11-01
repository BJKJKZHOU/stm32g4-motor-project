#include "Vofa.h"
#include "usart.h"
#include "Vofa_STM32G474.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>  // 添加stdio.h以支持snprintf

extern UART_HandleTypeDef hlpuart1;

// 通道数据存储数组（先声明）
static float channel_data[MAX_RECEIVING_CHANNELS] = {0};

// 通道名称存储数组（先声明）
static char channel_names[MAX_RECEIVING_CHANNELS][32] = {
    "RECEIVING_CHANNEL_0", "RECEIVING_CHANNEL_1", "RECEIVING_CHANNEL_2", "RECEIVING_CHANNEL_3",
    "RECEIVING_CHANNEL_4", "RECEIVING_CHANNEL_5", "RECEIVING_CHANNEL_6", "RECEIVING_CHANNEL_7",
    "RECEIVING_CHANNEL_8", "RECEIVING_CHANNEL_9", "RECEIVING_CHANNEL_10", "RECEIVING_CHANNEL_11",
    "RECEIVING_CHANNEL_12", "RECEIVING_CHANNEL_13", "RECEIVING_CHANNEL_14", "RECEIVING_CHANNEL_15"};

// 双缓冲接收缓冲区
uint8_t rx_buffer[2][RX_BUFFER_SIZE];
volatile uint8_t active_buffer = 0;           // 当前活动缓冲区索引
volatile uint16_t rx_data_length[2] = {0, 0}; // 两个缓冲区的数据长度

static volatile uint8_t dma_tx_complete = 1;

// 发送数据回调函数 - 保持不变
void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length)
{
    (void)handle;
    // 等待之前的传输完成
    while (!dma_tx_complete)
    {
    }
    dma_tx_complete = 0;
    HAL_UART_Transmit_DMA(&hlpuart1, data, length);
}

 
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1)
    {
        dma_tx_complete = 1;
    }
}

uint16_t Vofa_GetReceivedData(uint8_t *buffer, uint16_t buffer_len)
{
    // 原子读取当前活动缓冲区索引和非活动缓冲区的数据长度
    uint8_t inactive_buffer;
    uint16_t copy_len = 0;
    
    // 进入临界区，防止DMA回调在读取过程中切换缓冲区
    __disable_irq();
    uint8_t current_active = active_buffer;
    inactive_buffer = current_active ^ 1;
    uint16_t data_len = rx_data_length[inactive_buffer];
    __enable_irq();

    if (data_len == 0)
    {
        return 0;
    }

    copy_len = (data_len < buffer_len) ? data_len : buffer_len;
    memcpy(buffer, rx_buffer[inactive_buffer], copy_len);

    // 清空已读取的数据长度，实现单次消费语义
    rx_data_length[inactive_buffer] = 0;

    // 返回实际复制的长度
    return copy_len;
}

void Vofa_STM32G474_Init(void)
{
    // 清空双缓冲区
    memset(rx_buffer[0], 0, RX_BUFFER_SIZE);
    memset(rx_buffer[1], 0, RX_BUFFER_SIZE);
    rx_data_length[0] = 0;
    rx_data_length[1] = 0;
    active_buffer = 0;

    // 启动双缓冲DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rx_buffer[active_buffer], RX_BUFFER_SIZE);
}

// 双缓冲接收完成回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != LPUART1)
        return;

    // 记录当前活动缓冲区的数据长度
    rx_data_length[active_buffer] = (Size < RX_BUFFER_SIZE) ? Size : RX_BUFFER_SIZE;

    // 切换到另一个缓冲区
    active_buffer ^= 1;

    // 使用新的缓冲区重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer[active_buffer], RX_BUFFER_SIZE);
}

// 任务上下文中的协议解析函数（修复数据竞争）
void Vofa_ParseReceivedData(void)
{
    // 使用局部变量确保一致性
    uint8_t inactive_buffer;
    uint16_t data_len;
    
    // 进入临界区，防止DMA回调在读取过程中切换缓冲区
    __disable_irq();
    inactive_buffer = active_buffer ^ 1;
    data_len = rx_data_length[inactive_buffer];
    __enable_irq();
    
    if (data_len > 0 && data_len <= RX_BUFFER_SIZE) {
        uint8_t local_buffer[RX_BUFFER_SIZE];
        memcpy(local_buffer, rx_buffer[inactive_buffer], data_len);
        
        // 快速清空标记
        rx_data_length[inactive_buffer] = 0;
        
        Vofa_ParseCustomProtocol(local_buffer, data_len);
    }
}


// 自定义协议解析函数
void Vofa_ParseCustomProtocol(uint8_t *data, uint16_t length)
{
    if (length == 0)
        return;

    // 将接收到的数据转换为字符串（确保以NULL结尾）
    char str_buffer[RX_BUFFER_SIZE + 1];
    memcpy(str_buffer, data, length);
    str_buffer[length] = '\0';

    char *ptr = str_buffer;
    char *end_ptr = str_buffer + length;

    while (ptr < end_ptr)
    {
        // 查找帧头 "ch"
        char *ch_pos = strstr(ptr, "ch");
        if (ch_pos == NULL)
            break;

        ptr = ch_pos + 2; // 跳过 "ch"

        // 解析通道号
        uint8_t channel_num = 0;
        while (ptr < end_ptr && isdigit((unsigned char)*ptr))
        {
            channel_num = channel_num * 10 + (*ptr - '0');
            ptr++;
        }

        // 检查分隔符 '='
        if (ptr >= end_ptr || *ptr != '=')
            continue;
        ptr++; // 跳过 '='

        // 解析浮点数
        char *num_start = ptr;
        while (ptr < end_ptr && (*ptr == '.' || isdigit((unsigned char)*ptr) || *ptr == '-' || *ptr == '+'))
        {
            ptr++;
        }

        if (num_start < ptr)
        {
            // 提取浮点数
            char num_str[32];
            uint16_t num_len = ptr - num_start;
            if (num_len >= sizeof(num_str))
                num_len = sizeof(num_str) - 1;
            memcpy(num_str, num_start, num_len);
            num_str[num_len] = '\0';

            float value = strtof(num_str, NULL);

            // 存储到对应的通道
            if (channel_num < MAX_RECEIVING_CHANNELS)
            {
                channel_data[channel_num] = value;
            }
        }

        // 检查是否有下一个通道数据（分隔符 ';'）
        if (ptr < end_ptr && *ptr == ';')
        {
            ptr++; // 跳过 ';'
        }
        else
        {
            break;
        }
    }
}

// 获取指定通道的数据
float Vofa_GetChannelData(Vofa_ChannelTypeDef channel)
{
    if (channel < MAX_RECEIVING_CHANNELS)
    {
        return channel_data[channel];
    }
    return 0.0f;
}

// 修改通道名称
uint8_t Vofa_SetChannelName(uint8_t channel_id, const char *new_name)
{
    if (channel_id < MAX_RECEIVING_CHANNELS && new_name != NULL)
    {
        // 即使新名称为空，也允许设置
        strncpy(channel_names[channel_id], new_name, 31);
        channel_names[channel_id][31] = '\0';
        return 1; // 成功
    }
    return 0; // 失败
}

// 获取通道名称
const char *Vofa_GetChannelName(uint8_t channel_id)
{
    if (channel_id < MAX_RECEIVING_CHANNELS)
    {
        return channel_names[channel_id];
    }
    return ""; // 返回空字符串
}