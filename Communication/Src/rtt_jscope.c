/*============================================================================
    File Name     : rtt_jscope.c
    Description   : SEGGER RTT 用于 J-Scope 波形显示的封装模块实现
    Author        : ZHOUHENG
    Date          : 2025-11-28
*=============================================================================
*/

#include "rtt_jscope.h"
#include <string.h>
#include "SEGGER_RTT.h"

/* RTT 通道缓冲区 */
static uint8_t rtt_jscope_buffer[RTT_JSCOPE_BUFFER_SIZE];

static uint8_t rtt_jscope_initialized = 0;

int RTT_JScope_Init(void)
{
    if (rtt_jscope_initialized) {
        return 0; // 已初始化
    }

    // 配置 RTT 通道 1 用于 J-Scope
    // 参数说明：
    // - RTT_JSCOPE_CHANNEL: 通道索引
    // - "JScope": 通道名称（在 J-Link RTT Viewer 中显示）
    // - rtt_jscope_buffer: 缓冲区指针
    // - RTT_JSCOPE_BUFFER_SIZE: 缓冲区大小
    // - SEGGER_RTT_MODE_NO_BLOCK_SKIP: 缓冲区满时跳过新数据（不阻塞）
    int result = SEGGER_RTT_ConfigUpBuffer(
        RTT_JSCOPE_CHANNEL,
        "JScope",
        rtt_jscope_buffer,
        RTT_JSCOPE_BUFFER_SIZE,
        SEGGER_RTT_MODE_NO_BLOCK_SKIP
    );

    if (result < 0) {
        return -1; // 配置失败
    }

    rtt_jscope_initialized = 1;
    return 0;
}

/**
 * @brief 发送浮点数组到 J-Scope
 */
uint32_t RTT_JScope_SendData(const float *data, uint32_t count)
{
    if (!rtt_jscope_initialized || data == NULL || count == 0) {
        return 0;
    }

    // 限制通道数
    if (count > RTT_JSCOPE_MAX_CHANNELS) {
        count = RTT_JSCOPE_MAX_CHANNELS;
    }

    // 计算数据大小（字节）
    uint32_t data_size = count * sizeof(float);

    // 发送数据到 RTT 通道
    // SEGGER_RTT_Write 是非阻塞的，如果缓冲区满会根据模式处理
    uint32_t bytes_written = SEGGER_RTT_Write(
        RTT_JSCOPE_CHANNEL,
        data,
        data_size
    );

    return bytes_written;
}

/**
 * @brief 发送单个浮点数到 J-Scope
 */
uint32_t RTT_JScope_SendSingle(uint8_t channel, float value)
{
    if (!rtt_jscope_initialized || channel >= RTT_JSCOPE_MAX_CHANNELS) {
        return 0;
    }

    // 创建临时数组
    float data[RTT_JSCOPE_MAX_CHANNELS] = {0};
    data[channel] = value;

    return RTT_JScope_SendData(data, RTT_JSCOPE_MAX_CHANNELS);
}
