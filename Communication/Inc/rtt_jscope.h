/*============================================================================
    File Name     : rtt_jscope.h
    Description   : SEGGER RTT 用于 J-Scope 波形显示的封装模块
    Author        : ZHOUHENG
    Date          : 2025-11-28
    ----------------------------------------------------------------------
    功能说明：
    1. 封装 SEGGER RTT 用于 J-Scope 实时波形显示
    2. 支持多通道浮点数据发送
    3. 提供与 VOFA 类似的接口，便于切换

    使用方法：
    1. 调用 RTT_JScope_Init() 初始化
    2. 调用 RTT_JScope_SendData() 发送浮点数组
    3. 在 J-Scope 中配置对应的通道数和采样率

    注意事项：
    - RTT 发送速度极快（~1MB/s），不会阻塞
    - 支持在中断中调用（RTT 内部有保护机制）
    - 缓冲区满时会丢弃旧数据（SEGGER_RTT_MODE_NO_BLOCK_SKIP）
*=============================================================================
*/

#ifndef __RTT_JSCOPE_H__
#define __RTT_JSCOPE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


/* RTT 通道配置 */
#define RTT_JSCOPE_CHANNEL          1       // 使用 RTT 通道 1（通道 0 通常用于 printf）
#define RTT_JSCOPE_BUFFER_SIZE      2048    // RTT 上行缓冲区大小（字节）

/* J-Scope 数据格式配置 */
#define RTT_JSCOPE_MAX_CHANNELS     16      // 最大支持通道数


int RTT_JScope_Init(void);

/**
 * @brief 发送浮点数组到 J-Scope
 * @param data: 浮点数组指针
 * @param count: 数组元素个数（通道数）
 * @note 数据格式：连续的 float 数组，J-Scope 会自动解析
 * @retval 实际发送的字节数
 */
uint32_t RTT_JScope_SendData(const float *data, uint32_t count);

/**
 * @brief 发送单个浮点数到 J-Scope
 * @param channel: 通道索引（0-15）
 * @param value: 浮点数值
 * @note 适用于单通道发送场景
 * @retval 实际发送的字节数
 */
uint32_t RTT_JScope_SendSingle(uint8_t channel, float value);


#ifdef __cplusplus
}
#endif

#endif /* __RTT_JSCOPE_H__ */
