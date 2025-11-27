/*============================================================================
    File Name     : can_comm_threadx.c
    Description   : CAN 通信线程实现
    Author        : ZHOUHENG
    Date          : 2025-11-27
*=============================================================================
*/

#include "can_comm_threadx.h"
#include "can_comm.h"

/* 线程句柄 */
TX_THREAD can_comm_thread;

/**
 * @brief  CAN 通信线程初始化
 * @param  memory_ptr: 内存池指针
 * @retval TX_SUCCESS 或错误码
 */
UINT CAN_Comm_ThreadX_Init(VOID *memory_ptr)
{
    UINT ret = TX_SUCCESS;
    TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
    CHAR *pointer;

    /* 分配线程栈空间 */
    if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                         CAN_COMM_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    /* 创建 CAN 通信线程 */
    if (tx_thread_create(&can_comm_thread, "CAN Comm Thread", can_comm_thread_entry, 0, pointer,
                         CAN_COMM_STACK_SIZE, CAN_COMM_THREAD_PRIO, CAN_COMM_THREAD_PREEMPTION_THRESHOLD,
                         CAN_COMM_THREAD_TIME_SLICE, CAN_COMM_THREAD_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return ret;
}

/**
 * @brief  CAN 通信线程入口函数
 * @param  thread_input: 线程输入参数（未使用）
 * @retval None
 */
void can_comm_thread_entry(ULONG thread_input)
{
    UNUSED(thread_input);

    // 初始化 CAN 通信模块
    CAN_Config_t can_config = {
        .tx_enable = true,       // 默认使能发送
        .tx_period_ms = 100,     // 100ms 发送周期
        .rx_enable = true,       // 使能接收
        .motor_id = 0            // 默认电机 ID 0
    };

    CAN_Comm_Init(&can_config);

    // 主循环
    while (1)
    {
        // 执行 CAN 发送任务
        //CAN_Comm_TxTask();

        // 线程休眠（根据发送周期调整）
        tx_thread_sleep(10);  // 10 ticks = 10ms（假设 tick = 1ms）
    }
}
