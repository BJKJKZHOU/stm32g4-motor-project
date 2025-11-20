/*============================================================================
    File Name     : current_stub.c
    Description   : 电流采样模块的测试桩
    Author        : ZHOUHENG
    Date          : 2025-11-20
*=============================================================================*/

#include "Current.h"
#include <string.h>

// 全局变量
CurrentSample_t g_CurrentSample = {0};

// 模拟的电流值（用于测试）
static float g_test_ia_pu = 0.0f;
static float g_test_ib_pu = 0.0f;
static float g_test_ic_pu = 0.0f;

/**
 * @brief 电流采样初始化（桩函数）
 */
void Current_Init(void)
{
    memset(&g_CurrentSample, 0, sizeof(CurrentSample_t));
    g_test_ia_pu = 0.0f;
    g_test_ib_pu = 0.0f;
    g_test_ic_pu = 0.0f;
}

/**
 * @brief 获取三相电流（物理值）
 */
void Current_GetABC(float *ia, float *ib, float *ic)
{
    if (ia != NULL) *ia = g_CurrentSample.current_abc[0];
    if (ib != NULL) *ib = g_CurrentSample.current_abc[1];
    if (ic != NULL) *ic = g_CurrentSample.current_abc[2];
}

/**
 * @brief 获取三相电流（标幺值）
 */
void Current_GetPU(float *ia_pu, float *ib_pu, float *ic_pu)
{
    if (ia_pu != NULL) *ia_pu = g_test_ia_pu;
    if (ib_pu != NULL) *ib_pu = g_test_ib_pu;
    if (ic_pu != NULL) *ic_pu = g_test_ic_pu;
}

/**
 * @brief 获取ADC原始值
 */
uint16_t* Current_GetRawADC(void)
{
    return g_CurrentSample.adc_raw;
}

/**
 * @brief 设置测试用的电流值（标幺值）
 * @note 这是测试专用函数，不在头文件中声明
 */
void Current_SetTestValues_PU(float ia_pu, float ib_pu, float ic_pu)
{
    g_test_ia_pu = ia_pu;
    g_test_ib_pu = ib_pu;
    g_test_ic_pu = ic_pu;
}

/**
 * @brief ADC触发点更新（桩函数）
 */
void Update_ADC_Trigger_Point(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3)
{
    // 桩函数，不做实际操作
    (void)duty_ch1;
    (void)duty_ch2;
    (void)duty_ch3;
}
