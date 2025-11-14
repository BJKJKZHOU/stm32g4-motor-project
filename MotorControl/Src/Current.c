/*============================================================================
    File Name     : Current.c
    Description   : 电流采样相关
    Author        : ZHOUHENG
    Date          : 2025-11-14
    ----------------------------------------------------------------------
    实现三相电流采样、ADC数据处理和标幺值转换
*=============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include "Current.h"
#include "Normalization.h"

/* 全局变量定义 --------------------------------------------------------------*/
CurrentSample_t g_CurrentSample = {0};

/* 私有函数声明 --------------------------------------------------------------*/
static void Current_ADCToPhysical(uint16_t adc_val, float *current);
static void Current_PhysicalToPU(float current, float *current_pu);

/**
 * @brief  电流采样模块初始化
 * @param  None
 * @retval None
 */
void Current_Init(void)
{
    // 清零采样数据结构
    for (int i = 0; i < CURRENT_SAMPLE_CHANNELS; i++)
    {
        g_CurrentSample.adc_raw[i] = CURRENT_OFFSET_ADC;
        g_CurrentSample.current_abc[i] = 0.0f;
        g_CurrentSample.current_pu[i] = 0.0f;
    }
    g_CurrentSample.sample_count = 0;
    g_CurrentSample.sample_valid = 0;
}

/**
 * @brief  获取三相电流物理值
 * @param  ia, ib, ic: 输出三相电流 (A)
 * @retval None
 */
void Current_GetABC(float *ia, float *ib, float *ic)
{
    if (ia) *ia = g_CurrentSample.current_abc[0];
    if (ib) *ib = g_CurrentSample.current_abc[1];
    if (ic) *ic = g_CurrentSample.current_abc[2];
}

/**
 * @brief  获取三相电流标幺值
 * @param  ia_pu, ib_pu, ic_pu: 输出三相电流标幺值
 * @retval None
 */
void Current_GetPU(float *ia_pu, float *ib_pu, float *ic_pu)
{
    if (ia_pu) *ia_pu = g_CurrentSample.current_pu[0];
    if (ib_pu) *ib_pu = g_CurrentSample.current_pu[1];
    if (ic_pu) *ic_pu = g_CurrentSample.current_pu[2];
}

/**
 * @brief  获取ADC原始值
 * @param  None
 * @retval 指向ADC原始值数组的指针
 */
uint16_t* Current_GetRawADC(void)
{
    return g_CurrentSample.adc_raw;
}

/**
 * @brief  ADC值转换为物理电流值
 * @param  adc_val: ADC原始值
 * @param  current: 输出电流值 (A)
 * @retval None
 */
static void Current_ADCToPhysical(uint16_t adc_val, float *current)
{
    // ADC电压 = (ADC值 / 4096) * 3.3V
    float adc_voltage = ((float)adc_val / ADC_RESOLUTION_12BIT) * ADC_VREF;

    // 零点电压（中点电压）
    float offset_voltage = ((float)CURRENT_OFFSET_ADC / ADC_RESOLUTION_12BIT) * ADC_VREF;

    // 差分电压
    float diff_voltage = adc_voltage - offset_voltage;

    // 电流 = 差分电压 / (采样电阻 * 运放增益)
    *current = diff_voltage / (CURRENT_SENSE_RESISTOR * CURRENT_OPAMP_GAIN);
}

/**
 * @brief  物理电流值转换为标幺值
 * @param  current: 物理电流值 (A)
 * @param  current_pu: 输出标幺值
 * @retval None
 */
static void Current_PhysicalToPU(float current, float *current_pu)
{
    // 使用归一化模块的基准电流进行标幺化
    extern NormalizationParams_t g_NormParams; // FIXME
    *current_pu = current / g_NormParams.I_base;
}

/**
 * @brief  ADC注入转换完成回调函数
 * @param  hadc: ADC句柄
 * @retval None
 * @note   此函数在ADC注入通道转换完成时被HAL库调用
 *         TIM1 CH4双边沿触发，每个PWM周期调用2次
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC2)
    {
        // 读取三相电流ADC原始值
        g_CurrentSample.adc_raw[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);  // Ia
        g_CurrentSample.adc_raw[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);  // Ib
        g_CurrentSample.adc_raw[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);  // Ic

        // 转换为物理值
        for (int i = 0; i < CURRENT_SAMPLE_CHANNELS; i++)
        {
            Current_ADCToPhysical(g_CurrentSample.adc_raw[i], &g_CurrentSample.current_abc[i]);
            Current_PhysicalToPU(g_CurrentSample.current_abc[i], &g_CurrentSample.current_pu[i]);
        }

        // 更新采样计数和有效标志
        g_CurrentSample.sample_count++;
        g_CurrentSample.sample_valid = 1;

        // TODO: 在此处调用FOC控制循环或将数据传递给控制任务
        // 例如：FOC_CurrentLoop(&g_CurrentSample);
    }
}