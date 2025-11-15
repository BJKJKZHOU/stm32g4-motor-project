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
#include "motor_params.h"
#include "adc.h"  // 双ADC模式需要访问hadc1和hadc2
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
    // 使用归一化模块进行标幺化转换
    // 获取当前激活的电机ID
    uint8_t active_motor_id = MotorParams_GetActiveMotor();
    
    // 检查是否有激活的电机
    if (active_motor_id >= motors_number) {
        *current_pu = 0.0f;  // 没有激活电机，返回0
        return;
    }

    *current_pu = Normalization_ToPerUnit(active_motor_id, NORMALIZE_CURRENT, current);
}

/**
 * @brief  ADC注入转换完成回调函数（双ADC同步模式）
 * @param  hadc: ADC句柄
 * @retval None
 * @note   在双ADC同步模式下，只有Master（ADC1）会触发中断
 *         ADC1采样：PA0(Ia), PA1(Ib)
 *         ADC2采样：PA6(Ic), PA7(Vbus)
 *         TIM1 CH4双边沿触发，每个PWM周期调用2次
 */

CCMRAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 在双ADC同步模式下，只有Master（ADC1）会触发中断
    if (hadc->Instance == ADC1)
    {
        // 读取ADC1的数据（Ia和Ib）
        g_CurrentSample.adc_raw[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);  // Ia (PA0)
        g_CurrentSample.adc_raw[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);  // Ib (PA1)

        // 读取ADC2的数据（Ic和Vbus）
        g_CurrentSample.adc_raw[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);  // Ic (PA6)
        // uint16_t vbus_adc = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);  // Vbus (PA7)，如果需要

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
