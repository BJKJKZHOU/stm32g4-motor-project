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
#include "tim.h"
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

// ============================================================================
// ADC电流采样触发点动态调整实现
// ============================================================================

// 调试变量 - 用于监控触发点调整
volatile uint32_t g_adc_trigger_point = TRIGGER_HIGH_SIDE;  // 当前ADC触发点
volatile uint32_t g_min_duty = 0;                           // 最小占空比
volatile uint32_t g_max_duty = 0;                           // 最大占空比
volatile uint8_t g_trigger_strategy = 0;                    // 触发策略：0=高侧，1=低侧，2=降级

/**
 * @brief ADC电流采样触发点动态调整函数
 *
 * @param duty_ch1 通道1占空比计数值 (0-ARR_PERIOD)
 * @param duty_ch2 通道2占空比计数值 (0-ARR_PERIOD)
 * @param duty_ch3 通道3占空比计数值 (0-ARR_PERIOD)
 *
 * @note 中心对齐PWM模式下的采样时序：
 *       - 上管导通：CNT < CCR（考虑死区）
 *       - 下管导通：CNT > CCR + 死区 或 CNT < CCR - 死区
 *       - ADC触发后需要约415个计数完成16倍过采样
 *       - 触发点在ARR-300时，可用采样窗口为600个计数（向上+向下）
 *
 * @note 采样策略：
 *       1. 找到三相中最小占空比（下管导通时间最长）
 *       2. 占空比 < 90%：固定在ARR-300触发（高侧采样）
 *       3. 占空比 90-98%：动态调整到低侧采样
 *       4. 占空比 > 98%：降级采样或跳过
 */
void Update_ADC_Trigger_Point(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3)
{
    // 1. 找到三相中的最小和最大占空比
    uint32_t min_duty = duty_ch1;
    uint32_t max_duty = duty_ch1;

    if (duty_ch2 < min_duty) min_duty = duty_ch2;
    if (duty_ch3 < min_duty) min_duty = duty_ch3;

    if (duty_ch2 > max_duty) max_duty = duty_ch2;
    if (duty_ch3 > max_duty) max_duty = duty_ch3;

    // 保存到调试变量
    g_min_duty = min_duty;
    g_max_duty = max_duty;

    uint32_t trigger_point;

    // 2. 根据最小占空比选择触发策略
    if (min_duty < DUTY_THRESHOLD_HIGH) {
        // ========== 策略1：高侧采样（占空比 < 90%） ==========
        // 在ARR附近触发，此时所有相的下管都有足够导通时间
        trigger_point = TRIGGER_HIGH_SIDE;  // ARR - 300
        g_trigger_strategy = 0;

    } else if (min_duty < DUTY_CRITICAL_HIGH) {
        // ========== 策略2：低侧动态采样（占空比 90-98%） ==========
        // 在低侧（接近0）采样，选择最小占空比相的下管导通中点
        // 下管导通区间：[0, min_duty - 死区)
        // 采样需要415个计数，所以触发点要留出足够裕量

        uint32_t low_side_window = min_duty - TIM1_DEADTIME;  // 下管导通窗口大小

        if (low_side_window > (ADC_SAMPLE_TIME + 100)) {
            // 有足够空间，在窗口中间触发
            trigger_point = (low_side_window - ADC_SAMPLE_TIME) / 2;
        } else {
            // 空间不足，尽量靠前触发
            trigger_point = DUTY_THRESHOLD_LOW / 2;  // 约212
        }

        g_trigger_strategy = 1;

    } else {
        // ========== 策略3：极高占空比降级处理（占空比 > 98%） ==========
        // 下管导通时间极短，无法保证采样质量
        // 选项A：保持上一次触发点（惯性采样）
        // 选项B：强制在低侧采样（可能采样不完整）
        // 选项C：跳过本次采样（设置触发点为0，禁用触发）

        // 这里选择选项B：强制低侧采样
        trigger_point = DUTY_THRESHOLD_LOW / 2;  // 约212
        g_trigger_strategy = 2;

        // 可选：如果需要跳过采样，取消下面的注释
        // trigger_point = 0;  // 禁用触发
    }

    // 3. 边界检查：确保触发点在合法范围内
    if (trigger_point > ARR_PERIOD) {
        trigger_point = TRIGGER_HIGH_SIDE;  // 回退到默认值
    }

    // 4. 更新TIM1_CH4比较值（ADC触发点）
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger_point);

    // 5. 保存到调试变量
    g_adc_trigger_point = trigger_point;
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
