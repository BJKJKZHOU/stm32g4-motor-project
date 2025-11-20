/*============================================================================
    File Name     : Current.h
    Description   : 电流采样相关
    Author        : ZHOUHENG
    Date          : 2025-11-14
    ----------------------------------------------------------------------
    使用ADC2采样电流，转换为标幺值，ADC基础设置为此模块的下层。

    【采样配置】
   //  1、采样方式：双采样单更新模式
   //     - TIM1 CH4双边沿触发（上升沿+下降沿）
   //     - 每个PWM周期采样2次（中心对齐模式的上升和下降阶段）
   //     - 重复计数器=0，每个PWM周期产生1次更新事件


    1、单采样单更新采样方式
       - ARR附近触发，单次，过采样
       RCR = 1


    2、采样时间窗口
       - PWM周期：50μs (20kHz)
       - TIM1 CH4脉宽：250个时钟 ≈ 1.47μs
       - ADC转换时间：(6.5+12.5) = 19个ADC时钟 ≈ 0.45μs
       - 死区时间：85个时钟 ≈ 0.5μs
       - 安全窗口：250 - 85 - 19 ≈ 146个时钟 ≈ 0.86μs（充足）

    3、硬件配置
       - ADC2通道1 (PA0)：Ia相电流
       - ADC2通道2 (PA1)：Ib相电流
       - ADC2通道3 (PA6)：Ic相电流
       - ADC2通道4 (PA7)：母线电压（常规通道）
       - 采样电阻和运放参数：待定义

    4、触发时序
       - TIM1中心对齐模式，ARR=4250
       - CH4比较值=250，在计数上升和下降时各触发一次ADC
       - 采样点位于PWM有效矢量期间，确保电流路径稳定

*=============================================================================
*/

#ifndef __CURRENT_H
#define __CURRENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 电流采样相关宏定义 --------------------------------------------------------*/
#define CURRENT_SAMPLE_CHANNELS     3       // 三相电流采样通道数
#define ADC_RESOLUTION_12BIT        4096    // 12位ADC分辨率
#define ADC_VREF                    3.3f    // TODO ADC参考电压 (V)

/* 硬件参数（根据实际硬件调整）*/
#define CURRENT_SENSE_RESISTOR      0.001f   // TODO 采样电阻 (Ω) 丝印 R001​ 1mΩ
#define CURRENT_OPAMP_GAIN          100.0f   // TODO 运放增益 芯片 INA4181A3IPWR 
#define CURRENT_OFFSET_ADC          2048    // 电流零点对应的ADC值（12位中点）

/* 电流采样数据结构 ----------------------------------------------------------*/
typedef struct {
    uint16_t adc_raw[CURRENT_SAMPLE_CHANNELS];  // ADC原始值：Ia, Ib, Ic
    float current_abc[CURRENT_SAMPLE_CHANNELS]; // 三相电流物理值 (A)
    float current_pu[CURRENT_SAMPLE_CHANNELS];  // 三相电流标幺值
    uint32_t sample_count;                      // 采样计数器
    uint8_t sample_valid;                       // 采样数据有效标志
} CurrentSample_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern CurrentSample_t g_CurrentSample;

/* 函数声明 ------------------------------------------------------------------*/
void Current_Init(void);
void Current_GetABC(float *ia, float *ib, float *ic);
void Current_GetPU(float *ia_pu, float *ib_pu, float *ic_pu);
uint16_t* Current_GetRawADC(void);


// ADC电流采样触发点动态调整函数
// 输入:
// - duty_ch1, duty_ch2, duty_ch3: 三相PWM占空比计数值 (0-ARR_PERIOD)
// 功能：
// 1. 分析三相占空比，找到最小占空比（下管导通时间最长）
// 2. 根据占空比范围选择最佳ADC触发时机
// 3. 动态调整TIM1_CH4比较值以优化电流采样时序
// 4. 确保ADC采样在下管导通期间完成
// 策略：
// - 占空比 < 90%: 在ARR附近触发（高侧采样）
// - 占空比 90-98%: 在低侧动态调整触发点
// - 占空比 > 98%: 降级采样或跳过

// ADC触发点调试变量
extern volatile uint32_t g_adc_trigger_point;        // 当前ADC触发点
extern volatile uint32_t g_min_duty;                 // 最小占空比
extern volatile uint32_t g_max_duty;                 // 最大占空比
extern volatile uint8_t g_trigger_strategy;          // 触发策略：0=高侧，1=低侧，2=降级

void Update_ADC_Trigger_Point(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3);

#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_H */
