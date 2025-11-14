/*============================================================================
    File Name     : Current.h
    Description   : 电流采样相关
    Author        : ZHOUHENG
    Date          : 2025-11-14
    ----------------------------------------------------------------------
    使用ADC2采样电流，转换为标幺值，ADC基础设置为此模块的下层。

    【采样配置】
    1、采样方式：双采样单更新模式
       - TIM1 CH4双边沿触发（上升沿+下降沿）
       - 每个PWM周期采样2次（中心对齐模式的上升和下降阶段）
       - 重复计数器=0，每个PWM周期产生1次更新事件

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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"

/* 电流采样相关宏定义 --------------------------------------------------------*/
#define CURRENT_SAMPLE_CHANNELS     3       // 三相电流采样通道数
#define ADC_RESOLUTION_12BIT        4096    // 12位ADC分辨率
#define ADC_VREF                    3.3f    // TODO ADC参考电压 (V)

/* 硬件参数（根据实际硬件调整）*/
#define CURRENT_SENSE_RESISTOR      0.05f   // TODO 采样电阻 (Ω) -  待确认
#define CURRENT_OPAMP_GAIN          10.0f   // TODO 运放增益 - 待确认
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

#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_H */
