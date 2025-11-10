/*============================================================================
    File Name     : stm32_hal_stubs.c
    Description   : STM32 HAL函数存根实现 - 用于单元测试
    Author        : ZHOUHENG
    Date          : 2025-11-10
    ----------------------------------------------------------------------       
    说明：
        这个文件包含了STM32 HAL函数的存根实现，用于在PC环境下运行单元测试
        避免了对实际硬件的依赖
=============================================================================*/

#include "stm32_hal_stubs.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// ARM DSP函数存根实现
void arm_sin_cos_f32(float theta, float *sin_val, float *cos_val)
{
    if (sin_val != NULL) {
        *sin_val = sinf(theta);
    }
    if (cos_val != NULL) {
        *cos_val = cosf(theta);
    }
}

void arm_cfft_f32(const arm_cfft_instance_f32 *S, float *p1, uint8_t ifftFlag, uint8_t bitReverseFlag)
{
    // 空实现，测试中不使用FFT功能
    (void)S;
    (void)p1;
    (void)ifftFlag;
    (void)bitReverseFlag;
}

// CORDIC句柄实例
CORDIC_HandleTypeDef hcordic;

// CORDIC HAL函数存根实现
HAL_StatusTypeDef HAL_CORDIC_Configure(CORDIC_HandleTypeDef *hcordic, CORDIC_ConfigTypeDef *sConfig)
{
    // 存根实现，总是返回成功
    (void)hcordic;
    (void)sConfig;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CORDIC_Calculate(CORDIC_HandleTypeDef *hcordic, int32_t *pIn, int32_t *pOut, uint32_t NbCalc, uint32_t Timeout)
{
    // 存根实现，使用软件计算代替硬件CORDIC
    (void)hcordic;
    (void)NbCalc;
    (void)Timeout;
    
    if (pIn == NULL || pOut == NULL) {
        return HAL_ERROR;
    }
    
    // 将Q1.31格式的角度转换为浮点数
    float angle_rad = (float)pIn[0] / 2147483647.0f * 3.141592653589793f;
    
    // 使用数学库计算正弦和余弦
    float sin_val = sinf(angle_rad);
    float cos_val = cosf(angle_rad);
    
    // 转换回Q1.31格式 - 注意：Q1.31范围是[-1, 1)映射到[-2^31, 2^31-1]
    // 对于1.0，应该映射到0x7FFFFFFF (2147483647)而不是2147483648
    if (cos_val >= 1.0f) {
        pOut[0] = 2147483647;  // INT32_MAX
    } else if (cos_val <= -1.0f) {
        pOut[0] = -2147483648; // INT32_MIN
    } else {
        pOut[0] = (int32_t)(cos_val * 2147483647.0f);
    }
    
    if (sin_val >= 1.0f) {
        pOut[1] = 2147483647;  // INT32_MAX
    } else if (sin_val <= -1.0f) {
        pOut[1] = -2147483648; // INT32_MIN
    } else {
        pOut[1] = (int32_t)(sin_val * 2147483647.0f);
    }
    
    
    return HAL_OK;
}