/*============================================================================
    File Name     : stm32_hal_stubs.h
    Description   : STM32 HAL函数存根 - 用于单元测试
    Author        : ZHOUHENG
    Date          : 2025-11-10
    ----------------------------------------------------------------------       
    说明：
        这个文件包含了STM32 HAL函数的存根实现，用于在PC环境下运行单元测试
        避免了对实际硬件的依赖
============================================================================*/

#ifndef STM32_HAL_STUBS_H
#define STM32_HAL_STUBS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// 包含STM32 HAL stub头文件（提供基本类型定义）
#include "stm32g4xx_hal.h"

// 防止 ARM DSP 头文件冲突
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

// 在包含 arm_math.h 之前定义我们的类型以避免重定义
// CORDIC 相关类型定义
typedef struct {
    uint32_t Function;      /*!< CORDIC Function */
    uint32_t Scale;         /*!< CORDIC Scaling factor */
    uint32_t InSize;        /*!< CORDIC Input data size in bits */
    uint32_t OutSize;       /*!< CORDIC Output data size in bits */
    uint32_t NbWrite;       /*!< CORDIC Number of expected writes */
    uint32_t NbRead;        /*!< CORDIC Number of expected reads */
    uint32_t Precision;     /*!< CORDIC Number of calculation cycles */
} CORDIC_ConfigTypeDef;

// CORDIC 句柄类型
typedef struct {
    void *Instance;    // CORDIC instance
} CORDIC_HandleTypeDef;

// 外部 CORDIC 句柄声明
extern CORDIC_HandleTypeDef hcordic;

// 包含 ARM DSP 库
#include "arm_math.h"

// HAL 状态枚举 - 避免与 arm_math.h 中的定义冲突
#ifndef HAL_STATUS_TYPEDEF_DEFINED
#define HAL_STATUS_TYPEDEF_DEFINED
typedef enum {
    HAL_OK       = 0x00,
    HAL_ERROR    = 0x01,
    HAL_BUSY     = 0x02,
    HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;
#endif

// CORDIC 函数枚举
#define CORDIC_FUNCTION_COSINE     0x00000000U
#define CORDIC_FUNCTION_SINE       0x00000001U
#define CORDIC_FUNCTION_PHASE      0x00000002U
#define CORDIC_FUNCTION_MODULUS    0x00000003U
#define CORDIC_FUNCTION_ARCTAN     0x00000004U
#define CORDIC_FUNCTION_COSINE_SINE 0x00000005U

// CORDIC 缩放枚举
#define CORDIC_SCALE_0            0x00000000U
#define CORDIC_SCALE_1            0x00000001U
#define CORDIC_SCALE_2            0x00000002U
#define CORDIC_SCALE_3            0x00000003U
#define CORDIC_SCALE_4            0x00000004U
#define CORDIC_SCALE_5            0x00000005U
#define CORDIC_SCALE_6            0x00000006U
#define CORDIC_SCALE_7            0x00000007U

// CORDIC 输入输出大小枚举
#define CORDIC_INSIZE_32BITS      0x00000000U
#define CORDIC_INSIZE_16BITS      0x00000001U
#define CORDIC_INSIZE_8BITS       0x00000002U
#define CORDIC_INSIZE_4BITS       0x00000003U

#define CORDIC_OUTSIZE_32BITS     0x00000000U
#define CORDIC_OUTSIZE_16BITS     0x00000001U
#define CORDIC_OUTSIZE_8BITS      0x00000002U
#define CORDIC_OUTSIZE_4BITS      0x00000003U

// CORDIC 读写次数枚举
#define CORDIC_NBWRITE_1          0x00000000U
#define CORDIC_NBWRITE_2          0x00000001U
#define CORDIC_NBREAD_1           0x00000000U
#define CORDIC_NBREAD_2           0x00000001U

// CORDIC 精度枚举
#define CORDIC_PRECISION_1CYCLE   0x00000000U
#define CORDIC_PRECISION_2CYCLES  0x00000001U
#define CORDIC_PRECISION_3CYCLES  0x00000002U
#define CORDIC_PRECISION_4CYCLES  0x00000003U
#define CORDIC_PRECISION_5CYCLES  0x00000004U
#define CORDIC_PRECISION_6CYCLES  0x00000005U
#define CORDIC_PRECISION_7CYCLES  0x00000006U

// HAL CORDIC 函数声明
HAL_StatusTypeDef HAL_CORDIC_Configure(CORDIC_HandleTypeDef *hcordic, CORDIC_ConfigTypeDef *sConfig);
HAL_StatusTypeDef HAL_CORDIC_Calculate(CORDIC_HandleTypeDef *hcordic, int32_t *pInBuffer, int32_t *pOutBuffer, uint32_t NbCalc, uint32_t Timeout);
HAL_StatusTypeDef HAL_CORDIC_CalculateZO(CORDIC_HandleTypeDef *hcordic, int32_t *pInBuffer, int32_t *pOutBuffer, uint32_t NbCalc, uint32_t Timeout);

// 其他 HAL 函数
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);

// ARR_PERIOD宏定义（通常在tim.h中定义）
#ifndef ARR_PERIOD
#define ARR_PERIOD 5312  // 测试用的默认值
#endif

// DWT（Data Watchpoint and Trace）相关定义
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)

typedef struct {
    uint32_t CTRL;
    uint32_t CYCCNT;
} DWT_Type_Stub;

typedef struct {
    uint32_t DEMCR;
} CoreDebug_Type_Stub;

extern DWT_Type_Stub *DWT;
extern CoreDebug_Type_Stub *CoreDebug;

// TIM（定时器）相关定义
// 注意：TIM_HandleTypeDef已在stm32g4xx_hal.h中定义
extern TIM_HandleTypeDef htim1;

// TIM通道定义
#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U

// TIM HAL宏定义（测试环境中为空操作）
#define __HAL_TIM_SET_COMPARE(htim, Channel, Compare) \
    do { (void)(htim); (void)(Channel); (void)(Compare); } while(0)

#ifdef __cplusplus
}
#endif

#endif /* STM32_HAL_STUBS_H */