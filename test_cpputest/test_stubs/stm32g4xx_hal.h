/* Minimal STM32G4xx HAL stub header for testing */
#ifndef __STM32G4XX_HAL_H
#define __STM32G4XX_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Basic STM32 types - only declare if not already defined */
#ifndef __IO
#define __IO volatile
#endif

#ifndef __I
#define __I volatile const
#endif

#ifndef __O
#define __O volatile
#endif

/* Minimal peripheral handle definitions */
typedef struct {
    void *Instance;
} UART_HandleTypeDef;

typedef struct {
    void *Instance;
} TIM_HandleTypeDef;

typedef struct {
    void *Instance;
} ADC_HandleTypeDef;

typedef struct {
    void *Instance;
} DMA_HandleTypeDef;

#ifndef HAL_STATUS_TYPEDEF_DEFINED
#define HAL_STATUS_TYPEDEF_DEFINED
typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4XX_HAL_H */
