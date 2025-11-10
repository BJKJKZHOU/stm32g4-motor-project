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

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4XX_HAL_H */