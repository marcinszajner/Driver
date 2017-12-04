/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3348_discovery.h"
#include "control_defines.h"
//#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
extern void Init_FIR(void);
/* Exported variables --------------------------------------------------------*/
extern HRTIM_HandleTypeDef hhrtim;
extern ADC_HandleTypeDef AdcHandle;

#endif /* __MAIN_H */
