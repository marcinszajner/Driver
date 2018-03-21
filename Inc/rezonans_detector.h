/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REZONANS_DETECTOR_H
#define __REZONANS_DETECTOR_H

/* Includes ------------------------------------------------------------------*/
#include "main.h" // TODO move init structure from main.c to separated file
#include "arm_math.h"
#include "arm_const_structs.h"
#include "fxpt_atan2.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint32_t rezonans_detector(void);
void Start_Detector(void);
void Stop_Detector(void);
void HRTIM_Config_rezonans_detector(uint32_t frequency);
/* Exported variables --------------------------------------------------------*/

#endif /* __REZONANS_DETECTOR_H */