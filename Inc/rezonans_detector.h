/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REZONANS_DETECTOR_H
#define __REZONANS_DETECTOR_H

/* Includes ------------------------------------------------------------------*/
#include "main.h" // TODO move init structure from main.c to separated file
#include "arm_math.h"
#include "arm_const_structs.h"
#include "fxpt_atan2.h"

#define start_timer()   *((volatile uint32_t*)0xE0001000) = 0x40000001  // Enable CYCCNT register
#define stop_timer()    *((volatile uint32_t*)0xE0001000) = 0x40000000  // Disable CYCCNT register
#define get_timer()     *((volatile uint32_t*)0xE0001004)               // Get value from CYCCNT register

#define M_PI 3.14159265358979323846f

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