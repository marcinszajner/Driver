/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3348_discovery.h"
#include "control_defines.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "fxpt_atan2.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
extern void Init_FIR(void);
void Start_Driver(void);
void Stop_Driver(void);
void GPIO_HRTIM_outputs_Config(void);
void ADC_Config(void);
/* Exported variables --------------------------------------------------------*/
extern HRTIM_HandleTypeDef hhrtim;
extern ADC_HandleTypeDef AdcHandle;
extern HRTIM_TimeBaseCfgTypeDef timebase_config;
extern HRTIM_TimerCfgTypeDef timer_config;
extern HRTIM_OutputCfgTypeDef output_config_TD1;
extern HRTIM_BurstModeCfgTypeDef burst_mode_config;
extern HRTIM_CompareCfgTypeDef compare_config;
extern HRTIM_ADCTriggerCfgTypeDef adc_trigger_config;
extern uint32_t measurement_done;
extern uint32_t adc_sample[64];
extern uint32_t it1, it2;      // start and stop flag

extern ADC_MultiModeTypeDef MultiModeConfig;
extern ADC_InjectionConfTypeDef InjectionConfig;

#endif /* __MAIN_H */
