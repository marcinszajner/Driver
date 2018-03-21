/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3xx_IT_H
#define __STM32F3xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
   
#include "control_defines.h"
#include "stm32f3xx_hal.h"
#include "stm32f3348_discovery.h"
#include "arm_math.h"
#include "rezonans_detector.h"
   
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern uint32_t Detection_in_progress;  
extern uint32_t measurement_done;
extern int32_t counter;
extern uint32_t adc_sample[64];
extern void Stop_Detector(void);
   
static int32_t shiftBuffer[SAMPLEFILTER_TAP_NUM];
static void Vout_Check(void);
   
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void HRTIM1_TIMD_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void Init_FIR(void);
void Stop_Driver(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F3xx_IT_H */
