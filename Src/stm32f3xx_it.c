/**
  ******************************************************************************
  * @file    Templates/Src/stm32f3xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_it.h"  

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle;
HRTIM_HandleTypeDef hhrtim;

//uint32_t firStateF32[SAMPLEFILTER_TAP_NUM];
int32_t outputF32 = 0;
//multiple by 100000

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 200000 Hz

fixed point precision: 20 bits

* 0 Hz - 8000 Hz
  gain = 1
  desired ripple = 0.01 dB
  actual ripple = n/a

* 32000 Hz - 100000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/
static int filter_taps[SAMPLEFILTER_TAP_NUM] = {
  3537,
  2866,
  173,
  -5808,
  -12607,
  -15165,
  -7834,
  12479,
  43676,
  78095,
  105039,
  115240,
  105039,
  78095,
  43676,
  12479,
  -7834,
  -15165,
  -12607,
  -5808,
  173,
  2866,
  3537
};

volatile uint32_t VinConversionR1, VinConversionR2, VinConversionR3, VinConversionR4,VoutConversion;
uint32_t CurrentDutyA;
uint32_t CurrentDutyB;
uint32_t CurrentDutyB_Mixed;
uint32_t CTMin;
uint32_t CTMax;
uint32_t CTRange;
uint32_t VoutT, VoutRange, VoutA, VoutB;
uint32_t NewPeriodValue;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
extern void SetHRTIM_BuckMode(void);
extern void SetHRTIM_BoostMode(void);
extern void SetHRTIM_MixedMode(void);
inline int32_t PID(void);
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F3xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) , for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f3xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
* @brief  This function handles ADC1_2 interrupt request.
* @param  None
* @retval None
*/
void ADC1_2_IRQHandler(void)
{
#ifdef PROFILLING
    GPIOB->ODR ^= GPIO_PIN_7; //toggle LED6
#endif
    /* Clear ADC interrupt flag */
  __HAL_ADC_CLEAR_FLAG(&AdcHandle, ADC_FLAG_JEOS);
  if(Detection_in_progress)
  {
    if(measurement_done == 1)
    {
      adc_sample[63] = VIN(AdcHandle.Instance->JDR1);
    }
    else
    {
      adc_sample[counter] = VIN(AdcHandle.Instance->JDR1);
    }
    //}
    counter++;
    if(counter >= 63 &&  measurement_done == 0)
    {
      Stop_Detector();
      counter = 0;
      measurement_done = 1;
      __HAL_ADC_CLEAR_FLAG(&AdcHandle, ADC_FLAG_JEOS);
    }
  }
#ifdef PROFILLING
  GPIOB->ODR ^= GPIO_PIN_7;  //toggle LED6
#endif
}

void HRTIM1_TIMD_IRQHandler(void)
{
  
  __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_IT_REP);
   if(0)
  { 

#ifdef PROFILLING
  GPIOB->ODR ^= GPIO_PIN_7;  //toggle LED6
#endif
  static uint32_t CurrentPeriod;
  int32_t acc = 0;
  
  //read adc value
  VoutConversion = AdcHandle.Instance->JDR1;  
  
  // FIR filter implementation
  shiftBuffer[0] = shiftBuffer[1];
  acc += shiftBuffer[0] * filter_taps[0];
  shiftBuffer[1] = shiftBuffer[2];
  acc += shiftBuffer[1] * filter_taps[1];
  shiftBuffer[2] = shiftBuffer[3];
  acc += shiftBuffer[2] * filter_taps[2];
  shiftBuffer[3] = shiftBuffer[4];
  acc += shiftBuffer[3] * filter_taps[3];
  shiftBuffer[4] = shiftBuffer[5];
  acc += shiftBuffer[4] * filter_taps[4];
  shiftBuffer[5] = shiftBuffer[6];
  acc += shiftBuffer[5] * filter_taps[5];
  shiftBuffer[6] = shiftBuffer[7];
  acc += shiftBuffer[6] * filter_taps[6];
  shiftBuffer[7] = shiftBuffer[8];
  acc += shiftBuffer[7] * filter_taps[7];
  shiftBuffer[8] = shiftBuffer[9];
  acc += shiftBuffer[8] * filter_taps[8];
  shiftBuffer[9] = shiftBuffer[10];
  acc += shiftBuffer[9] * filter_taps[9];
  shiftBuffer[10] = shiftBuffer[11];
  acc += shiftBuffer[10] * filter_taps[10];
  shiftBuffer[11] = shiftBuffer[12];
  acc += shiftBuffer[11] * filter_taps[11];
  shiftBuffer[12] = shiftBuffer[13];
  acc += shiftBuffer[12] * filter_taps[12];
  shiftBuffer[13] = shiftBuffer[14];
  acc += shiftBuffer[13] * filter_taps[13];
  shiftBuffer[14] = shiftBuffer[15];
  acc += shiftBuffer[14] * filter_taps[14];
  shiftBuffer[15] = shiftBuffer[16];
  acc += shiftBuffer[15] * filter_taps[15];
  shiftBuffer[16] = shiftBuffer[17];
  acc += shiftBuffer[16] * filter_taps[16];
  shiftBuffer[17] = shiftBuffer[18];
  acc += shiftBuffer[17] * filter_taps[17];
  shiftBuffer[18] = shiftBuffer[19];
  acc += shiftBuffer[18] * filter_taps[18];
  shiftBuffer[19] = shiftBuffer[20];
  acc += shiftBuffer[19] * filter_taps[19];
  shiftBuffer[20] = shiftBuffer[21];
  acc += shiftBuffer[20] * filter_taps[20];
  shiftBuffer[21] = shiftBuffer[22];
  acc += shiftBuffer[21] * filter_taps[21];

  shiftBuffer[22] = VoutConversion;
  acc += shiftBuffer[22] * filter_taps[22];
  
  VoutConversion = (acc>>20);
  
  NewPeriodValue = PID();
  CurrentPeriod = (MIN_FRQUENCY) - (NewPeriodValue);

  if(CurrentPeriod <= MAX_FREQUENCY)
  {
    //Control frequency is over max value, burst mode control
    if(!(HRTIM_BURSTMODECTL_ENABLED & hhrtim.Instance->sCommonRegs.BMCR))
    {
      HAL_HRTIM_BurstModeCtl(&hhrtim, HRTIM_BURSTMODECTL_ENABLED);
    }
    /* Set new LLC frequency */
    __HAL_HRTIM_SETPERIOD(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, MAX_FREQUENCY);
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_2, (85 * MAX_FREQUENCY)/100);    

    uint32_t linkToBurstMode = LINK_TO_BURST_MODE;
    int32_t temp = (((MAX_FREQUENCY) - (CurrentPeriod)) / (linkToBurstMode))-1;

    hhrtim.Instance->sCommonRegs.BMCMPR = temp+1;
  }
  else
  {
    //frequency control
    if(HRTIM_BURSTMODECTL_ENABLED & hhrtim.Instance->sCommonRegs.BMCR)
    {
      HAL_HRTIM_BurstModeCtl(&hhrtim, HRTIM_BURSTMODECTL_DISABLED);
    }

    /* Set new LLC frequency */
    __HAL_HRTIM_SETPERIOD(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, CurrentPeriod);
    
    /* Adjust Turn-on and turn-off time for SR1 */
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_2, (85 * CurrentPeriod)/100);
  }
  
#ifdef PROFILLING
  GPIOB->ODR ^= GPIO_PIN_7;  //toggle LED6
#endif
  }
}

void Init_FIR(void)
{
  for(uint32_t i =0; i < SAMPLEFILTER_TAP_NUM; ++i)
  {
    shiftBuffer[i] = 0xFFF; // max 12bit value
  }
}

inline int32_t PID(void)
{
  
  /* Here should be PID. Now it is only proportial part */
  
  VoutT = (VOUT_TARGET * VIN_RESISTOR_RATIO) / 10000; /* introduced resistor bridge ratio ~20% */
  VoutT = (VoutT * 0xFFF) / REAL_3V3; /* converted to 12-bit ADC full range with practical 3.3V of application */
  
  uint32_t LinkAdcToFeedback = LINK_ADC_TO_FEEDBACK;
 
  /* Compute PI for Buck Mode */
  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
  int32_t pid_out;
  //int32_t error;

  pid_out = VoutConversion - VoutT;
  
  if(0 > pid_out)
  {
    pid_out = 0;
  }
  
  pid_out = (pid_out * LinkAdcToFeedback)/1000;
  
  if(MAX_VALUE_OF_FEEDBACK_LOOP < pid_out)
  {
    pid_out = MAX_VALUE_OF_FEEDBACK_LOOP;
  }
  
  return  pid_out*KP;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
