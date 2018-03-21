/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rezonans_detector.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
HRTIM_TimeBaseCfgTypeDef timebase_config;
HRTIM_TimerCfgTypeDef timer_config;
HRTIM_OutputCfgTypeDef output_config_TD1;
HRTIM_BurstModeCfgTypeDef burst_mode_config;
HRTIM_CompareCfgTypeDef compare_config;
HRTIM_ADCTriggerCfgTypeDef adc_trigger_config;

ADC_MultiModeTypeDef MultiModeConfig;
ADC_InjectionConfTypeDef InjectionConfig;

uint32_t min_frequency =                  MIN_FRQUENCY;
uint32_t max_frequency =                  MAX_FREQUENCY;
uint32_t difference_frequency_reg_value = (MIN_FRQUENCY - MAX_FREQUENCY);
uint32_t max_burst_control_value =        (((DIFERENCE_FREQUENCY_REG_VALUE)*(PERCENT_OF_BURST_MODE_CONTROL))/1000);
uint32_t max_value_of_feedback_loop =     ((DIFERENCE_FREQUENCY_REG_VALUE) + (MAX_BURST_CONTROL_VALUE));
uint32_t link_adc_tofeedback =            (((MAX_VALUE_OF_FEEDBACK_LOOP)*(1000))/(ADC_12_BIT_RESULUTION));
uint32_t link_to_burst_mode =             ((MAX_BURST_CONTROL_VALUE) / (BURST_MODE_PERIOD));

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void HRTIM_Config_HalfMode(uint32_t frequency);
void ADC_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint32_t active_led = 0;
  uint32_t resonant_frequency = 0;

  //Hardware abstraction layer initialization
  HAL_Init();

  /* Configure the system clock to have a system clock = 72 Mhz */
  SystemClock_Config();
  
#ifdef STM32F334_EVAL_BOARD
  /* Initialize STM32F3348-DISCO LEDs */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  /* Initialize User_Button on STM32F3348-DISCO */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO); 
#endif

  
  resonant_frequency = rezonans_detector();
  
  
  //Initialize HRTIM and related inputs
  HRTIM_Config_HalfMode(resonant_frequency);
  
  //Initialize GPIO's HRTIM outputs
  //GPIO_HRTIM_outputs_Config();
  
  /* Initialize ADC to be triggered by the HRTIMER */
  //ADC_Config();
  
  //Init FIR array with max value for soft start
  Init_FIR();
  
  //Start driver
  Start_Driver();
  
  /* Infinite loop */
  while (1)
  {
#ifdef STM32F334_EVAL_BOARD
    if((BSP_PB_GetState(BUTTON_USER) == SET))
    {
      active_led++;
      if(active_led >= 4)
      {
        active_led = 0;
      }
    }
    switch(active_led)
    {
      case 0:
        BSP_LED_Off(LED4);
        BSP_LED_Off(LED5);
        BSP_LED_Off(LED6);
        BSP_LED_Toggle(LED3);
        break;
      case 1:
        BSP_LED_Off(LED3);
        BSP_LED_Off(LED5);
        BSP_LED_Off(LED6);
        BSP_LED_Toggle(LED4);
        break;
      case 2:
        BSP_LED_Off(LED3);
        BSP_LED_Off(LED4);
        BSP_LED_Off(LED5);
        BSP_LED_Toggle(LED6);
        break;
      case 3:
        BSP_LED_Off(LED3);
        BSP_LED_Off(LED4);
        BSP_LED_Off(LED6);
        BSP_LED_Toggle(LED5);
        break;
      default:
        BSP_LED_Toggle(LED3);
        BSP_LED_Toggle(LED4);
        BSP_LED_Toggle(LED5);
        BSP_LED_Toggle(LED6);
    }
#endif
    HAL_Delay(400);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  Configure GPIO outputs for the HRTIM
* @param  None
* @retval None
*/
void GPIO_HRTIM_outputs_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIOA clock for timer A outputs */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure HRTIM output: TA1 (PA8) and TA2 (PA9)*/
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;;  
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;  
  GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable GPIOB clock for timer D outputs */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure HRTIM output: TD1 (PB14) and TD2 (PB15)*/
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15; 
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;               //At begine output signal is disabled!
  GPIO_InitStruct.Pull = GPIO_NOPULL;;  
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;  
  GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
* @brief  Configure HRTIM
* @param  None
* @retval None
*/
static void HRTIM_Config_HalfMode(uint32_t frequency)
{

  /* ----------------------------------------------- */
  /* HRTIM Global initialization: Clock and DLL init */
  /* ----------------------------------------------- */
  /* Initialize the HRTIM structure (minimal configuration) */
  //hhrtim.Instance = HRTIM1;
  //hhrtim.Init.HRTIMInterruptResquests = HRTIM_IER_BMPER; /*!<  Burst mode period interrupt enable */
  //hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;

  /* Initialize HRTIM */
  //HAL_HRTIM_Init(&hhrtim);

  /* HRTIM DLL calibration: periodic calibration, set period to 14µs */
  //HAL_HRTIM_DLLCalibrationStart(&hhrtim, HRTIM_CALIBRATIONRATE_14);
  /* Wait calibration completion*/
  //if (HAL_HRTIM_PollForDLLCalibration(&hhrtim, 100) != HAL_OK)
  //{
  //  Error_Handler(); /* if DLL or clock is not correctly set */
  //}
  
  burst_mode_config.ClockSource = HRTIM_BURSTMODECLOCKSOURCE_TIMER_D;
  burst_mode_config.IdleDuration = 0;                                   //start value of idle durration.
  burst_mode_config.Mode = HRTIM_BURSTMODE_CONTINOUS;
  burst_mode_config.Period = BURST_MODE_PERIOD;                         //Define how many periods belong to burst mode
  burst_mode_config.PreloadEnable = HRIM_BURSTMODEPRELOAD_ENABLED;
  burst_mode_config.Prescaler = HRTIM_BURSTMODEPRESCALER_DIV1;
  burst_mode_config.Trigger = HRTIM_BURSTMODETRIGGER_TIMERD_CMP1;
  HAL_HRTIM_BurstModeConfig(&hhrtim, &burst_mode_config);
  
  /* ------------------------------------------------------------ */
  /* TIMERD initialization: set Hal Mode frequency and continuous mode */
  /* ------------------------------------------------------------ */
  timebase_config.Period = frequency;                                           //start frequency value
  timebase_config.RepetitionCounter = 0;                                        //Event rate divider
  timebase_config.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  timebase_config.Mode = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &timebase_config);
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, &timebase_config);// half bridge control

  
  /* ---------------------------------------------------------- */
  /* Master timer initialization                                */
  /* Update on repetition, interrupt issued on repetition event */
  /* Master timer is used for primary side switches             */
  /* ---------------------------------------------------------- */
  timer_config.DMARequests = HRTIM_TIM_DMA_NONE;
  timer_config.DMASrcAddress = 0x0;
  timer_config.DMADstAddress = 0x0;
  timer_config.DMASize = 0x0;
  timer_config.HalfModeEnable = HRTIM_HALFMODE_ENABLED;
  timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  timer_config.InterruptRequests = HRTIM_MASTER_IT_NONE;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &timer_config);
  
  /* --------------------------------------------------------------------- */
  /* TIMERD global configuration: all values to default                    */
  /* --------------------------------------------------------------------- */
  
  timer_config.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  timer_config.InterruptRequests = HRTIM_TIM_IT_REP;
  timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  timer_config.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  timer_config.UpdateTrigger= HRTIM_TIMUPDATETRIGGER_MASTER;
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER + HRTIM_TIMRESETTRIGGER_MASTER_CMP1;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_D,&timer_config);
  
  
  /* --------------------------------------------------------- */
  /* TD1 waveform description TD1 set on period, reset on CMP1 */
  /* Compare 1 is automatically computed (Half mode)           */
  /* --------------------------------------------------------- */
  output_config_TD1.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_config_TD1.SetSource = HRTIM_OUTPUTSET_MASTERCMP1;
  output_config_TD1.ResetSource  = HRTIM_OUTPUTSET_MASTERPER;
  output_config_TD1.IdleMode = HRTIM_OUTPUTIDLEMODE_IDLE;
  output_config_TD1.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_config_TD1.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  output_config_TD1.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_config_TD1.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_DELAYED;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim,
                                 HRTIM_TIMERINDEX_TIMER_D,
                                 HRTIM_OUTPUT_TD1,              //HRTIM1_CHD1 PB14
                                 &output_config_TD1);

  /* Set compare registers for duty cycle on TD1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (85*frequency)/100;     // 85% duty cycle
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_D,
                                  HRTIM_COMPAREUNIT_2,
                                  &compare_config);
  
/*              NOT USED                */
//    /* Set compare registers for duty cycle on TD1 */
//  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
//  compare_config.AutoDelayedTimeout = 0;
//  compare_config.CompareValue = MIN_FRQUENCY/2;     // 50% duty cycle
//  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
//                                  HRTIM_TIMERINDEX_TIMER_D,
//                                  HRTIM_COMPAREUNIT_3,
//                                  &compare_config);
//  
//    /* Set compare registers for duty cycle on TD1 */
//  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
//  compare_config.AutoDelayedTimeout = 0;
//  compare_config.CompareValue = MIN_FRQUENCY/4;     // 25% duty cycle
//  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
//                                  HRTIM_TIMERINDEX_TIMER_D,
//                                  HRTIM_COMPAREUNIT_4,
//                                  &compare_config);
  
  adc_trigger_config.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERD_CMP2;     //Set ADC event source TIMER D CMP2
                             //+ HRTIM_ADCTRIGGEREVENT24_TIMERD_CMP3
                             //+ HRTIM_ADCTRIGGEREVENT24_TIMERD_CMP4
                             //+HRTIM_ADCTRIGGEREVENT24_TIMERD_RESET;
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_D;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_2, &adc_trigger_config);
}

/**
  * @brief  Configure ADC1 and ADC2 for being used with HRTIM
  * For each ADC, 4 injected channels are used
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
  AdcHandle.Instance = ADC2;

  /* ADC2 is working independently */
  MultiModeConfig.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  MultiModeConfig.Mode = ADC_MODE_INDEPENDENT;
  MultiModeConfig.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  HAL_ADCEx_MultiModeConfigChannel(&AdcHandle, &MultiModeConfig);

  /* ADC1 global initialization */
  /* 12-bit right-aligned format, discontinuous scan mode, running from PLL */
  AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode = ENABLE;
  AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait = DISABLE;
  AdcHandle.Init.ContinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfConversion = 1;                           // set number of conversation if number of active RANK's are changed
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion = 1;
  AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&AdcHandle);

  /* Discontinuous injected mode: 1st injected conversion for Vin on Ch2 */
  InjectionConfig.AutoInjectedConv = DISABLE;
  InjectionConfig.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG2;
  InjectionConfig.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  InjectionConfig.InjectedChannel = ADC_CHANNEL_2;              // ADC2_CH2 PA5
  InjectionConfig.InjectedDiscontinuousConvMode = ENABLE;
  InjectionConfig.InjectedNbrOfConversion = 1;
  InjectionConfig.InjectedOffset = 0;
  InjectionConfig.InjectedOffsetNumber = ADC_OFFSET_NONE;
  InjectionConfig.InjectedRank = ADC_INJECTED_RANK_1;
  InjectionConfig.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  InjectionConfig.InjectedSingleDiff = ADC_SINGLE_ENDED;
  InjectionConfig.QueueInjectedContext = DISABLE;
  HAL_ADCEx_InjectedConfigChannel(&AdcHandle, &InjectionConfig);
  
/*              NOT USED                */
  /* Configure the 2nd injected conversion on Ch2 */
  InjectionConfig.InjectedChannel = ADC_CHANNEL_2;
  InjectionConfig.InjectedRank = ADC_INJECTED_RANK_2;
  InjectionConfig.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADCEx_InjectedConfigChannel(&AdcHandle, &InjectionConfig);
  
  /* Configure the 3rd injected conversion on Ch2 */
  InjectionConfig.InjectedChannel = ADC_CHANNEL_2;
  InjectionConfig.InjectedRank = ADC_INJECTED_RANK_3;
  InjectionConfig.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADCEx_InjectedConfigChannel(&AdcHandle, &InjectionConfig);

  /* Configure the 4th injected conversion on Ch2 */
  InjectionConfig.InjectedChannel = ADC_CHANNEL_2;
  InjectionConfig.InjectedRank = ADC_INJECTED_RANK_4;
  InjectionConfig.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADCEx_InjectedConfigChannel(&AdcHandle, &InjectionConfig);

  /* Configure and enable ADC1_2_IRQHandler interrupt channel in NVIC */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  /* Run the ADC calibration in single-ended mode */
  HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED);
  
  /* Start ADC1 Injected Conversions */
  HAL_ADCEx_InjectedStart_IT(&AdcHandle);
}

/**
  * @brief  This function start driver.
  * @param  None
  * @retval None
  */
void Start_Driver(void)
{  
  
  hhrtim.Instance->sMasterRegs.MCNTR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CNTxR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CNTxR = 0x0000;
  
  /* ---------------*/
  /* HRTIM start-up */
  /* ---------------*/
  /* Enable HRTIM's outputs TD1 */
  /* Note: it is necessary to enable also GPIOs to have outputs functional */
  /* This must be done after HRTIM initialization */
  //HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TD1);
  
    //HAL_HRTIM_BurstModeCtl(&hhrtim, HRTIM_BURSTMODECTL_ENABLED);
  /* Start HRTIM's TIMER D*/
  //HAL_HRTIM_WaveformCounterStart_IT(&hhrtim, HRTIM_TIMERID_TIMER_D);
  /* Start HRTIM's Master timer, TIMER A and B */
  hhrtim.Instance->sCommonRegs.CR2 = (HRTIM_TIMERRESET_MASTER|HRTIM_TIMERRESET_TIMER_D | HRTIM_TIMERUPDATE_MASTER|HRTIM_TIMERUPDATE_D);
  //HAL_HRTIM_WaveformCounterStart_IT(&hhrtim, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);
  hhrtim.Instance->sMasterRegs.MCR |= (HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_D);
  
  //hhrtim.Instance->sMasterRegs.MDIER |= HRTIM_TIMERID_TIMER_D;
  
  //This delay is nedded! It is aganist transient state on HRTIM driver
  //HAL_Delay(1);
  //HAL_HRTIM_WaveformCounterStart_IT(&hhrtim, HRTIM_TIMERID_TIMER_A);
  //Active output
  GPIOB->AFR[1] |= 0xDD000000;  //Set GPIOB alternative function PIN 14 and 15
  GPIOB->MODER  |= 0xA0000000;  //SET GPIOB port mode PIN 14 and 15 to Alternative function mode
}

void Stop_Driver(void)
{
  //HAL_HRTIM_WaveformOutputStop(&hhrtim, HRTIM_OUTPUT_TD1);
  //HAL_ADCEx_InjectedStop_IT(&AdcHandle);
  //HAL_HRTIM_BurstModeCtl(&hhrtim, HRTIM_BURSTMODECTL_DISABLED);
  
  //HAL_HRTIM_WaveformCounterStop_IT(&hhrtim, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);
  hhrtim.Instance->sMasterRegs.MCR &= ~(HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);
  
  //HAL_HRTIM_SoftwareReset(&hhrtim, HRTIM_TIMERRESET_TIMER_C);
  //HAL_HRTIM_SoftwareReset(&hhrtim, HRTIM_TIMERRESET_MASTER);
  //HAL_HRTIM_SoftwareReset(&hhrtim, HRTIM_TIMERRESET_TIMER_D);
  
  hhrtim.Instance->sCommonRegs.CR2 = (HRTIM_TIMERRESET_MASTER|HRTIM_TIMERRESET_TIMER_C|HRTIM_TIMERRESET_TIMER_D | HRTIM_TIMERUPDATE_MASTER|HRTIM_TIMERUPDATE_C|HRTIM_TIMERUPDATE_D);
  
  HAL_HRTIM_WaveformSetOutputLevel(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, HRTIM_OUTPUTLEVEL_INACTIVE);
  
  hhrtim.Instance->sMasterRegs.MCNTR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CNTxR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CNTxR = 0x0000;
  //GPIOB->AFR[1] |= 0x00000000;  //Set GPIOB alternative function PIN 14 and 15
  //GPIOB->MODER  |= 0x00000000;  //SET GPIOB port mode PIN 14 and 15 to Alternative function mode
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}