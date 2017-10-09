/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
#define HRTIM_INPUT_CLOCK       ((uint64_t)144000000)   /* Value in Hz */

/* Formula below works down to 70.3kHz (with presc ratio = 1) */ 
#define _100KHz_PERIOD ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 100000))

#define STM32F334_EVAL_BOARD

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
HRTIM_HandleTypeDef hhrtim;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void GPIO_HRTIM_outputs_Config(void);
static void HRTIM_Config_SinglePWM(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  HAL_Init();

  /* Configure the system clock to have a system clock = 72 Mhz */
  SystemClock_Config();

#ifdef STM32F334_EVAL_BOARD
  /* Initialize STM32F3348-DISCO LEDs */
  BSP_LED_Init(LED3);   // Indicates Error
  BSP_LED_Init(LED5);   // Indicates MCU is active
  /* Initialize User_Button on STM32F3348-DISCO */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO); 
#endif

  HRTIM_Config_SinglePWM();     /* Initialize HRTIM and related inputs */
  GPIO_HRTIM_outputs_Config();  /* Initialize GPIO's HRTIM outputs */
  /* Infinite loop */
  while (1)
  {
    BSP_LED_Toggle(LED5);
    HAL_Delay(100);
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
static void GPIO_HRTIM_outputs_Config(void)
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
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;;  
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;  
  GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void HRTIM_Config_SinglePWM(void)
{
  HRTIM_TimeBaseCfgTypeDef timebase_config;
  HRTIM_TimerCfgTypeDef timer_config;
  HRTIM_OutputCfgTypeDef output_config_TD1;
  HRTIM_BurstModeCfgTypeDef burst_mode_config;

  /* ----------------------------------------------- */
    /* HRTIM Global initialization: Clock and DLL init */
  /* ----------------------------------------------- */
  /* Initialize the HRTIM structure (minimal configuration) */
  hhrtim.Instance = HRTIM1;
  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IER_BMPER;
  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;

  /* Initialize HRTIM */
  HAL_HRTIM_Init(&hhrtim);

  /* HRTIM DLL calibration: periodic calibration, set period to 14µs */
  HAL_HRTIM_DLLCalibrationStart(&hhrtim, HRTIM_CALIBRATIONRATE_14);
  /* Wait calibration completion*/
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim, 100) != HAL_OK)
  {
    Error_Handler(); /* if DLL or clock is not correctly set */
  }
  
  
  burst_mode_config.ClockSource = HRTIM_BURSTMODECLOCKSOURCE_TIMER_D;
  burst_mode_config.IdleDuration = 3;
  burst_mode_config.Mode = HRTIM_BURSTMODE_CONTINOUS;
  burst_mode_config.Period = 7;
  burst_mode_config.PreloadEnable = HRIM_BURSTMODEPRELOAD_DISABLED;
  burst_mode_config.Prescaler = HRTIM_BURSTMODEPRESCALER_DIV1;
  burst_mode_config.Trigger = HRTIM_BURSTMODETRIGGER_TIMERD_CMP1;
  HAL_HRTIM_BurstModeConfig(&hhrtim, &burst_mode_config);
  
  /* ------------------------------------------------------------ */
  /* TIMERD initialization: set PWM frequency and continuous mode */
  /* ------------------------------------------------------------ */
  timebase_config.Period = _100KHz_PERIOD;
  timebase_config.RepetitionCounter = 0;
  timebase_config.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  timebase_config.Mode = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, &timebase_config);

  
  /* --------------------------------------------------------------------- */
  /* TIMERD global configuration: all values to default                    */
  /* --------------------------------------------------------------------- */
  timer_config.DMARequests = HRTIM_TIM_DMA_NONE;
  timer_config.DMASrcAddress = 0x0;
  timer_config.DMADstAddress = 0x0;
  timer_config.DMASize = 0x0;
  timer_config.HalfModeEnable = HRTIM_HALFMODE_ENABLED;
  timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_config.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  timer_config.InterruptRequests = HRTIM_TIM_IT_NONE;
  timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  timer_config.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  timer_config.UpdateTrigger= HRTIM_TIMUPDATETRIGGER_TIMER_D;
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_D,&timer_config);
  
  /* --------------------------------------------------------- */
  /* TD1 waveform description TD1 set on period, reset on CMP1 */
  /* --------------------------------------------------------- */
  output_config_TD1.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_config_TD1.SetSource = HRTIM_OUTPUTSET_TIMPER;
  output_config_TD1.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1;
  output_config_TD1.IdleMode = HRTIM_OUTPUTIDLEMODE_IDLE;
  output_config_TD1.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_config_TD1.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  output_config_TD1.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_config_TD1.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim,
                                 HRTIM_TIMERINDEX_TIMER_D,
                                 HRTIM_OUTPUT_TD1,
                                 &output_config_TD1);

  /* Set compare registers for duty cycle on TD1 */
//  compare_config.CompareValue = _100KHz_PERIOD/2;     // 50% duty cycle
//  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
//                                  HRTIM_TIMERINDEX_TIMER_D,
//                                  HRTIM_COMPAREUNIT_1,
//                                  &compare_config);

  /* ---------------*/
  /* HRTIM start-up */
  /* ---------------*/
  /* Enable HRTIM's outputs TD1 */
  /* Note: it is necessary to enable also GPIOs to have outputs functional */
  /* This must be done after HRTIM initialization */
  HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TD1); 


  /* Start HRTIM's TIMER D*/
  HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_TIMER_D);
  HAL_HRTIM_BurstModeCtl(&hhrtim, HRTIM_BURSTMODECTL_ENABLED);
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