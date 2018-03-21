/* Includes ------------------------------------------------------------------*/
#include "rezonans_detector.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t value = 1100;
uint32_t Detection_in_progress = 1;
uint32_t measurement_done;
int32_t counter;
uint32_t adc_sample[64];


#define start_timer()   *((volatile uint32_t*)0xE0001000) = 0x40000001  // Enable CYCCNT register
#define stop_timer()    *((volatile uint32_t*)0xE0001000) = 0x40000000  // Disable CYCCNT register
#define get_timer()     *((volatile uint32_t*)0xE0001004)               // Get value from CYCCNT register
uint32_t it1, it2;      // start and stop flag


struct RezonansValue {
  uint32_t main_frequency_power;
  uint32_t frequency;
} rezonans_value;

uint32_t real_frequency = 0;
//uint32_t best_value[2];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint32_t rezonans_detector()
{
  uint32_t frequency = 4608000/value;
  int16_t complex[64][2];
  int32_t P16;
  rezonans_value.frequency = 0;
  rezonans_value.main_frequency_power = 0;
  
  uint32_t ifftFlag = 0;
  uint32_t doBitReverse = 1;
  uint16_t angle;
  
  // inicjalizacja peryferium pod skanowanie (pojedynczy strzal z burst mode ustalonym na 4sygnaly)
  HRTIM_Config_rezonans_detector(frequency);
  GPIO_HRTIM_outputs_Config();
  //HAL_HRTIM_BurstModeCtl(&hhrtim, HRTIM_BURSTMODECTL_ENABLED);
  ADC_Config();
  Start_Detector();
  HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TD1);
  
  //Start driver
  while(Detection_in_progress)
  {
    if(measurement_done)
    {
      
      start_timer();          // start the timer.
      it1 = get_timer();      // store current cycle-count in a local
      
      for(int32_t i = 0; i < 64; i++)
      {
        complex[i][0] = adc_sample[i];
        complex[i][1] = 0;
      }

      arm_cfft_q15(&arm_cfft_sR_q15_len64, (q15_t*)complex, ifftFlag, doBitReverse);
      
      angle = fxpt_atan2(complex[16][0], complex[16][1]);     
       
      complex[16][0] = complex[16][0] < 0 ? -complex[16][0] : complex[16][0];
      complex[16][1] = complex[16][1] < 0 ? -complex[16][1] : complex[16][1];
      P16 = complex[16][0]*complex[16][0] + complex[16][1]*complex[16][1];

      uint32_t j = 1;
      uint32_t temp = P16;
      while(P16 > j)
      {
        P16 = (P16 + j)/2;
        j = temp/P16;
      }      
      
      if(rezonans_value.main_frequency_power < P16 && angle > 0x8000)
      {
        rezonans_value.main_frequency_power = P16;
        rezonans_value.frequency = value;
      } 
      
      float float_temp = 1/((1/(float)rezonans_value.frequency) - 0.00005);
      real_frequency = (uint32_t)float_temp; 

      it2 = get_timer() - it1;    // Derive the cycle-count difference
      stop_timer();               // If timer is not needed any more, stop  
      
        HAL_Delay(5);

      value -= 1;
      if(value <= 705)
      {
        value = 1100;
        Detection_in_progress = 0;
      }
      frequency = 46080000/value;
      
      //Set new ADC triger
      __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_2, (25 * frequency)/200);
      __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_3, (50 * frequency)/200);
      __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_4, (75 * frequency)/200);
      
      __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, (0 * frequency)/100);
      __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, (50 * frequency)/100);
      
      //Set new period value for timers
      __HAL_HRTIM_SETPERIOD(&hhrtim, HRTIM_TIMERINDEX_MASTER, frequency);
      __HAL_HRTIM_SETPERIOD(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, frequency);
      __HAL_HRTIM_SETPERIOD(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, frequency/2);
      counter = 0;
      
      Start_Detector();
      measurement_done = 0;
    }
  }
  Stop_Detector();
  return (46080000/(real_frequency));
}

/**
* @brief  Configure HRTIM
* @param  None
* @retval None
*/
static void HRTIM_Config_rezonans_detector(uint32_t frequency)
{

  /* ----------------------------------------------- */
  /* HRTIM Global initialization: Clock and DLL init */
  /* ----------------------------------------------- */
  /* Initialize the HRTIM structure (minimal configuration) */
  hhrtim.Instance = HRTIM1;
  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IER_BMPER; /*!<  Burst mode period interrupt enable */
  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;

  /* Initialize HRTIM */
  HAL_HRTIM_Init(&hhrtim);

  /* HRTIM DLL calibration: periodic calibration, set period to 14µs */
  HAL_HRTIM_DLLCalibrationStart(&hhrtim, HRTIM_CALIBRATIONRATE_14);
  /* Wait calibration completion*/
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim, 100) != HAL_OK)
  {
    while(1); /* if DLL or clock is not correctly set */
  }
  
  /* ------------------------------------------------------------ */
  /* TIMERD initialization: set Hal Mode frequency and continuous mode */
  /* ------------------------------------------------------------ */
  timebase_config.Period = frequency;                                           //start frequency value
  timebase_config.RepetitionCounter = 0;                                        //Event rate divider
  timebase_config.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  timebase_config.Mode = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &timebase_config);
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, &timebase_config);// half bridge control
  
  timebase_config.Period = frequency/2;                                         //Timer C work with 2x frequency
  timebase_config.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, &timebase_config);//trigering ADC

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
  
  compare_config.CompareValue = (5 * frequency)/100;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_MASTER,
                                  HRTIM_COMPAREUNIT_2,
                                  &compare_config);

  /* Compare 3 is used for SR2 turn-on */
  compare_config.CompareValue = (55 * frequency)/100;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_MASTER,
                                  HRTIM_COMPAREUNIT_3,
                                  &compare_config);
  
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
  timer_config.InterruptRequests = HRTIM_TIM_IT_NONE;
  timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  timer_config.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  timer_config.UpdateTrigger= HRTIM_TIMUPDATETRIGGER_MASTER;
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER + HRTIM_TIMRESETTRIGGER_MASTER_CMP1;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_D,&timer_config);
  
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_C,&timer_config);
  
 /* ----------------------------------------------- */
  /* Configure Master timer compare events           */
  /* Compare 1 is automatically computed (Half mode) */
  /* ----------------------------------------------- */
  
  /* --------------------------------------------------------- */
  /* TD1 waveform description TD1 set on period, reset on CMP1 */
  /* Compare 1 is automatically computed (Half mode)           */
  /* --------------------------------------------------------- */
  output_config_TD1.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_config_TD1.SetSource = HRTIM_OUTPUTSET_MASTERCMP1;
  output_config_TD1.ResetSource  = HRTIM_OUTPUTSET_MASTERPER;
  output_config_TD1.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  output_config_TD1.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_config_TD1.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  output_config_TD1.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_config_TD1.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim,
                                 HRTIM_TIMERINDEX_TIMER_D,
                                 HRTIM_OUTPUT_TD1,                              //HRTIM1_CHD1 PB14
                                 &output_config_TD1);

  /* Set compare registers for turn-off time on TB1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (25 * frequency)/200;                           //Set 1st adc triger to 12.5% and 62.5% of period (45 and 225 degrees)
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_C,
                                  HRTIM_COMPAREUNIT_2,
                                  &compare_config);

  /* Set compare 3 for sampling before turn-on on SR1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (50 * frequency)/200;                           //Set 2nd adc triger to 25% and 75% of period (90 and 270 degrees)
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_C,
                                  HRTIM_COMPAREUNIT_3,
                                  &compare_config);

  /* Set compare 4 for sampling before turn-off on SR1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (75 * frequency)/200;                           //Set 2nd adc triger to 37.5% and 87.5% of period (135 and 315 degrees)
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_C,
                                  HRTIM_COMPAREUNIT_4,
                                  &compare_config);
 
 
  adc_trigger_config.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERC_CMP2
                             + HRTIM_ADCTRIGGEREVENT24_TIMERC_CMP3
                             + HRTIM_ADCTRIGGEREVENT24_TIMERC_CMP4
                             + HRTIM_ADCTRIGGEREVENT24_TIMERC_PERIOD;           //Set reset event as adc triger to 50% and 100% of period (180 and 360 degrees)
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_C;
  
  HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_2, &adc_trigger_config);
  
  HAL_HRTIM_SoftwareUpdate(&hhrtim,
                        HRTIM_TIMERUPDATE_MASTER
                      + HRTIM_TIMERUPDATE_A
                      + HRTIM_TIMERUPDATE_B
                      + HRTIM_TIMERUPDATE_C
                      + HRTIM_TIMERUPDATE_D
                      + HRTIM_TIMERUPDATE_E);
}

/**
  * @brief  This function start detector.
  * @param  None
  * @retval None
  */
void Start_Detector(void)
{  
  
  hhrtim.Instance->sMasterRegs.MCNTR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CNTxR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CNTxR = 0x0000;
  
  /* Start HRTIM's Master timer, TIMER A and B */
  hhrtim.Instance->sCommonRegs.CR2 = (HRTIM_TIMERRESET_MASTER|HRTIM_TIMERRESET_TIMER_C|HRTIM_TIMERRESET_TIMER_D | HRTIM_TIMERUPDATE_MASTER|HRTIM_TIMERUPDATE_C|HRTIM_TIMERUPDATE_D);
  hhrtim.Instance->sMasterRegs.MCR |= (HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);

  GPIOB->AFR[1] |= 0xDD000000;  //Set GPIOB alternative function PIN 14 and 15
  GPIOB->MODER  |= 0xA0000000;  //SET GPIOB port mode PIN 14 and 15 to Alternative function mode
}

/**
  * @brief  This function stop detector.
  * @param  None
  * @retval None
  */
void Stop_Detector(void)
{
  hhrtim.Instance->sMasterRegs.MCR &= ~(HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D); 
  hhrtim.Instance->sCommonRegs.CR2 = (HRTIM_TIMERRESET_MASTER|HRTIM_TIMERRESET_TIMER_C|HRTIM_TIMERRESET_TIMER_D | HRTIM_TIMERUPDATE_MASTER|HRTIM_TIMERUPDATE_C|HRTIM_TIMERUPDATE_D);
  
  HAL_HRTIM_WaveformSetOutputLevel(&hhrtim, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, HRTIM_OUTPUTLEVEL_INACTIVE);
  
  hhrtim.Instance->sMasterRegs.MCNTR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CNTxR = 0x0000;
  hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CNTxR = 0x0000;
}