#ifndef __CONTROL_DEFINES
#define __CONTROL_DEFINES

#define STM32F334_EVAL_BOARD
//#define ARM_MATH_CM4
//#define PROFILLING

#define HRTIM_INPUT_CLOCK               ((uint64_t)144000000)   /* Value in Hz */

/*!< fHRCK: fHRTIM x 32U = 4.608 GHz - Resolution: 217 ps  - Min PWM frequency: 70.3 kHz (fHRTIM=144MHz)      */
/*!< fHRCK: fHRTIM x 16U = 2.304 GHz - Resolution: 434 ps  - Min PWM frequency: 35.1 KHz (fHRTIM=144MHz)      */
/*!< fHRCK: fHRTIM x 8U =  1.152 GHz - Resolution: 868 ps  - Min PWM frequency: 17.6 kHz (fHRTIM=144MHz)      */
/*!< fHRCK: fHRTIM x 4U =    576 MHz - Resolution: 1.73 ns - Min PWM frequency:  8.8 kHz (fHRTIM=144MHz)      */
/*!< fHRCK: fHRTIM x 2U =    288 MHz - Resolution: 3.47 ns - Min PWM frequency:  4.4 kHz (fHRTIM=144MHz)      */
/*!< fHRCK: fHRTIM =         144 MHz - Resolution: 6.95 ns - Min PWM frequency:  2.2 kHz (fHRTIM=144MHz)      */
/*!< fHRCK: fHRTIM / 2U =     72 MHz - Resolution: 13.88 ns- Min PWM frequency:  1.1 kHz (fHRTIM=144MHz)      */
/*!< fHRCK: fHRTIM / 4U =     36 MHz - Resolution: 27.7 ns - Min PWM frequency:    550Hz (fHRTIM=144MHz)      */
#define MULTIPLER                       16
#define HRTIM_OUPUT_CLOCK               HRTIM_INPUT_CLOCK * MULTIPLER           
#define HRTIM_OUPUT_CLOCK_KHZ           HRTIM_OUPUT_CLOCK/100                   // divider 100 beacause detector use 0.1kHz

#define MIN_FRQUENCY                    ((uint16_t)((HRTIM_OUPUT_CLOCK) / 80000))
#define MAX_FREQUENCY                   ((uint16_t)((HRTIM_OUPUT_CLOCK) / 180000))
//#define VIN(x)                          ((uint32_t) ((((REAL_3V3*x)/0x1000)*10000) / VIN_RESISTOR_RATIO))
#define VIN(x)                          ((uint32_t) ((REAL_3V3*x)/0x1000))
#define VIN_RESISTOR_RATIO              ((uint16_t) 2012) /* theoretical bridge resistor Vin ratio (6.8K/(6.8K + 27K))*10000: can be adjusted if needed by measuring the real resistors ratio */
#define REAL_3V3                        ((uint16_t) 3300) /* Measure 3V3 of application and enter it as reference in mV (default value is 3300) */
#define SAMPLEFILTER_TAP_NUM            23
#define VOUT_TARGET                     0 // 5000 -> 5V
#define BURST_MODE_PERIOD               19

//TODO fix percent of burst mode control - problem with 50% > burst mode control
#define ADC_12_BIT_RESULUTION           0xFFF
#define DIFERENCE_FREQUENCY_REG_VALUE   (MIN_FRQUENCY - MAX_FREQUENCY)
#define PERCENT_OF_BURST_MODE_CONTROL   200                                     //200 = 20%
#define MAX_BURST_CONTROL_VALUE         (((DIFERENCE_FREQUENCY_REG_VALUE)*(PERCENT_OF_BURST_MODE_CONTROL))/1000)
#define MAX_VALUE_OF_FEEDBACK_LOOP      (DIFERENCE_FREQUENCY_REG_VALUE) + (MAX_BURST_CONTROL_VALUE)
#define LINK_ADC_TO_FEEDBACK            (((MAX_VALUE_OF_FEEDBACK_LOOP)*(1000))/(ADC_12_BIT_RESULUTION))
#define LINK_TO_BURST_MODE              ((MAX_BURST_CONTROL_VALUE) / (BURST_MODE_PERIOD))

#define KP                              4

#endif  /* __CONTROL_DEFINES */