#ifndef __CONTROL_DEFINES
#define __CONTROL_DEFINES

#define STM32F334_EVAL_BOARD
#define ARM_MATH_CM4
#define PROFILLING

#define HRTIM_INPUT_CLOCK               ((uint64_t)144000000)   /* Value in Hz */
/* Formula below works down to 70.3kHz (with presc ratio = 1) */ 
#define MIN_FRQUENCY                    ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 110000))
#define MAX_FREQUENCY                   ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 180000))
#define VIN(x)                          ((uint32_t) ((((REAL_3V3*x)/0x1000)*10000) / VIN_RESISTOR_RATIO))
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