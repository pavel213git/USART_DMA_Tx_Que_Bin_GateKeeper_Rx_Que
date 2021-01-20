/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_TEST_H
#define __ADC_TEST_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_tim.h"

/* Define ------------------------------------------------------------------*/ 
/* Parameters of timer (used as ADC conversion trigger) */
  /* Timer frequency (unit: Hz). With a timer 16 bits and time base           */
  /* freq min 1Hz, range is min=1Hz, max=32kHz.                               */
  #define TIMER_FREQUENCY                ((uint32_t) 100)
  /* Timer minimum frequency (unit: Hz), used to calculate frequency range.   */
  /* With a timer 16 bits, maximum frequency will be 32000 times this value.  */
  #define TIMER_FREQUENCY_RANGE_MIN      ((uint32_t)    1)
  /* Timer prescaler maximum value (0xFFFF for a timer 16 bits)               */
  #define TIMER_PRESCALER_MAX_VALUE      ((uint32_t)0xFFFF-1)

/* Delay between ADC enable and ADC end of calibration.                     */
  /* Delay estimation in CPU cycles: Case of ADC calibration done             */
  /* immediately after ADC enable, ADC clock setting slow                     */
  /* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
  /* (CPU clock / ADC clock) is above 32.                                     */
  #define ADC_DELAY_ENABLE_CALIB_CPU_CYCLES  (LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32)
/* Definitions of data related to this example */
  /* Definition of ADCx conversions data table size */
  #define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  100)
  #define ARRAY_DISCARD_VALUE 10
/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       ((uint32_t)3300)

/* Exported variables ------------------------------------------------------------*/
extern __IO uint16_t aADCxConvertedData[];

/* Function prototypes -----------------------------------------------*/
extern void Configure_ADC(void);
extern void Activate_ADC(void);
extern void Configure_DMA_for_ADC(void);
extern void Configure_TIM_TimeBase_ADC_trigger(void);
extern void AdcDmaTransferComplete_Callback(void);
extern void AdcDmaTransferHalf_Callback(void);
extern void AdcDmaTransferError_Callback(void);
extern uint16_t AdcAvgConvVol(uint16_t *ptr, uint8_t arr_size);

#endif /* __ADC_TEST_H */

/************************END OF FILE****/