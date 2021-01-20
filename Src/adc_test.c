/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "adc_test.h"

/* Global variables ---------------------------------------------------------*/
/* Variables for ADC conversion data */
__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data */
uint16_t adc_raw_data[ADC_CONVERTED_DATA_BUFFER_SIZE] = {0};

/**
  * @brief  Configure timer as a time base (timer instance: TIM3) 
  *         used to trig ADC conversion start.
  * @note   In this ADC example, timer instance must be on APB1 (clocked by PCLK1)
  *         to be compliant with frequency calculation used in this function.
  * @param  None
  * @retval None
  */
void Configure_TIM_TimeBase_ADC_trigger(void)
{
  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
  uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
  uint32_t timer_reload = 0;                      /* Timer reload value in function of timer prescaler to achieve time base period */
  
  /*## Configuration of NVIC #################################################*/
  /* Note: In this example, timer interrupts are not activated.               */
  /*       If needed, timer interruption at each time base period is          */
  /*       possible.                                                          */
  /*       Refer to timer examples.                                           */
  
  /* Configuration of timer as time base:                                     */ 
  /* Caution: Computation of frequency is done for a timer instance on APB1   */
  /*          (clocked by PCLK1)                                              */
  /* Timer frequency is configured from the following constants:              */
  /* - TIMER_FREQUENCY: timer frequency (unit: Hz).                           */
  /* - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency possible            */
  /*   (unit: Hz).                                                            */
  /* Note: Refer to comments at these literals definition for more details.   */
  
  /* Retrieve timer clock source frequency */
  /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
  /* clock source.                                                            */
  if (LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1)
  {
    timer_clock_frequency = __LL_RCC_CALC_PCLK1_FREQ(SystemCoreClock, LL_RCC_GetAPB1Prescaler());
  }
  else
  {
    timer_clock_frequency = (__LL_RCC_CALC_PCLK1_FREQ(SystemCoreClock, LL_RCC_GetAPB1Prescaler()) * 2);
  }
  
  /* Timer prescaler calculation */
  /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
  timer_prescaler = ((timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1);
  /* Timer reload calculation */
  timer_reload = (timer_clock_frequency / (timer_prescaler * TIMER_FREQUENCY));
  
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  
  /* Set timer pre-scaler value */
  LL_TIM_SetPrescaler(TIM3, (timer_prescaler - 1));
  
  /* Set timer auto-reload value */
  LL_TIM_SetAutoReload(TIM3, (timer_reload - 1));
  
  /* Counter mode: select up-counting mode */
  LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP); 
  
  /* Set the repetition counter */
  LL_TIM_SetRepetitionCounter(TIM3, 0);
  
  /* Note: In this example, timer interrupts are not activated.               */
  /*       If needed, timer interruption at each time base period is          */
  /*       possible.                                                          */
  /*       Refer to timer examples.                                           */
  
  /* Set timer the trigger output (TRGO) */
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM3);
}

/**
  * @brief  Configure ADC (ADC instance: ADC1) and GPIO used by ADC channels.
  * @note   In case re-use of this function outside of this example:
  *         This function includes checks of ADC hardware constraints before
  *         executing some configuration functions.
  *         - In this example, all these checks are not necessary but are
  *           implemented anyway to show the best practice usages
  *           corresponding to reference manual procedure.
  *           (On some STM32 series, setting of ADC features are not
  *           conditioned to ADC state. However, in order to be compliant with
  *           other STM32 series and to show the best practice usages,
  *           ADC state is checked anyway with same constraints).
  *           Software can be optimized by removing some of these checks,
  *           if they are not relevant considering previous settings and actions
  *           in user application.
  *         - If ADC is not in the appropriate state to modify some parameters,
  *           the setting of these parameters is bypassed without error
  *           reporting:
  *           it can be the expected behavior in case of recall of this 
  *           function to update only a few parameters (which update fullfills
  *           the ADC state).
  *           Otherwise, it is up to the user to set the appropriate error 
  *           reporting in user application.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_ADC(void)
{
  /*## Configuration of GPIO used by ADC channels ############################*/
  
  /* Note: On this STM32 device, ADC1 channel 4 is mapped on GPIO pin PA.04 */ 
  
  /* Enable GPIO Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  
  /* Configure GPIO in analog mode to be used as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable ADC1 interruptions */
  NVIC_SetPriority(ADC1_IRQn, 8); /* ADC IRQ greater priority than DMA IRQ */
  NVIC_EnableIRQ(ADC1_IRQn);
  
  /*## Configuration of ADC ##################################################*/
  
  /*## Configuration of ADC hierarchical scope: common to several ADC ########*/
  
  /* Enable ADC clock (core clock) */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if(__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */
    
    /* Set ADC measurement path to internal channels */
    // LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);
    
    
  /*## Configuration of ADC hierarchical scope: multimode ####################*/
  
    /* Set ADC multimode configuration */
    // LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_INDEPENDENT);
    
    /* Set ADC multimode DMA transfer */
    // LL_ADC_SetMultiDMATransfer(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_REG_DMA_EACH_ADC);
    
    /* Set ADC multimode: delay between 2 sampling phases */
    // LL_ADC_SetMultiTwoSamplingDelay(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_TWOSMP_DELAY_1CYCLE);
    
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC instance #################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */
    
    /* Set ADC conversion data alignment */
    // LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    
    /* Set Set ADC sequencers scan mode, for all ADC groups                   */
    /* (group regular, group injected).                                       */
    // LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);
    
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC group regular ############*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Set ADC group regular trigger source */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);//LL_ADC_REG_TRIG_SOFTWARE);
    
    /* Set ADC group regular trigger polarity */
    // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
    
    /* Set ADC group regular continuous mode */
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);//LL_ADC_REG_CONV_CONTINUOUS);//
    
    /* Set ADC group regular conversion data transfer */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    
    /* Set ADC group regular sequencer */
    /* Note: On this STM32 serie, ADC group regular sequencer is              */
    /*       fully configurable: sequencer length and each rank               */
    /*       affectation to a channel are configurable.                       */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_REG_SetSequencerLength()".                               */
    
    /* Set ADC group regular sequencer length and scan direction */
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    
    /* Set ADC group regular sequencer discontinuous mode */
    // LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    
    /* Set ADC group regular sequence: channel on the selected sequence rank. */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
  }
  
  
  /*## Configuration of ADC hierarchical scope: ADC group injected ###########*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */
    
    /* Set ADC group injected trigger source */
    // LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
    
    /* Set ADC group injected trigger polarity */
    // LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    
    /* Set ADC group injected conversion trigger  */
    // LL_ADC_INJ_SetTrigAuto(ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);
    
    /* Set ADC group injected sequencer */
    /* Note: On this STM32 serie, ADC group injected sequencer is             */
    /*       fully configurable: sequencer length and each rank               */
    /*       affectation to a channel are configurable.                       */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_INJ_SetSequencerLength()".                               */
    
    /* Set ADC group injected sequencer length and scan direction */
    // LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);
    
    /* Set ADC group injected sequencer discontinuous mode */
    // LL_ADC_INJ_SetSequencerDiscont(ADC1, LL_ADC_INJ_SEQ_DISCONT_DISABLE);
    
    /* Set ADC group injected sequence: channel on the selected sequence rank. */
    // LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_4);
  }
  
  
  /*## Configuration of ADC hierarchical scope: channels #####################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Set ADC channels sampling time */
    /* Note: Considering interruption occurring after each number of          */
    /*       "ADC_CONVERTED_DATA_BUFFER_SIZE" ADC conversions                 */
    /*       (IT from DMA transfer complete),                                 */
    /*       select sampling time and ADC clock with sufficient               */
    /*       duration to not create an overhead situation in IRQHandler.      */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_7CYCLES_5);
    
  }
  
  
  /*## Configuration of ADC transversal scope: analog watchdog ###############*/
  
  /* Note: On this STM32 serie, there is only 1 analog watchdog available.    */
  
  /* Set ADC analog watchdog: channels to be monitored */
  // LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_DISABLE);
  
  /* Set ADC analog watchdog: thresholds */
  // LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_HIGH, __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B));
  // LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_LOW, 0x000);
  
  
  /*## Configuration of ADC transversal scope: oversampling ##################*/
  
  /* Note: Feature not available on this STM32 serie */
  
  
  /*## Configuration of ADC interruptions ####################################*/
  /* Note: In this example, no ADC interruption enabled */
  
  /* Note: in this example, ADC group regular end of conversions              */
  /*       (number of ADC conversions defined by DMA buffer size)             */
  /*       are notified by DMA transfer interruptions).                       */
  
}

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @note   Operations:
  *         - ADC instance
  *           - Run ADC self calibration
  *           - Enable ADC
  *         - ADC group regular
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  *         - ADC group injected
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  * @param  None
  * @retval None
  */
void Activate_ADC(void)
{
    __IO uint32_t wait_loop_index = 0;
#if (USE_TIMEOUT == 1)
    uint32_t Timeout = 0; /* Variable used for timeout management */
#endif /* USE_TIMEOUT */
    
    /*## Operation on ADC hierarchical scope: ADC instance #####################*/
    
    /* Note: Hardware constraint (refer to description of the functions         */
    /*       below):                                                            */
    /*       On this STM32 serie, setting of these features are not             */
    /*       conditioned to ADC state.                                          */
    /*       However, in order to be compliant with other STM32 series          */
    /*       and to show the best practice usages, ADC state is checked.        */
    /*       Software can be optimized by removing some of these checks, if     */
    /*       they are not relevant considering previous settings and actions    */
    /*       in user application.                                               */
    if (LL_ADC_IsEnabled(ADC1) == 0)
    {
        /* Enable ADC */
        LL_ADC_Enable(ADC1);
        
        /* Delay between ADC enable and ADC start of calibration.                 */
        /* Note: Variable divided by 2 to compensate partially                    */
        /*       CPU processing cycles (depends on compilation optimization).     */
        wait_loop_index = (ADC_DELAY_ENABLE_CALIB_CPU_CYCLES >> 1);
        while(wait_loop_index != 0)
        {
            wait_loop_index--;
        }
        
        /* Run ADC self calibration */
        LL_ADC_StartCalibration(ADC1);
        
        /* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */
        
        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
        {
#if (USE_TIMEOUT == 1)
            /* Check Systick counter flag to decrement the time-out value */
            if (LL_SYSTICK_IsActiveCounterFlag())
            {
                if(Timeout-- == 0)
                {
                    /* Time-out occurred. Set LED to blinking mode */
                    LED_Blinking(LED_BLINK_ERROR);
                }
            }
#endif /* USE_TIMEOUT */
        }
        
    }
    
    /* Start ADC group regular conversion */
    /* Note: Hardware constraint (refer to description of the functions         */
    /*       below):                                                            */
    /*       On this STM32 serie, setting of these features are not             */
    /*       conditioned to ADC state.                                          */
    /*       However, in order to be compliant with other STM32 series          */
    /*       and to show the best practice usages, ADC state is checked.        */
    /*       Software can be optimized by removing some of these checks, if     */
    /*       they are not relevant considering previous settings and actions    */
    /*       in user application.                                               */
    if (LL_ADC_IsEnabled(ADC1) == 1)
    {
        LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
        //LL_ADC_REG_StartConversionSWStart(ADC1);
    }
    else
    {
        /* Error: ADC conversion start could not be performed */
        LED_Blinking(LED_BLINK_ERROR);
    }
    
    /*## Operation on ADC hierarchical scope: ADC group regular ################*/
    /* Note: No operation on ADC group regular performed here.                  */
    /*       ADC group regular conversions to be performed after this function  */
    /*       using function:                                                    */
    /*       "LL_ADC_REG_StartConversion();"                                    */
    
    /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
    /* Note: No operation on ADC group injected performed here.                 */
    /*       ADC group injected conversions to be performed after this function */
    /*       using function:                                                    */
    /*       "LL_ADC_INJ_StartConversion();"                                    */
    
}


/**
  * @brief  This function configures DMA for transfer of data from ADC
  * @param  None
  * @retval None
  */
void Configure_DMA_for_ADC(void)
{
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable DMA interruptions */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 9); /* DMA IRQ lower priority than ADC IRQ */
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
  /*## Configuration of DMA ##################################################*/
  /* Enable the peripheral clock of DMA */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  
  /* Configure the DMA transfer */
  /*  - DMA transfer in circular mode to match with ADC configuration:        */
  /*    DMA unlimited requests.                                               */
  /*  - DMA transfer from ADC without address increment.                      */
  /*  - DMA transfer to memory with address increment.                        */
  /*  - DMA transfer from ADC by half-word to match with ADC configuration:   */
  /*    ADC resolution 12 bits.                                               */
  /*  - DMA transfer to memory by half-word to match with ADC conversion data */
  /*    buffer variable type: half-word.                                      */
  LL_DMA_ConfigTransfer(DMA1,
                        LL_DMA_CHANNEL_1,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_MODE_CIRCULAR              |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_HALFWORD        |
                        LL_DMA_MDATAALIGN_HALFWORD        |
                        LL_DMA_PRIORITY_HIGH               );
  
  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&aADCxConvertedData,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_1,
                       ADC_CONVERTED_DATA_BUFFER_SIZE);
  
  /* Enable DMA transfer interruption: transfer complete */
  LL_DMA_EnableIT_TC(DMA1,
                     LL_DMA_CHANNEL_1);
  
  /* Enable DMA transfer interruption: half transfer */
  LL_DMA_EnableIT_HT(DMA1,
                     LL_DMA_CHANNEL_1);
  
  /* Enable DMA transfer interruption: transfer error */
  LL_DMA_EnableIT_TE(DMA1,
                     LL_DMA_CHANNEL_1);
  
  /*## Activation of DMA #####################################################*/
  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1,
                       LL_DMA_CHANNEL_1);
}

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferComplete_Callback(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
            
    //ptr = aADCxConvertedData;
    /* Computation of ADC conversions raw data to physical values               */
    /* using LL ADC driver helper macro.                                        */
    /* Management of the 2nd half of the buffer */
    if(((flag_DMA & DMA_FULL_TRANSFER_ADC) != DMA_FULL_TRANSFER_ADC))
    {
//        for (tmp_index = (ADC_CONVERTED_DATA_BUFFER_SIZE/2); tmp_index < ADC_CONVERTED_DATA_BUFFER_SIZE; tmp_index++)
//        {
//            //aADCxConvertedData_Voltage_mVolt[tmp_index] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index], LL_ADC_RESOLUTION_12B);
//            adc_raw_data[tmp_index] = aADCxConvertedData[tmp_index];
//        }
        
        SET_BIT(flag_DMA, DMA_FULL_TRANSFER_ADC);        
    }    
    
    if (xSemaphoreGiveFromISR(xBinSemaADC, &xHigherPriorityTaskWoken) != pdTRUE) 
    {
        /* Processing Error */
        LED_Blinking(LED_BLINK_ERROR);
    }
    
    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }  
}

/**
  * @brief  DMA half transfer callback
  * @note   This function is executed when the half transfer interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferHalf_Callback(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    //uint32_t tmp_index = 0;
    
    /* Computation of ADC conversions raw data to physical values               */
    /* using LL ADC driver helper macro.                                        */
    /* Management of the 1st half of the buffer */
    if((flag_DMA & DMA_HALF_TRANSFER_ADC) != DMA_HALF_TRANSFER_ADC)
    {
//        for (tmp_index = 0; tmp_index < (ADC_CONVERTED_DATA_BUFFER_SIZE/2); tmp_index++)
//        {
//            //            aADCxConvertedData_Voltage_mVolt[tmp_index] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index], LL_ADC_RESOLUTION_12B);
//            adc_raw_data[tmp_index] = aADCxConvertedData[tmp_index];
//        } 
        SET_BIT(flag_DMA, DMA_HALF_TRANSFER_ADC);
    }   
    
    if (xSemaphoreGiveFromISR(xBinSemaADC, &xHigherPriorityTaskWoken) != pdTRUE) 
    {
        /* Processing Error */
        LED_Blinking(LED_BLINK_ERROR);
    }
    
    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }  
}

/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void AdcDmaTransferError_Callback(void)
{
  /* Error detected during DMA transfer */
  LED_Blinking(LED_BLINK_ERROR);
}

//-*****************************************************************************************************
// Function Name  : adc_get_avgvalue
// Description    : takes the avg of collected max values from DMA and calculate peak voltage of waves
// inputs         : pointer to samples, address of outputs to be stored
// Outputs        : stores the maximum in the given address
// Returns        : none
//-******************************************************************************************************

uint16_t AdcAvgConvVol(uint16_t *ptr, uint8_t arr_size)
{
    uint8_t i,j;
    uint16_t avg = 0, adc_voltage = 0;
    uint32_t sum = 0 ;    
    
    
    for(i=0; i < (arr_size); i++)
    {
        for(j=i+1; j < (arr_size); j++)
        {
            if(ptr[i] > ptr[j])
            {
                ptr[i] = ptr[i] + ptr[j];
                ptr[j] = ptr[i] - ptr[j];
                ptr[i] = ptr[i] - ptr[j];
            }
        }
    }
    
    for(i = ARRAY_DISCARD_VALUE; i < (arr_size - ARRAY_DISCARD_VALUE); i++)
    {
        sum += ptr[i];            
    }
    avg = sum / (arr_size - (ARRAY_DISCARD_VALUE * 2));
    adc_voltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, avg, LL_ADC_RESOLUTION_12B);        
    return adc_voltage;
}