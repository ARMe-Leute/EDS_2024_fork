/*
 * mcalADC.h
 * Rev: 1.1
 *  Created on: Dec 18, 2020
 *      Author: Ralf
 *      edited by: Tobias Flaemig Dez, 2024
 */

#ifndef MCALADC_H_
#define MCALADC_H_

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @ingroup adc4
 * @{
 */

typedef enum
{
    ADC_OK      = 0,
    ADC_INVALID_RESOLUTION          = -120,
    ADC_INVALID_IRQ_TYPE            = -121,
    ADC_INVALID_CHANNEL_NUM         = -122,
    ADC_INVALID_NUM_SAMPLE_CYCLES   = -123,
    ADC_INVALID_SEQUENCE_LENGTH     = -124,
    ADC_INVALID_WATCHDOG_TYPE
} ADC_RETURN_CODE_t;

typedef enum
{
    ADC_RES_12BIT    = 0,
    ADC_RES_10BIT,
    ADC_RES_8BIT,
    ADC_RES_6BIT
} ADC_RESOLUTION_t;

typedef enum
{
    ADC_IRQ_OVERRUN         = 0,
    ADC_WATCHDOG_IRQ_EN,
    ADC_EOC_INJECTED_GRP,
    ADC_EOC_REGULAR_GRP
} ADC_IRQTYPES_t;

typedef enum
{
    ADC_CHN_0  = 0,
    ADC_CHN_1,
    ADC_CHN_2,
    ADC_CHN_3,
    ADC_CHN_4,
    ADC_CHN_5,
    ADC_CHN_6,
    ADC_CHN_7,
    ADC_CHN_8,
    ADC_CHN_9,
    ADC_CHN_10,
    ADC_CHN_11,
    ADC_CHN_12,
    ADC_CHN_13,
    ADC_CHN_14,
    ADC_CHN_15,
    ADC_CHN_16,		// cpuTemp
    ADC_CHN_17,
    ADC_CHN_18,
} ADC_CHANNEL_t;

typedef enum
{
    NO_ADC_WATCHDDOG                      = 0,
    ALL_INJECTED_CHANNELS,
    ALL_REGULAR_CHANNELS,
    ALL_REGULAR_AND_INJECTED_CHANNELS,
    SINGLE_INJECTED_CHANNEL,
    SINGLE_REGULAR_CHANNEL,
    SINGLE_REGULAR_AND_INJECTED_CHANNEL
} ADC_WATCHDOG_t;

typedef enum
{
    SAMPLE_CYCLES_3   = 0,
    SAMPLE_CYCLES_15,
    SAMPLE_CYCLES_28,
    SAMPLE_CYCLES_56,
    SAMPLE_CYCLES_84,
    SAMPLE_CYCLES_112,
    SAMPLE_CYCLES_144,
    SAMPLE_CYCLES_480,
} ADC_SAMPLING_CYCLES_t;

/**
 * @}
 */

/* Deprecated function(s) */
extern ADC_RETURN_CODE_t adcInitADC(ADC_TypeDef *adc);
extern ADC_RETURN_CODE_t adcDeinitADC(ADC_TypeDef *adc);

extern ADC_RETURN_CODE_t adcSelectADC(ADC_TypeDef *adc);
extern ADC_RETURN_CODE_t adcDeselectADC(ADC_TypeDef *adc);
extern ADC_RETURN_CODE_t adcEnableADC(ADC_TypeDef *adc);
extern ADC_RETURN_CODE_t adcDisableADC(ADC_TypeDef *adc);
extern uint16_t          adcGetConversionResult(ADC_TypeDef *adc);
extern ADC_RETURN_CODE_t adcSetResolution(ADC_TypeDef *adc, ADC_RESOLUTION_t res);
extern ADC_RETURN_CODE_t adcGetResolution(ADC_TypeDef *adc, ADC_RESOLUTION_t *res);
extern bool              adcIsAdcRunning(ADC_TypeDef *adc);
extern void              adcStartConversion(ADC_TypeDef *adc);
extern bool              adcIsConversionFinished(ADC_TypeDef *adc);

extern ADC_RETURN_CODE_t adcEnableInterrupt(ADC_TypeDef *adc, ADC_IRQTYPES_t irq);
extern ADC_RETURN_CODE_t adcDisableInterrupt(ADC_TypeDef *adc, ADC_IRQTYPES_t irq);
extern ADC_RETURN_CODE_t adcSetSampleCycles(ADC_TypeDef *adc, ADC_CHANNEL_t chn, ADC_SAMPLING_CYCLES_t cycles);


//extern ADC_RETURN_CODE_t adcEnableAnalogWatchdog(ADC_TypeDef *adc, ADC_WATCHDOG_CHANNEL_t wdtype);
//extern ADC_RETURN_CODE_t adcDisableAnalogWatchdog(ADC_TypeDef *adc, ADC_WATCHDOG_CHANNEL_t wdtype);
extern ADC_RETURN_CODE_t adcSelectAnalogWatchdogType(ADC_TypeDef *adc, ADC_WATCHDOG_t type);
extern ADC_RETURN_CODE_t adcEnableWatchdogForSingleChannel(ADC_TypeDef *adc, ADC_CHANNEL_t chn);
extern ADC_RETURN_CODE_t adcDisableWatchdogForSingleChannel(ADC_TypeDef *adc, ADC_CHANNEL_t chn);
extern ADC_RETURN_CODE_t adcEnableWatchdogForAllChannels(ADC_TypeDef *adc);
extern ADC_RETURN_CODE_t adcDisableWatchdogForAllChannels(ADC_TypeDef *adc);

extern ADC_RETURN_CODE_t adcSetChannelSequence(ADC_TypeDef *adc, ADC_CHANNEL_t *chnList, size_t listSize);

#ifdef __cplusplus
}
#endif


#endif /* MCALADC_H_ */
