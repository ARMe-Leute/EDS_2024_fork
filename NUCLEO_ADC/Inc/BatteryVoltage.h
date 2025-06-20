/**
 ******************************************************************************
 * @file           : BatteryVoltage.h
 * @brief          : Header file for reading analog voltage (e.g. battery).
 *                   Contains data structures and function prototypes for ADC setup
 *                   and voltage conversion. Supports internal temperature sensor
 *                   and low-pass filtering.
 * @note           : Only single-channel ADC is supported. Multi-channel operation
 *                   requires DMA and is not yet implemented.
 *
 * @date           : April 25, 2025
 ******************************************************************************
 */

#ifndef BATTERYVOLTAGE_H
#define BATTERYVOLTAGE_H

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <mcalADC.h>
#include <mcalGPIO.h>

/**
 * @brief Structure for single-channel ADC operation.
 */
typedef struct analogSingleCh
{
    ADC_TypeDef *adc;
    ADC_Common_TypeDef *adccommon;
    ADC_RESOLUTION_t chnResolution;
    size_t chnListSize;
    ADC_CHANNEL_t chnList[1];
    int chnADCValue[1];
    float chnVolt[1];
    float Prescaler[1];
    bool tempEnable;
    bool interruptEnable;
    float alpha_lowpass;
    float tempValue;
} analogSingleCh_t;

/**
 * @brief Structure for multi-channel ADC (not supported yet).
 * @note  Currently not supported. Multi-channel ADC operation requires DMA.
 */
typedef struct analogMultiCh
{
    ADC_TypeDef *adc;
    ADC_Common_TypeDef *adccommon;
    ADC_RESOLUTION_t chnResolution;
    size_t chnListSize;
    ADC_CHANNEL_t chnList[18];
    int chnADCValue[18];
    float chnVolt[18];
    float Prescaler[18];
    bool tempEnable;
    float scaler_lowpass;
    float tempValue;
} analogMultiCh_t;

/* Function declarations */
extern void InitADCGPIOPin(GPIO_TypeDef *port, PIN_NUM_t pin);
extern void InitADCChannels(analogSingleCh_t *analogCh);
extern void ADCInit(analogSingleCh_t *analogCh);
extern void conversionToVoltsCelsius(analogSingleCh_t *analogCh);
extern void getAnalogPinValuePolling(analogSingleCh_t *analogCh);
extern void getVoltagePinValuePolling(analogSingleCh_t *analogCh);

#endif /* BATTERYVOLTAGE_H */
