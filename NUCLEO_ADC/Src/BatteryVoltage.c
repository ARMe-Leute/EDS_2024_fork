/**
 ******************************************************************************
 * @file           : BatteryVoltage.c
 * @author         : Erick Schlosser
 * @brief          : This file implements the logic to read analog voltages
 *                   using the ADC (Analog-to-Digital Converter). It supports
 *                   single-channel mode, including optional temperature reading,
 *                   and uses a simple low-pass filter to smooth the results.
 *
 *                   Multichannel mode is not supported in this implementation
 *                   because it requires DMA (Direct Memory Access).
 *                   Only polling and interrupt-based single-channel modes
 *                   are currently implemented.
 *
 * @date           : April 25, 2025
 ******************************************************************************
 */

#include <stdint.h>        /* Standard integer types (e.g., uint32_t) */
#include <stdbool.h>       /* Boolean type (true/false) */
#include <stddef.h>        /* For size_t and NULL */
#include <mcalGPIO.h>      /* GPIO driver */
#include <mcalADC.h>       /* ADC driver */
#include "BatteryVoltage.h" /* Header for voltage conversion functions */

/*
 * This static variable stores the previous voltage value.
 * It is used in the low-pass filter to smooth the signal.
 */
static float previousResult = 0.0f;

/**
 * @brief Initializes one GPIO pin for ADC use by setting it to analog mode.
 *
 * @param port Pointer to the GPIO port (e.g., GPIOA)
 * @param pin  Pin number to be configured (e.g., PIN0)
 */
void InitADCGPIOPin(GPIO_TypeDef *port, PIN_NUM_t pin)
{
    gpioSelectPort(port); /* Enable the clock for the specified GPIO port */
    gpioSelectPinMode(port, pin, ANALOG); /* Set the pin to analog mode */
    gpioSelectPushPullMode(port, pin, NO_PULLUP_PULLDOWN); /* Disable pull-up/down resistors */
}

/**
 * @brief Initializes all ADC channels in the configuration list.
 *        This includes setting GPIO pins and ADC sampling time.
 *
 * @param analogCh Pointer to the analogSingleCh_t configuration structure.
 *
 * @note Only single-channel operation is supported in this version.
 *       Multi-channel requires DMA and is not yet implemented.
 */
void InitADCChannels(analogSingleCh_t *analogCh)
{
    size_t i;

    for (i = 0U; i < analogCh->chnListSize; ++i)
    {
        switch (analogCh->chnList[i])
        {
            case ADC_CHN_0:
                InitADCGPIOPin(GPIOA, PIN0); /* Configure GPIO pin PA0 as analog input */
                adcSetSampleCycles(analogCh->adc, ADC_CHN_0, SAMPLE_CYCLES_480); /* Set ADC sampling time */
                break;

            /* Additional channels (ADC_CHN_1 to ADC_CHN_15) can be configured here in the same way */

            case ADC_CHN_16:
                activateTemperatureSensor(analogCh->adccommon); /* Enable internal temperature sensor */
                adcSetSampleCycles(analogCh->adc, ADC_CHN_16, SAMPLE_CYCLES_480);
                break;

            default:
                /* Unknown or unsupported ADC channel */
                break;
        }
    }
}

/**
 * @brief Initializes the ADC peripheral and its settings.
 *        This function also enables or disables interrupt-based conversion.
 *
 * @param analogCh Pointer to the analog configuration structure.
 */
void ADCInit(analogSingleCh_t *analogCh)
{
    adcSelectADC(analogCh->adc); /* Enable ADC peripheral clock */

    if (analogCh->tempEnable != false)
    {
        /* If temperature sensor is enabled, force channel 16 */
        analogCh->chnList[0U] = ADC_CHN_16;
    }

    adcDisableADC(analogCh->adc); /* Disable ADC before configuration */

    InitADCChannels(analogCh); /* Configure the GPIOs and channels */

    adcSetChannelSequence(analogCh->adc, analogCh->chnList, analogCh->chnListSize); /* Set active channels */

    adcSetResolution(analogCh->adc, analogCh->chnResolution); /* Set the resolution (e.g., 12-bit) */

    adcEnableADC(analogCh->adc); /* Re-enable ADC after setup */

    if (analogCh->interruptEnable != false)
    {
        adcEnableInterrupt(analogCh->adc, ADC_EOC_REGULAR_GRP); /* Enable end-of-conversion interrupt */
        NVIC_EnableIRQ(ADC_IRQn); /* Enable ADC interrupt in NVIC */
    }
    else
    {
        adcDisableInterrupt(analogCh->adc, ADC_EOC_REGULAR_GRP); /* Use polling only */
    }
}

/**
 * @brief Reads one ADC value using polling (blocking wait).
 *
 * @param analogCh Pointer to configuration and result structure.
 */
void getAnalogPinValuePolling(analogSingleCh_t *analogCh)
{
    adcStartConversion(analogCh->adc); /* Start ADC conversion */

    while (!adcIsConversionFinished(analogCh->adc))
    {
        /* Wait until conversion is complete */
    }

    analogCh->chnADCValue[0] = adcGetConversionResult(analogCh->adc); /* Store raw ADC result */
}

/**
 * @brief Converts a raw ADC result into a voltage (and optionally temperature)
 *        using a digital low-pass filter.
 *
 * @param analogCh Pointer to analog channel configuration.
 */
void conversionToVoltsCelsius(analogSingleCh_t *analogCh)
{
    const float v25 = 0.76f; /* Voltage at 25°C for internal temperature sensor */
    const float avg_slope = 0.0025f; /* Slope from datasheet (V/°C) */
    const float refVoltage = analogCh->Prescaler[0] * 3.3f; /* Adjusted voltage reference */

    previousResult = analogCh->chnVolt[0]; /* Get last filtered voltage */

    /* Apply low-pass filter:
     * new_value = alpha * old_value + (1 - alpha) * new_input
     */
    switch (analogCh->chnResolution)
    {
        case ADC_RES_12BIT:
            analogCh->chnVolt[0] =
                analogCh->alpha_lowpass * previousResult +
                (1.0f - analogCh->alpha_lowpass) * (analogCh->chnADCValue[0] / 4095.0f) * refVoltage;
            break;

        case ADC_RES_10BIT:
            analogCh->chnVolt[0] =
                analogCh->alpha_lowpass * previousResult +
                (1.0f - analogCh->alpha_lowpass) * (analogCh->chnADCValue[0] / 1023.0f) * refVoltage;
            break;

        case ADC_RES_8BIT:
            analogCh->chnVolt[0] =
                analogCh->alpha_lowpass * previousResult +
                (1.0f - analogCh->alpha_lowpass) * (analogCh->chnADCValue[0] / 255.0f) * refVoltage;
            break;

        case ADC_RES_6BIT:
            analogCh->chnVolt[0] =
                analogCh->alpha_lowpass * previousResult +
                (1.0f - analogCh->alpha_lowpass) * (analogCh->chnADCValue[0] / 63.0f) * refVoltage;
            break;

        default:
            /* Invalid resolution */
            break;
    }

    /* If temperature sensor is active, convert voltage to °C */
    if (analogCh->tempEnable != false)
    {
        analogCh->tempValue = (v25 - analogCh->chnVolt[0]) / avg_slope + 25.0f;
    }
}

/**
 * @brief Combines ADC read and conversion in one step (polling).
 *
 * @param analogCh Pointer to configuration and result structure.
 */
void getVoltagePinValuePolling(analogSingleCh_t *analogCh)
{
    adcStartConversion(analogCh->adc); /* Start ADC conversion */

    while (!adcIsConversionFinished(analogCh->adc))
    {
        /* Wait until done */
    }

    analogCh->chnADCValue[0] = adcGetConversionResult(analogCh->adc); /* Store raw result */

    conversionToVoltsCelsius(analogCh); /* Convert to voltage and temperature */
}
