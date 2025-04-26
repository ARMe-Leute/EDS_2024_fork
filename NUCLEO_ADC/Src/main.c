/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Erick Schlosser
 * @brief          : Example program to read analog battery voltage with ADC.
 *                   Uses polling or interrupt-based ADC reading.
 *                   Converts to voltage and temperature (if enabled).
 *
 * @date           : April 25, 2025
 ******************************************************************************
 */
#include <stm32f401xe.h>           /* Device-specific header for STM32F401 */
#include <system_stm32f4xx.h>      /* System configuration */
#include <stm32f4xx.h>             /* General STM32F4 definitions */

#include <stdint.h>                /* Standard integer types */
#include <stdbool.h>               /* Standard boolean type */
#include <stddef.h>                /* NULL and size_t */
#include <stdlib.h>                /* Memory allocation */

#include <mcalGPIO.h>              /* GPIO abstraction layer */
#include <mcalADC.h>               /* ADC abstraction layer */

#include "BatteryVoltage.h"        /* Custom header for voltage reading logic */

#define SCB_CPACR (*(volatile uint32_t *)0xE000ED88U)
/* Register for enabling floating-point unit (FPU) access */

extern const bool timerTrigger = false;
/* Not used in this example, but due to problems in the compiler process needed*/

analogSingleCh_t *myADC = NULL;
/* Global pointer to ADC configuration struct (must be global for ISR access) */

/**
 * @brief Enable access to Floating Point Unit (FPU).
 *        Required for using floating point operations (like voltage conversion).
 */
void EnableFPU(void)
{
    SCB_CPACR |= (0xFU << 20U); /* Enable full access to CP10 and CP11 (FPU registers) */
    __DSB();                    /* Data Synchronization Barrier - ensures all memory accesses complete */
    __ISB();                    /* Instruction Synchronization Barrier - flushes CPU pipeline */
}

/**
 * @brief Interrupt Service Routine (ISR) for ADC.
 *        Only used if interrupt mode is enabled.
 */
void ADC_IRQHandler(void)
{
    if (!adcIsConversionFinished(myADC->adc))
    {
        /* If conversion is not done, return immediately */
        return;
    }

    /* Read ADC result and clear end-of-conversion flag */
    myADC->chnADCValue[0] = adcGetConversionResult(myADC->adc);

    /* Convert raw ADC value to voltage or temperature */
    conversionToVoltsCelsius(myADC);
}

/**
 * @brief Main entry point of the application.
 */
int main(void)
{
    EnableFPU(); /* Enable floating-point support */

    /* Dynamically allocate memory for single-channel ADC config */
    myADC = (analogSingleCh_t *)malloc(sizeof(analogSingleCh_t));

    /* Set up ADC parameters */

    myADC->adc = ADC1;                    /* Use ADC1 hardware */
    myADC->adccommon = ADC1_COMMON;       /* Use shared settings for ADC1 */
    myADC->chnResolution = ADC_RES_12BIT; /* Set ADC resolution to 12-bit */
    myADC->chnListSize = 1U;              /* Only one ADC channel used */
    myADC->chnList[0] = ADC_CHN_0;        /* Use channel 0 (PA0) */
    myADC->tempEnable = false;            /* Do not read internal temperature sensor */
    myADC->interruptEnable = false;       /* Use polling, not interrupt */
    myADC->alpha_lowpass = 0.7f;          /* Set filter strength (0.0 = fast, 1.0 = slow) */
    myADC->Prescaler[0] = 1.0f;           /* default: 1.0 = No voltage divider used */

    /*
     * OPTIONAL SETTINGS (Enable if needed for testing):
     * -------------------------------------------------
     * 1. Enable the internal temperature sensor:
     *    Set 'tempEnable' to true.
     *    The ADC will automatically use channel 16 and convert the result to °C.
     *
     * 2. Enable interrupt-based ADC reading instead of polling:
     *    Set 'interruptEnable' to true.
     *    An interrupt will be triggered when the ADC conversion finishes.
     *
     * IMPORTANT:
     * If both options are enabled, the interrupt handler will automatically
     * convert the temperature to degrees Celsius using the internal sensor.
     *
     * Example:
     *    myADC->tempEnable = true;
     *    myADC->interruptEnable = true;
     */

    ADCInit(myADC); /* Initialize ADC and GPIO pins */

    while (true)
    {
        if (myADC->interruptEnable == false)
        {
            /* Polling mode: read voltage in blocking mode */
            getVoltagePinValuePolling(myADC);

            /* Access the results (not printed, just stored) */
            myADC->chnVolt[0];   /* Voltage in volts */
            myADC->tempValue;    /* Temperature in °C (only if tempEnable is true) */
        }
        else
        {
            /* Interrupt mode (not used here) */
            adcStartConversion(myADC->adc); /* Start ADC manually */
            __WFI();                        /* Wait for interrupt to complete */
        }
    }

    return 0; /* Never reached */
}
