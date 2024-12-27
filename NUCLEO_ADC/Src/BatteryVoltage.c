/**
 ******************************************************************************
 * @file           : ReadOutBatteryVoltage.c
 * @author         : Erick Schlosser
 * @brief          : This is the C source code for reading out the Battery Voltage.
 *              	 It includes the implementations of the functions defined in "RotaryPushButton.h."
 *              	 Additionally, this source code contains the implementation of the interrupt functions.
 *              	 This library is based on the CMSIS and MCAL library.
 * @date		   : December 27, 2023
 ******************************************************************************
 */
#include <BatteryVoltage.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <mcalGPIO.h>
#include <mcalADC.h>

/* Global Variables */
static uint16_t batteryVoltageMilliVolts = 0;
static uint16_t ADCValue = 0;

/* ADC Initialization and Configuration for Polling */
void ADCInit(GPIO_TypeDef *port, PIN_NUM_t pin, ADC_TypeDef *adc, ADC_CHANNEL_t chnList[], size_t listSize, ADC_RESOLUTION_t resolution) {
    if ((port == NULL) || (adc == NULL)) {
        return; 											// Safety check if port and adc pointer have been initialized correct
    }
    gpioSelectPort(port); 									// Activate GPIO clock
    adcSelectADC(adc); 										// Activate ADC clock
    gpioSelectPinMode(port, pin, ANALOG);					// Setting the GPIO Pin to analog mode
    gpioSelectPushPullMode(port, pin, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
    adcDisableADC(adc);										// Disable the ADC to configure it
    adcSetChannelSequence(adc, chnList, listSize);			// Select the ADC-Channel
    adcSetResolution(adc, resolution);						// Select the ADC Resolution
    adcEnableADC(adc);										// Enable the ADC again after configuration
}

/* Reading the ADC Value with Polling and returning the calculated Voltage*/
uint16_t getBatteryMilliVolts(ADC_TypeDef *adc) {
    if (adc == NULL) {
        return 0; // Safety check
    }
    adcStartConversion(adc);
    while (!adcIsConversionFinished(adc)) {
        ; //waiting for the adc conversion to be finished
    }
    ADCValue = adcGetConversionResult(adc);
    batteryVoltageMilliVolts = ADCValue * BATTERY_VOLTAGE_COEFFICIENT;
    return batteryVoltageMilliVolts;
}
/* end of BatteryVoltage.c */
