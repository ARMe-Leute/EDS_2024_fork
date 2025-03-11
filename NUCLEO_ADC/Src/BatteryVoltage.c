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
const float vtemp25 = 1.43;
const float ratioTemp =	22.222;
uint16_t ADCTemp;

void InitADCGPIOPin(GPIO_TypeDef *port, PIN_NUM_t pin){
    gpioSelectPort(port); 									// Activate GPIO clock
	gpioSelectPinMode(port, pin, ANALOG);					// Setting the GPIO Pin to analog mode
	gpioSelectPushPullMode(port, pin, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
}

void InitADCChannels(ADC_CHANNEL_t chnList[], size_t listSize){
	for (int i = 0; i < listSize; i++) {
		switch (chnList[i]) {
		        case ADC_CHN_0:
		            // ADC_CHN_0 -> PA0
		            InitADCGPIOPin(GPIOA, PIN0);
		            break;

		        case ADC_CHN_1:
		            // ADC_CHN_1 -> PA1
		            InitADCGPIOPin(GPIOA, PIN1);
		            break;

		        case ADC_CHN_2:
		            // ADC_CHN_2 -> PA2
		            InitADCGPIOPin(GPIOA, PIN2);
		            break;

		        case ADC_CHN_3:
		            // ADC_CHN_3 -> PA3
		            InitADCGPIOPin(GPIOA, PIN3);
		            break;

		        case ADC_CHN_4:
		            // ADC_CHN_4 -> PA4
		            InitADCGPIOPin(GPIOA, PIN4);
		            break;

		        case ADC_CHN_5:
		            // ADC_CHN_5 -> PA5
		            InitADCGPIOPin(GPIOA, PIN5);
		            break;

		        case ADC_CHN_6:
		            // ADC_CHN_6 -> PA6
		            InitADCGPIOPin(GPIOA, PIN6);
		            break;

		        case ADC_CHN_7:
		            // ADC_CHN_7 -> PA7
		            InitADCGPIOPin(GPIOA, PIN7);
		            break;

		        case ADC_CHN_8:
		            // ADC_CHN_8 -> PB0
		            InitADCGPIOPin(GPIOB, PIN0);
		            break;

		        case ADC_CHN_9:
		            // ADC_CHN_9 -> PB1
		            InitADCGPIOPin(GPIOB, PIN1);
		            break;

		        case ADC_CHN_10:
		            // ADC_CHN_10 -> PC0
		            InitADCGPIOPin(GPIOC, PIN0);
		            break;

		        case ADC_CHN_11:
		            // ADC_CHN_11 -> PC1
		            InitADCGPIOPin(GPIOC, PIN1);
		            break;

		        case ADC_CHN_12:
		            // ADC_CHN_12 -> PC2
		            InitADCGPIOPin(GPIOC, PIN2);
		            break;

		        case ADC_CHN_13:
		            // ADC_CHN_13 -> PC3
		            InitADCGPIOPin(GPIOC, PIN3);
		            break;

		        case ADC_CHN_14:
		            // ADC_CHN_14 -> PC4
		            InitADCGPIOPin(GPIOC, PIN4);
		            break;

		        case ADC_CHN_15:
		            // ADC_CHN_15 -> PC5
		            InitADCGPIOPin(GPIOC, PIN5);
		            break;

		        case ADC_CHN_16:
		            // Interner Temperatur-Sensor, kein externer Pin
		        	//activateTemperatureSensor();
		            break;

		        default:
		            //printf("Unbekannter ADC-Kanal!\n");
		            break;
		    }
	}
}



/* ADC Initialization and Configuration for Polling */
void ADCInit(GPIO_TypeDef *port, PIN_NUM_t pin, ADC_TypeDef *adc, ADC_CHANNEL_t chnList[], size_t listSize, ADC_RESOLUTION_t resolution) {
    if ((port == NULL) || (adc == NULL)) {
        return; 											// Safety check if port and adc pointer have been initialized correct
    }
    InitADCChannels(chnList, listSize);
    adcSelectADC(adc); 										// Activate ADC clock
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
