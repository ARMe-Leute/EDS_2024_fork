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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <mcalGPIO.h>
#include <mcalADC.h>
#include "BatteryVoltage.h"


float previousResult = 0.0f;

/*
 * InitADCGPIOPin ist dafür da die dazugehörigen GPIO Pins als analoge Eingänge zu konfigurieren.
 */
void InitADCGPIOPin(GPIO_TypeDef *port, PIN_NUM_t pin){
    gpioSelectPort(port); 									// Activate GPIO clock
	gpioSelectPinMode(port, pin, ANALOG);					// Setting the GPIO Pin to analog mode
	gpioSelectPushPullMode(port, pin, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
}

/*
 * InitADCChannels schaut nach welche ADC Channels verwendet werden und initialisiert die dazugehörigen Pins über InitADCGPIOPin.
 * Ist schon für die MultiChannel-Variante geschrieben. Nur der Übergabe-TypeDef muss auf den MutliChannel TypeDef gewechselt werden.
 */
void InitADCChannels(analogSingleCh_t *analogCh){
	for (int i = 0; i < analogCh->chnListSize; i++) {
		switch (analogCh->chnList[i]) {
			case ADC_CHN_0:
				// ADC_CHN_0 -> PA0
				InitADCGPIOPin(GPIOA, PIN0);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_0, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_1:
				// ADC_CHN_1 -> PA1
				InitADCGPIOPin(GPIOA, PIN1);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_1, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_2:
				// ADC_CHN_2 -> PA2
				InitADCGPIOPin(GPIOA, PIN2);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_2, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_3:
				// ADC_CHN_3 -> PA3
				InitADCGPIOPin(GPIOA, PIN3);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_3, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_4:
				// ADC_CHN_4 -> PA4
				InitADCGPIOPin(GPIOA, PIN4);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_4, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_5:
				// ADC_CHN_5 -> PA5
				InitADCGPIOPin(GPIOA, PIN5);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_5, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_6:
				// ADC_CHN_6 -> PA6
				InitADCGPIOPin(GPIOA, PIN6);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_6, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_7:
				// ADC_CHN_7 -> PA7
				InitADCGPIOPin(GPIOA, PIN7);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_7, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_8:
				// ADC_CHN_8 -> PB0
				InitADCGPIOPin(GPIOB, PIN0);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_8, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_9:
				// ADC_CHN_9 -> PB1
				InitADCGPIOPin(GPIOB, PIN1);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_9, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_10:
				// ADC_CHN_10 -> PC0
				InitADCGPIOPin(GPIOC, PIN0);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_10, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_11:
				// ADC_CHN_11 -> PC1
				InitADCGPIOPin(GPIOC, PIN1);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_11, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_12:
				// ADC_CHN_12 -> PC2
				InitADCGPIOPin(GPIOC, PIN2);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_12, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_13:
				// ADC_CHN_13 -> PC3
				InitADCGPIOPin(GPIOC, PIN3);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_13, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_14:
				// ADC_CHN_14 -> PC4
				InitADCGPIOPin(GPIOC, PIN4);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_14, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_15:
				// ADC_CHN_15 -> PC5
				InitADCGPIOPin(GPIOC, PIN5);
				adcSetSampleCycles(analogCh->adc, ADC_CHN_15, SAMPLE_CYCLES_480);
				break;

			case ADC_CHN_16:
				// Interner Temperatur-Sensor, kein externer Pin
				activateTemperatureSensor(analogCh->adccommon);		// Aktivieren des internen Temperatursensors
				adcSetSampleCycles(analogCh->adc, ADC_CHN_16, SAMPLE_CYCLES_480);
				break;

			default:
				//printf("Unbekannter ADC-Kanal!\n");
				break;
		}
	}
}


/* ADC Initialization and Configuration for Polling */
void ADCInit(analogSingleCh_t *analogCh) {
	adcSelectADC(analogCh->adc); 		// Activate ADC clock
	// Falls der tempEnable auf True gesetzt ist, soll im SingleChannelMode nur die Temperatur ausgewertet werden.
    if (analogCh->tempEnable)
    {
    	/* Für SingleChannel: */
    	analogCh->chnList[0] = ADC_CHN_16;			// Channel 16 (interner Temperatursensor) als einzigen Kanal setzen
    	/* Für MultiChannel:
    	 * analogCh->chnListSize = analogCh->chnListSize +1;			// Listengröße um 1 erhöhen (weil Kanal 16 hinzugefügt wird)
    	 * analogCh->chnList[analogCh->chnListSize -1] = ADC_CHN_16 ; 	// Kanal 16 in die Kanalliste einfügen
    	 */
    }
    adcDisableADC(analogCh->adc);		// Disable the ADC to configure it
    InitADCChannels(analogCh);			// ADC Kanäle konfigurieren
    adcSetChannelSequence(analogCh->adc, analogCh->chnList, analogCh->chnListSize);			// Select the ADC-Channel
    adcSetResolution(analogCh->adc, analogCh->chnResolution); // Select the ADC Resolution
    adcEnableADC(analogCh->adc);										// Enable the ADC again after configuration
    if (analogCh->interruptEnable == 1)
    {
    	adcEnableInterrupt(analogCh->adc, ADC_EOC_REGULAR_GRP); // Aktivierung des Interrupts für die End-of-conversion flag
    	NVIC_EnableIRQ(ADC_IRQn);
    }
    else
    {
    	adcDisableInterrupt(analogCh->adc, ADC_EOC_REGULAR_GRP); // Deaktivieren des Interrupts
    }
}

/* Reading the ADC Value with Polling and writing the ADC Value (0-4095 at 12bit) into analogCh*/
void getAnalogPinValuePolling(analogSingleCh_t *analogCh) {
    adcStartConversion(analogCh->adc);
    while (!adcIsConversionFinished(analogCh->adc)) {
        ; //waiting for the adc conversion to be finished
    }
    analogCh->chnADCValue[0] = adcGetConversionResult(analogCh->adc);
}
/*
 * Converting to Volts and Celsius with a lowpass-filter
 */
void conversionToVoltsCelsius(analogSingleCh_t *analogCh){
	const float v25 = 0.76f; 			// Voltage at 25°C
	const float avg_slope = 0.0025f;	// Volt per °C
	const float refVoltage = (analogCh->Prescaler[0]) * 3.3f;
	previousResult = analogCh->chnVolt[0];
	switch (analogCh->chnResolution)
	{
		/* calculating and writing the result of the lowpass-filter in volts into the typedef
		*  Calculation of Lowpass-Filter:
		*	y[n] = α*y[n-1] + (1-α)*x[n]
		*/
		case ADC_RES_12BIT:
			analogCh->chnVolt[0] = analogCh->alpha_lowpass * previousResult + (1.0f - analogCh->alpha_lowpass) * analogCh->chnADCValue[0]/4095.0f * refVoltage;
			break;
		case ADC_RES_10BIT:
			analogCh->chnVolt[0] = analogCh->alpha_lowpass * previousResult + (1.0f - analogCh->alpha_lowpass) * analogCh->chnADCValue[0]/1023.0f * refVoltage;
			break;
		case ADC_RES_8BIT:
			analogCh->chnVolt[0] = analogCh->alpha_lowpass * previousResult + (1.0f - analogCh->alpha_lowpass) * analogCh->chnADCValue[0]/255.0f * refVoltage;
			break;
		case ADC_RES_6BIT:
			analogCh->chnVolt[0] = analogCh->alpha_lowpass * previousResult + (1.0f - analogCh->alpha_lowpass) * analogCh->chnADCValue[0]/63.0f * refVoltage;
			break;
	}
	// Converting the voltage to the temperature in °C if tempEnable is set to true
	if (analogCh->tempEnable == 1)
	{
		analogCh->tempValue = (v25 - analogCh->chnVolt[0]) / avg_slope + 25.0f;
	}
}

/* Reading the ADC Value with Polling and writing it into analogCh*/
void getVoltagePinValuePolling(analogSingleCh_t *analogCh){
	adcStartConversion(analogCh->adc);
	while (!adcIsConversionFinished(analogCh->adc))
	{
		; //waiting for the adc conversion to be finished
	}
	analogCh->chnADCValue[0] = adcGetConversionResult(analogCh->adc);
	conversionToVoltsCelsius(analogCh);
}

