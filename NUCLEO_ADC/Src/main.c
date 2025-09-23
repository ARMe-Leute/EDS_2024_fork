/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Erick Schlosser
 * @brief          : This is an example code for using the BatteryVoltage.c library.
 *              	 This code is based on the RotaryPushButton, CMSIS and MCAL library.
 * @date		   : December 02, 2024
 ******************************************************************************
 */

#include <BatteryVoltage.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * Kommentar in der naechsten Zeile entfernen, wenn Sie die ADC_MCAL testen möchten.
 */
#define MCAL_ADC

#include <mcalGPIO.h>
#include <mcalADC.h>
#include <stdio.h>

extern bool timerTrigger = false;  								// Necessary for the mcalTimer
static uint8_t colorcode = 0;									// Variable used for cycling through the colors

GPIO_TypeDef 			*port  = GPIOA;							// Port which is used for the analog signal
PIN_NUM_t				*pin   = PIN0;							// Pin which is used for the analog signal
ADC_TypeDef				*adc   = ADC1;							// ADC which is used
ADC_RESOLUTION_t 		resolution = ADC_RES_12BIT;				// Resolution of the ADC
ADC_CHANNEL_t chnList[] = { ADC_CHN_0 };						// List of ADC channels in a sequence
size_t         listSize = sizeof(chnList) / sizeof(chnList[0]);	// Calculate number of channel-list elements

<<<<<<< HEAD
/* Depending on the Voltage the Color of the RotaryPushButton is changed using the RotaryPushButton library */
void setColorDependingOnValue(int caseNumber){
=======
    const uint16_t threshold = 4096/4;		// 1/4 of the full scale 12Bit value

#define 	LED_GPIO		GPIOB
#define 	LED_red			PIN3
#define		LED_green		PIN4
#define		LED_blue		PIN10
#define 	Bit_red			(unsigned int) 1
#define		Bit_blue		(unsigned int) 2
#define		Bit_green		(unsigned int) 4
    gpioSelectPort(LED_GPIO);
    gpioSelectPinMode(LED_GPIO, LED_red, OUTPUT);
    gpioSelectPinMode(LED_GPIO, LED_green, OUTPUT);
    gpioSelectPinMode(LED_GPIO, LED_blue, OUTPUT);

	gpioSetPin(LED_GPIO, LED_red);	 // switch off LED red
	gpioSetPin(LED_GPIO, LED_green);  // switch off LED green

#define 	ADC_GPIO		GPIOA
#define 	PA0				PIN0
    gpioSelectPort(ADC_GPIO);
    gpioSelectPinMode(ADC_GPIO, PA0, ANALOG); // PA0 : Analog-Modus

#ifdef MCAL_ADC     // Beginn der MCAL-Version

    // Liste der ADC-Kanaele in einer Sequenz
    ADC_CHANNEL_t chnList[] = { ADC_CHN_0 };

    // Anzahl der Listenelemente berechnen
    size_t         listSize = sizeof(chnList) / sizeof(chnList[0]);

    adcSelectADC(adc);                     // ADC1: Bustakt aktivieren

    // Konfiguration der Sequenz und Eintrag der Laenge von chnList[]
    adcSetChannelSequence(adc, chnList, listSize);
    adcEnableADC(adc);

    while (1)
    {
        adcStartConversion(adc);
        gpioResetPin(LED_GPIO, LED_blue);	//switch LED blue on
        while (!(adcIsConversionFinished(adc)))
        {
            ;
        }
        gpioSetPin(LED_GPIO, LED_blue);	//switch LED blue off
        result = adcGetConversionResult(adc);

        if (result > threshold)
              {
              	gpioSetPin(LED_GPIO, LED_red);			//LED off
              }
              else
              {
              	gpioResetPin(LED_GPIO, LED_red);		// LED On
              }
>>>>>>> b87c7c8006f51f0097ae241f20d834498e69beb3

    switch (caseNumber) {
        case 0:
            setRotaryColor(LED_BLACK);
            break;
        case 1:
            setRotaryColor(LED_WHITE);
            break;
        case 2:
            setRotaryColor(LED_RED);
            break;
        case 3:
            setRotaryColor(LED_GREEN);
            break;
        case 4:
            setRotaryColor(LED_BLUE);
            break;
        case 5:
            setRotaryColor(LED_MAGENTA);
            break;
        case 6:
            setRotaryColor(LED_CYAN);
            break;
        case 7:
            setRotaryColor(LED_YELLOW);
            break;
        case 8:
            setRotaryColor(LED_WHITE);
            break;
        case 9:
            setRotaryColor(LED_RED); // Beispiel für Wiederholung
            break;
        case 10:
            setRotaryColor(LED_GREEN); // Beispiel für Wiederholung
            break;
        default:
            printf("Invalid case number!\n");
            break;
    }
}

/* Main Code of the Example Code */
int main(void) {
    ADCInit(port, *pin, adc, chnList, listSize, resolution);	// Initialization of the GPIO and the ADC
    while (1) {
        colorcode = getBatteryMilliVolts(adc) * 0.003333;			// receiving the voltage and expanding it to the 11 cases of setColorDependingOnValue()
        setColorDependingOnValue(colorcode);					// Change the color depending on the colorcode value
    }
}
