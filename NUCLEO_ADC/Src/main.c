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

#include <mcalGPIO.h>
#include <mcalADC.h>
#include <stdio.h>
#include <RotaryPushButton.h>

extern bool timerTrigger = false;  								// Necessary for the mcalTimer
static uint8_t colorcode = 0;									// Variable used for cycling through the colors
/*
GPIO_TypeDef 			*port  = GPIOA;							// Port which is used for the analog signal
PIN_NUM_t				*pin   = PIN0;							// Pin which is used for the analog signal
ADC_TypeDef				*adc   = ADC1;							// ADC which is used
ADC_RESOLUTION_t 		resolution = ADC_RES_12BIT;				// Resolution of the ADC
ADC_CHANNEL_t chnList[] = { ADC_CHN_0 , ADC_CHN_15};						// List of ADC channels in a sequence
size_t         listSize = sizeof(chnList) / sizeof(chnList[0]);	// Calculate number of channel-list elements
*/

/* Depending on the Voltage the Color of the RotaryPushButton is changed using the RotaryPushButton library */
void setColorDependingOnValue(int caseNumber){

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

void initADC(analogCh_t *configADC){
	configADC->adc = ADC1;
	configADC->chnResolution = ADC_RES_12BIT;
	configADC->chnList[0] = ADC_CHN_0;
	configADC->chnListSize = 1;
	configADC->tempEnable = 1;
}

/* Main Code of the Example Code */
int main(void) {
	analogCh_t myADC;
	initADC(&myADC);
    ADCInit(&myADC);	// Initialization of the GPIO and the ADC
    while(1)
    {
    	result[0] = adcGetConversionResult(myADC->adc);
		uint16_t raw = ;
		float tempC   = ;

		// Hier könnten Sie den Wert z. B. auf ein Display ausgeben
		// oder über UART senden, LED ansteuern etc.
		// ...
		setColorDependingOnValue(colorcode);					// Change the color depending on the colorcode value

		// Einfache Pause
		for (volatile int i=0; i<1000000; i++) { /* Warten */ }
	}
    return 0;
}
