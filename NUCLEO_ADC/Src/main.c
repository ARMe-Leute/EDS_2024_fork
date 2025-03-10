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

GPIO_TypeDef 			*port  = GPIOA;							// Port which is used for the analog signal
PIN_NUM_t				*pin   = PIN0;							// Pin which is used for the analog signal
ADC_TypeDef				*adc   = ADC1;							// ADC which is used
ADC_RESOLUTION_t 		resolution = ADC_RES_12BIT;				// Resolution of the ADC
ADC_CHANNEL_t chnList[] = { ADC_CHN_0 };						// List of ADC channels in a sequence
size_t         listSize = sizeof(chnList) / sizeof(chnList[0]);	// Calculate number of channel-list elements

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


/* Main Code of the Example Code */
int main(void) {
    ADCInit(port, *pin, adc, chnList, listSize, resolution);	// Initialization of the GPIO and the ADC
    while (1) {
        colorcode = getBatteryMilliVolts(adc) * 0.003333;			// receiving the voltage and expanding it to the 11 cases of setColorDependingOnValue()
        setColorDependingOnValue(colorcode);					// Change the color depending on the colorcode value
    }
}
