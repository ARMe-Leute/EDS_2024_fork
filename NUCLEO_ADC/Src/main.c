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
void adcActivate(void)
{
    GPIO_TypeDef 			*port  = GPIOA;							// Port which is used for the analog signal
#ifdef BALA2024
    PIN_NUM_t				pinA1   = PIN1;							// Pin which is used for the analog signal
#else
    PIN_NUM_t				pinA1   = PIN0;							// Pin which is used for the analog signal
#endif

    ADC_TypeDef				*adc   = ADC1;							// ADC which is used
    ADC_RESOLUTION_t 		resolution = ADC_RES_12BIT;				// Resolution of the ADC
    //ADC_CHN_16,				TempSensor
    ADC_CHANNEL_t chnList[] = { ADC_CHN_1};				// List of ADC channels PA1 BatteryVoltage, Intern CPU Temperature

    size_t         listSize = sizeof(chnList) / sizeof(chnList[0]);	// Calculate number of channel-list elements

    ADCInit(port, pinA1, adc, chnList, listSize, resolution);	// Initialization of the GPIO and the ADC
    adcStartConversion(adc);
}

/* Reading the ADC Value, restart conversion and returning the calculated Voltage*/
BatStat_t getBatVolt(analogCh_t* pADChn)
{
    const float ratioCh1 = 0.00888;  // coefficient for the ratio 11:1 (R1+R2)/R1 multiplied with 0.00080586 R1 =1k; R2 =10k
   /*
    * Variable for IC Temp measurement
    *  const float ratioCh16 = 0.00080586;
    const float vtemp25 = 1.43;
    const float ratioTemp =	22.222;
    uint16_t ADCTemp;
   */
    uint16_t ADCValue;
	if (pADChn->adc == NULL) {
        return 0; // Safety check
    }


	if (!adcIsConversionFinished(pADChn->adc))
   	{
    	adcStartConversion(pADChn->adc);
	}
    else
    {
       	ADCValue = adcGetConversionResult(pADChn->adc);
    	//ADCTemp = adcGetConversionResult(adc);
    	adcStartConversion(pADChn->adc);
    }
	//float cpuTemp = ratioTemp*(vtemp25-(float)ADCTemp*ratioCh16) + 25; // temp = (1.43 - Vtemp) / 4.5 + 25.00
    pADChn->BatVolt = ADCValue * ratioCh1;

    if (pADChn->BatVolt > halfBatVolt)
    {
    	pADChn->BatStatus=okBat;
    }
    else
    {
    	if (pADChn->BatVolt > emptyBatVolt)
    	{ pADChn->BatStatus=halfBat; }
    	else
    	{ pADChn->BatStatus=emptyBat; }
    }
    return pADChn->BatStatus;
}
/* end of BatteryVoltage Stuff from BALO main */

/* Main Code of the Example Code */
int main(void) {
    ADCInit(port, *pin, adc, chnList, listSize, resolution);	// Initialization of the GPIO and the ADC
    while (1) {
        colorcode = getBatteryMilliVolts(adc) * 0.003333;			// receiving the voltage and expanding it to the 11 cases of setColorDependingOnValue()
        setColorDependingOnValue(colorcode);					// Change the color depending on the colorcode value
        getBatVolt();
    }
}
