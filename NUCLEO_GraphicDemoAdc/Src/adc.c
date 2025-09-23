/****************************************************************************************
*
* Hardware configuration and low level functions
*
*
****************************************************************************************/

#include <stdio.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <mcalADC.h>
#include <ST7735.h>
#include "../Inc/adc.h"







analogCh_t analogCh;




// Init hardware

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



void activateADC(PIN_NUM_t pinA1)
{
    GPIO_TypeDef 			*port  = GPIOA;							// Port which is used for the analog signal

//    PIN_NUM_t				pinA1   = PIN1;	 //! Pin1 used for Balancer2024 BATTERY-Volatge
//    PIN_NUM_t				pinA1   = PIN0;	 //! Pin which is used for the analog signal


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
/* end of BatteryVoltage.c */

