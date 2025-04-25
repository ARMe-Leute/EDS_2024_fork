#include <stm32f401xe.h>
#include <system_stm32f4xx.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <mcalGPIO.h>
#include <mcalADC.h>

#include "BatteryVoltage.h"
#define SCB_CPACR  (*(volatile uint32_t *)0xE000ED88U)


const bool timerTrigger;
analogSingleCh_t *myADC = NULL;  // globale Definition (sichtbar für ISR)
/*****************************************************************************/
/* Funktion:  EnableFPU                                                      */
/*****************************************************************************/
void EnableFPU(void)
{
    // CP10 und CP11 auf Full Access setzen (Bits 20..23 = 0xF)
    SCB_CPACR |= (0xF << 20);

    // Daten-/Instruktions-Synchronisierung
    __DSB();
    __ISB();
}

void ADC_IRQHandler(void)
{
    // Sofort raus, wenn EOC nicht aktiv ist → ISR tut dann gar nichts
    if (!adcIsConversionFinished(myADC->adc))
        return;

    // Konvertierungswert lesen → löscht automatisch EOC-Flag
    myADC->chnADCValue[0] = adcGetConversionResult(myADC->adc);
    conversionToVoltsCelsius(myADC);
}

int main(void)
{
	EnableFPU();
	myADC = (analogSingleCh_t *)malloc(sizeof(analogSingleCh_t));
	myADC->adc = ADC1;
	myADC->adccommon = ADC1_COMMON;
	myADC->chnResolution = ADC_RES_12BIT;
	myADC->chnListSize = 1;
	myADC->chnList[0] = ADC_CHN_0;
	myADC->tempEnable = 0;
	myADC->interruptEnable = 0;
	myADC->alpha_lowpass = 0.7f;
	myADC->Prescaler[0] = 1;

	ADCInit(myADC);
    while(1)
    {
    	// Polling Betrieb
    	if(myADC->interruptEnable == 0)
    	{
    		getVoltagePinValuePolling(myADC);
    		myADC->chnVolt[0];
    		myADC->tempValue;
    	}
    	// Interrupt Betrieb
    	else
    	{
    		adcStartConversion(myADC->adc);
    		__WFI();
    	}

    }
    return 0;
}
