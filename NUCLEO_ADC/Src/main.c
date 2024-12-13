/**
 * Kap14-Einfaches ADC-Projekt
 * ===========================
 *	Anpassungen bzgl Quellcode von Jesse:
 *	- Aktivierung des Bustaktes für Port A
 *	- Register adc->SQR1 mit korrektem Wert 0 gesetzt
 *
 *
 * Am Abgriff eines Potentiometers wird eine Spannung, die zwischen 0 und 3,3V
 * liegt, abgegriffen und GPIOA/Pin 0 zugefuehrt.
 */

//#include <stm32f4xx.h>
#include <stm32f401xe.h>		//change to Board NUCLEO F401RE
#include <system_stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * Kommentar in der naechsten Zeile entfernen, wenn Sie die ADC_MCAL testen möchten.
 */
//#define MCAL_ADC

#include <mcalGPIO.h>
#ifdef MCAL_ADC
#include <mcalADC.h>
#endif

int main(void)
{

    ADC_TypeDef   *adc    = ADC1;
    uint16_t       result = 0;

    const uint16_t threshold = 4095/2;		// Half of the full scale 12Bit value

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

    }

#else       // Ende der MCAL-Version, Beginn: Direkte Registerprogrammierung

    uint16_t 	   status_conv = 1;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    		 // ADC1: Bustakt aktivieren

    //adc->CR2   = 0x0;
    adc->SQR3  &= ~ADC_SQR3_SQ1;         	// 	init first source and set for channel 0

    /* This are usefull MCU internal ADC-sources - suitable for tests and calibration routines

    ADC_Common_TypeDef *adc_com = ADC1_COMMON;
    adc->SQR3  |= ADC_SQR3_SQ1_4;           // set to Channel 16... internal Temp Sensor
    adc_com->CCR |= ADC_CCR_TSVREFE; // Set the TSVREFE bit in the ADC_CCR register to wake up the temperature sensor from power down mode

    //adc->SQR3  |= ADC_SQR3_SQ1_4 | ADC_SQR3_SQ1_0;           // set to Channel 17... internal VREF = 1,20V
	*/

    adc->SQR1  &=  ~ADC_SQR1_L;             // ... set 0 for the length of 1
    adc->CR2   |= ADC_CR2_ADON;             // ADC aktivieren

    while (1)
    {
    	adc->CR2 |= ADC_CR2_SWSTART;       // A/D-Wandlung wird gestartet
        gpioResetPin(LED_GPIO, LED_blue);	//switch LED blue on
        do
        {
          status_conv = (uint16_t)(adc->SR & ADC_SR_EOC);
        } while (status_conv == 0);   // Warten, bis Konvertierung beendet
        gpioSetPin(LED_GPIO, LED_blue);
        result = adc->DR;                  // konvertierte Daten abholen

        if (result > threshold)
        {
        	gpioSetPin(LED_GPIO, LED_red);			//LED off
        }
        else
        {
        	gpioResetPin(LED_GPIO, LED_red);		// LED On
        }
    }
#endif      // Ende: Direkte Registerprogrammierung ADC


    return 0;
}
