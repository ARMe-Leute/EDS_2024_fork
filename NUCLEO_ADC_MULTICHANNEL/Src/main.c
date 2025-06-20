#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stm32f4xx.h>
#include <mcalADC.h>
#include <mcalGPIO.h>

#define NUM_CHANNELS 3
ADC_TypeDef *adc = ADC1;

// Hier werden nachher die Resultate aus den einzelnen Kanälen gespeichert
uint16_t g_adcValues[NUM_CHANNELS];

static ADC_CHANNEL_t g_channelList[NUM_CHANNELS] =
{
    ADC_CHN_0,  // z.B. PA0
    ADC_CHN_1,  // z.B. PA1
    ADC_CHN_2   // z.B. PA2
};

static void ADC1_Setup(void)
{
    // 1) ADC-Takt einschalten
    adcSelectADC(adc);

    // 2) ADC ganz einschalten => ADON-Bit
    adcEnableADC(adc);

    // 3) Auflösung setzen (z.B. 12 Bit)
    adcSetResolution(adc, ADC_RES_12BIT);

    // 4) Scan Mode und Continuous Mode in den Steuerregistern CR1/CR2:
    //    - SCAN = 1 in CR1
    //    - CONT = 1 in CR2
    //adc->CR1 |= ADC_CR1_SCAN;    // Bit 8
    //adc->CR2 |= ADC_CR2_CONT;    // Bit 1

    // 5) Sample-Zeit für jeden Kanal (ggf. je nach Quelle anpassen)
    //    Beispiel: 15 ADC-Zyklen pro Kanal
    adcSetSampleCycles(adc, ADC_CHN_0, SAMPLE_CYCLES_480);
    adcSetSampleCycles(adc, ADC_CHN_1, SAMPLE_CYCLES_480);
    adcSetSampleCycles(adc, ADC_CHN_2, SAMPLE_CYCLES_480);

    // 6) Reihenfolge festlegen (Sequence):
    adcSetChannelSequence(adc, g_channelList, NUM_CHANNELS);


    // 8) Den ersten Schwung Messungen starten:
    adcStartConversion(adc);
}

int main(void)
{
    // Hier eventuell erst SystemClock_Config(), falls noch nicht geschehen
    // ...
    gpioSelectPort(GPIOA); 									// Activate GPIO clock
	gpioSelectPinMode(GPIOA, 0, ANALOG);					// Setting the GPIO Pin to analog mode
	gpioSelectPushPullMode(GPIOA, 0, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
	gpioSelectPinMode(GPIOA, 1, ANALOG);					// Setting the GPIO Pin to analog mode
	gpioSelectPushPullMode(GPIOA, 1, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
	gpioSelectPinMode(GPIOA, 2, ANALOG);					// Setting the GPIO Pin to analog mode
	gpioSelectPushPullMode(GPIOA, 2, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
    // ADC initialisieren
    ADC1_Setup();

    while(1)
    {
        // Warten, bis eine komplette Sequenz fertig ist
        // -> Mit N Kanälen in Scan+Continuous Mode wird das EOC-Bit
        //    nach JEDEM Kanal gesetzt (default: EOCS=0 => erst am Ende der Sequenz).
        //    Leider variiert das von STM32 zu STM32.
        //    Prüfe ggf. CR2-Bit EOCS (Bit 10), ob du End-of-Sequence oder End-of-Conversion pro Kanal haben willst.
        //
        //    Im einfachsten Fall:
        while (!adcIsConversionFinished(adc))
        {
            // Warten auf EOC
        }

        // DR enthält das Resultat des letzten Kanals der Sequenz.
        // Für Multi-Channel-Scan spult STM die Kanäle durch.
        // Man kann die einzelnen Werte NICHT direkt nacheinander aus DR abholen,
        // weil jeder Kanal jeweils kurz vor EOC da reingeschrieben wird.
        //
        // Einfache Variante: Wir gehen davon aus, dass man nur den letzten Wert hat.
        for (uint32_t i = 0; i < NUM_CHANNELS; i++)
        {
            // Hier vereinfachen wir:
            //   - wir lesen DR (damit wir den i-ten Kanal "verbrauchen")
            //   - dann warten wir bis "wieder EOC"
            g_adcValues[i] = adcGetConversionResult(adc);
            adcStartConversion(adc);
            // EOC-Flag ist jetzt weg, also warte erneut
            if (i < (NUM_CHANNELS - 1))
            {
               while (!adcIsConversionFinished(adc));
            }
        }

        // -> g_adcValues[0], g_adcValues[1], g_adcValues[2]
        //    enthalten jetzt die Messungen

        // Nächste Schleife, er misst ja in CONT=1 endlos
        // ...
    }
}
