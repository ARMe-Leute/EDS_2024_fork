#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stm32f4xx.h>

// MCAL und CMSIS
#include <mcalGPIO.h>
#include <mcalADC.h>

#define SCB_CPACR  (*(volatile uint32_t *)0xE000ED88U)

/* ADC Setup */
ADC_TypeDef *adc          = ADC1;
ADC_Common_TypeDef *adcccr=ADC1_COMMON;
ADC_Common_TypeDef *adc_C = ADC1;
ADC_CHANNEL_t chnList[]   = {ADC_CHN_16};
size_t seqLen             = 1;

/* Hier speichern wir Messwerte und gefilterten Wert */
volatile uint16_t g_adcValue = 0;
volatile float    g_tempC    = 0.0f;
volatile float    g_tempC_filt = 0.0f;  // gefilterter Temperaturwert

/* Tiefpass-Parameter */
#define ALPHA (0.7f)  // z.B. 0.7 – anpassen nach Bedarf - nicht als globale Variable

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

/*****************************************************************************/
/* Funktion:  ConvertTempSensorValueToC                                      */
/* Zweck:     Rechnet den ADC-Rohwert in Grad Celsius um.                    */
/*****************************************************************************/
float ConvertTempSensorValueToC(uint16_t adcValue)
{
    float vsense = ((float)adcValue * 3.3f) / 4095.0f;
    float v25       = 0.76f;     // Spannung bei 25°C
    float avg_slope = 0.0025f;   // Spannung pro °C
    float temperature = (v25 - vsense) / avg_slope + 25.0f;
    return temperature;
}

/*****************************************************************************/
/* Funktion:  ADC1_Init                                                      */
/* Zweck:     Initialisiert ADC1 für kontinuierlichen Betrieb mit Interrupt. */
/*****************************************************************************/
void ADC1_Init(void)
{
    /* MCAL-Funktion zum ADC auswählen */
    adcSelectADC(adc);

    /* Temperatursensor aktivieren */
    activateTemperatureSensor(adcccr);

    /* ADC ausschalten, um Einstellungen ändern zu können */
    adcDisableADC(adc);

    /* Sampling-Zeit einstellen */
    adcSetSampleCycles(adc, ADC_CHN_16, SAMPLE_CYCLES_480);

    /* Kanal-Sequenz konfigurieren (nur Kanal 16, TempSensor) */
    adcSetChannelSequence(adc, chnList, seqLen);

    /* ADC wieder einschalten */
    adcEnableADC(adc);


    /* End-of-Conversion-Interrupt einschalten (CR1:EOCIE) */
    adcEnableInterrupt(adc, ADC_EOC_REGULAR_GRP);

    /* ADC-Interrupt im NVIC aktivieren */
    NVIC_EnableIRQ(ADC_IRQn);
}

/*****************************************************************************/
/* Funktion:  ADC_IRQHandler                                                 */
/* Zweck:     Liest ADC aus und aktualisiert globalen Mess- & Filterwert.    */
/*****************************************************************************/
void ADC_IRQHandler(void)
{
    /* Prüfen, ob wirklich der End-of-Conversion-Interrupt anliegt */
    if (adcIsConversionFinished(adc))
    {
        /* Lesen des Conversion-Result → löscht auch das EOC-Flag */
        uint16_t raw = (uint16_t)(adcGetConversionResult(adc) & 0xFFFF);

        /* Aktuelle Temperatur (ungeregelt) */
        g_adcValue = raw;
        g_tempC    = ConvertTempSensorValueToC(raw);

        /* Tiefpass-Filter anwenden:
           Alte Schreibweise: y[n] = α*y[n-1] + (1-α)*x[n] */
        g_tempC_filt = ALPHA * g_tempC_filt + (1.0f - ALPHA) * g_tempC;
    }
}

int main(void)
{
    EnableFPU();
    ADC1_Init();

    while(1)
    {
    	adcStartConversion(adc);
        /* Warten auf Interrupt (oder andere Aufgaben tun) */
        __WFI();
    }
    return 0;
}
