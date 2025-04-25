#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stm32f4xx.h>

// MCAL und CMSIS
#include <mcalGPIO.h>
#include <mcalADC.h>

#define SCB_CPACR  (*(volatile uint32_t *)0xE000ED88U)


ADC_TypeDef *adc = ADC1;
ADC_Common_TypeDef *adcccr=ADC1_COMMON;
ADC_CHANNEL_t chnList[] = {ADC_CHN_16};
size_t seqLen = 1;

void EnableFPU(void)
{
    // CP10 und CP11 auf Full Access setzen (Bits 20..23 = 0xF)
    SCB_CPACR |= (0xF << 20);

    // Daten-/Instruktions-Synchronisierung
    __DSB();
    __ISB();
}

/*****************************************************************************/
/* Funktion:  ADC1_Init                                                      */
/* Zweck:     Schaltet ADC1-Takt frei, aktiviert Temp-Sensor und richtet     */
/*            ADC1 für Single Conversion auf Channel 16 ein.                 */
/*****************************************************************************/
void ADC1_Init(void)
{


    // 1) ADC1-Takt aktivieren
    //RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    adcSelectADC(adc);

    // 2) Temperatur-Sensor in ADC_CCR aktivieren
    //ADC_CCR |= ADC_CCR_TSVREFE;
    activateTemperatureSensor(adcccr);

    // 3) ADC1 deaktivieren, bevor wir CR1/CR2 einstellen (sicherheitshalber)
    //ADC1_CR2 &= ~ADC_CR2_ADON;
    adcDisableADC(adc);

    // 4) Sample-Time für Kanal 16 konfigurieren
    //    Kanal 16 wird in SMPR1[ (16-10)*3 ] = SMPR1[18..20 Bits] gesetzt
    //    7 (111b) in diesem Feld => 480 Zyklen.
    //    (je nach Referenzmanual: Bits für CH16 sind an Position 18..20 in SMPR1).
    //ADC1_SMPR1 &= ~(7UL << (3 * (16 - 10)));    // vorher nullen
    //ADC1_SMPR1 |=  (7UL << (3 * (16 - 10)));    // 111 => 480 cycles
    adcSetSampleCycles(adc, ADC_CHN_16, SAMPLE_CYCLES_480);

    // 5) Kanal 16 als einzige Conversion in der Regular Sequence SQR3
    //    Die untersten 5 Bits in SQR3 wählen den Kanal aus.
    //ADC1_SQR3 = 16UL;  // => 16 & 0x1F
    adcSetChannelSequence(adc, chnList, seqLen);

    // 6) Sequenz-Länge = 1
    //    SQR1[L[3:0]] => 0 => 1 Conversion
    //ADC1_SQR1 = 0; // L=0 => 1 Conversion
    //adcSetChannelSequence(adc, chnList, seqLen);

    // 7) ADC1 einschalten (ADON-Bit)
    //ADC1_CR2 |= ADC_CR2_ADON;
    adcEnableADC(adc);
}

/*****************************************************************************/
/* Funktion:  ADC1_ReadTemperatureRaw                                       */
/* Zweck:     Startet eine Single-Conversion auf CH16 und liefert ADC-Wert.  */
/*****************************************************************************/
uint16_t ADC1_ReadTemperatureRaw(void)
{
    // 1) Start der Software-Conversion
    //ADC1_CR2 |= ADC_CR2_SWSTART;
    adcStartConversion(adc);

    // 2) Warten auf End-of-Conversion (EOC)
    while (adcIsConversionFinished(adc) == 0) {
    }

    // 3) ADC-Daten aus dem Data-Register auslesen
    //uint16_t adcValue = (uint16_t)(ADC1_DR & 0xFFFF);
    uint16_t adcValue = adcGetConversionResult(adc);

    return adcValue;
}

/*****************************************************************************/
/* Funktion:  ConvertTempSensorValueToC                                      */
/* Zweck:     Rechnet den ADC-Rohwert in Grad Celsius um.                    */
/*****************************************************************************/
float ConvertTempSensorValueToC(uint16_t adcValue)
{
    // Rechne ADC-Wert in Spannung um (bei 12-bit ADC: max 4095)
    // Annahme Vref = 3.3V (ggf. anpassen):
    float vsense = ((float)adcValue * 3.3f) / 4095.0f;

    // Typische Kennwerte (Datenblatt STM32F4):
    // V25 ~ 0,76V, Avg_Slope ~ 2,5mV/°C => 0,0025V/°C
    float v25        = 0.76f;      // Spannung bei 25°C
    float avg_slope  = 0.0025f;    // Spannung pro °C
    //zu const variablen verändern, damit nur in ROM gespeichert wird und nicht zusätzlich auch im RAM

    // T(°C) = (V25 - Vsense) / Avg_Slope + 25
    // Achtung: Je nach Datenblatt kann die Formel leicht variieren.
    float temperature = (v25 - vsense) / avg_slope + 25.0f;

    return temperature;
}
int main(void)
{
    // 1) System-Clock initialisieren, falls nötig (hier nur angedeutet)
    //    (PLL einstellen, Flash Waitstates, ...)
    //    -> An dieser Stelle müsste Ihr eigener Code stehen.

    // 2) GPIOs ggf. initialisieren (z. B. für LED-Ausgabe oder UART).

    // 3) ADC initialisieren
	EnableFPU();
    ADC1_Init();

    // 4) Endlosschleife
    while(1)
    {
        uint16_t raw = ADC1_ReadTemperatureRaw();
        float tempC   = ConvertTempSensorValueToC(raw);

        // Einfache Pause
        for (volatile int i=0; i<1000000; i++) { /* Warten */ }
    }

    return 0;
}
