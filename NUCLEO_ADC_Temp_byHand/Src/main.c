//#include <stm32f446xx.h>
#include <stm32f401xe.h>
#include <system_stm32f4xx.h>

/*****************************************************************************/
/* Wichtige Registerdefinitionen und Bitmasken für STM32F4 (Auswahl/Beispiel)*/
/*****************************************************************************/

// (Beispiel-Adressen aus STM32F407/STM32F4xx; bitte für Ihr Modell prüfen)
#define PERIPH_BASE       (0x40000000UL)
#define AHB1PERIPH_BASE   (PERIPH_BASE + 0x00020000UL)
#define APB2PERIPH_BASE   (PERIPH_BASE + 0x00010000UL)
#define ADC1_BASE         (APB2PERIPH_BASE + 0x2000UL)
#define ADC_COMMON_BASE   (APB2PERIPH_BASE + 0x2300UL) // "ADC_CCR" liegt im Common-Bereich

// --- RCC (Reset and Clock Control) ---
#define RCC_BASE               (AHB1PERIPH_BASE + 0x3800UL)
#define RCC_APB2ENR_OFFSET     0x44
#define RCC_APB2ENR            (*((volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET)))

#define RCC_APB2ENR_ADC1EN     (1UL << 8)  // Bit 8: ADC1-Klock enable

// --- ADC Common Register ---
#define ADC_CCR        (*((volatile uint32_t *)(ADC_COMMON_BASE + 0x04)))
// Im Referenzmanual oft ADC->CCR genannt; Offset 0x04 ab ADC_COMMON_BASE.

// Bitmasken für ADC_CCR
#define ADC_CCR_TSVREFE   (1UL << 23)  // Temperature sensor und V_REFINT einschalten

// --- ADC1 einzelne Register (hier stark vereinfacht) ---
#define ADC1_SR      (*((volatile uint32_t *)(ADC1_BASE + 0x00))) // Status register
#define ADC1_CR1     (*((volatile uint32_t *)(ADC1_BASE + 0x04))) // Control register 1
#define ADC1_CR2     (*((volatile uint32_t *)(ADC1_BASE + 0x08))) // Control register 2
#define ADC1_SMPR1   (*((volatile uint32_t *)(ADC1_BASE + 0x0C))) // Sample time register 1
#define ADC1_SMPR2   (*((volatile uint32_t *)(ADC1_BASE + 0x10))) // Sample time register 2
#define ADC1_SQR1    (*((volatile uint32_t *)(ADC1_BASE + 0x2C))) // Regular seq register 1
#define ADC1_SQR2    (*((volatile uint32_t *)(ADC1_BASE + 0x30))) // Regular seq register 2
#define ADC1_SQR3    (*((volatile uint32_t *)(ADC1_BASE + 0x34))) // Regular seq register 3
#define ADC1_DR      (*((volatile uint32_t *)(ADC1_BASE + 0x4C))) // Data register

// --- ADC1_SR Bits ---
#define ADC_SR_EOC     (1UL << 1)   // End of conversion

// --- ADC1_CR2 Bits ---
#define ADC_CR2_ADON   (1UL << 0)   // ADC einschalten
#define ADC_CR2_SWSTART (1UL << 30) // Software trigger start
// ... ggf. weitere Bits wie CONT für Continuous Mode, ALIGN, DMA etc.

#define SCB_CPACR  (*(volatile uint32_t *)0xE000ED88U)

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
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 2) Temperatur-Sensor in ADC_CCR aktivieren
    ADC_CCR |= ADC_CCR_TSVREFE;

    // 3) ADC1 deaktivieren, bevor wir CR1/CR2 einstellen (sicherheitshalber)
    ADC1_CR2 &= ~ADC_CR2_ADON;

    // 4) Sample-Time für Kanal 16 konfigurieren
    //    Kanal 16 wird in SMPR1[ (16-10)*3 ] = SMPR1[18..20 Bits] gesetzt
    //    7 (111b) in diesem Feld => 480 Zyklen.
    //    (je nach Referenzmanual: Bits für CH16 sind an Position 18..20 in SMPR1).
    ADC1_SMPR1 &= ~(7UL << (3 * (16 - 10)));    // vorher nullen
    ADC1_SMPR1 |=  (7UL << (3 * (16 - 10)));    // 111 => 480 cycles

    // 5) Kanal 16 als einzige Conversion in der Regular Sequence SQR3
    //    Die untersten 5 Bits in SQR3 wählen den Kanal aus.
    ADC1_SQR3 = 16UL;  // => 16 & 0x1F

    // 6) Sequenz-Länge = 1
    //    SQR1[L[3:0]] => 0 => 1 Conversion
    ADC1_SQR1 = 0; // L=0 => 1 Conversion

    // 7) ADC1 einschalten (ADON-Bit)
    ADC1_CR2 |= ADC_CR2_ADON;

    // Kurz warten, damit sich ADC intern einschwingen kann.
    // Evtl. kleine Software-Pause (z. B. for-Schleife).
    for (volatile int i = 0; i < 100000; i++) { /* kleine Wartezeit */ }
}

/*****************************************************************************/
/* Funktion:  ADC1_ReadTemperatureRaw                                       */
/* Zweck:     Startet eine Single-Conversion auf CH16 und liefert ADC-Wert.  */
/*****************************************************************************/
uint16_t ADC1_ReadTemperatureRaw(void)
{
    // 1) Start der Software-Conversion
    ADC1_CR2 |= ADC_CR2_SWSTART;

    // 2) Warten auf End-of-Conversion (EOC)
    while ((ADC1_SR & ADC_SR_EOC) == 0) {
        // hier optional Watchdog/Timeout
    }

    // 3) ADC-Daten aus dem Data-Register auslesen
    uint16_t adcValue = (uint16_t)(ADC1_DR & 0xFFFF);

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

        // Hier könnten Sie den Wert z. B. auf ein Display ausgeben
        // oder über UART senden, LED ansteuern etc.
        // ...

        // Einfache Pause
        for (volatile int i=0; i<1000000; i++) { /* Warten */ }
    }

    // Sollte nie hierher kommen
    return 0;
}
