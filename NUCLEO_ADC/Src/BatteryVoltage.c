/**
 ******************************************************************************
 * @file           : ReadOutBatteryVoltage.c
 * @author         : Erick Schlosser
 * @brief          : This is the C source code for reading out the Battery Voltage.
 *              	 It includes the implementations of the functions defined in "RotaryPushButton.h."
 *              	 Additionally, this source code contains the implementation of the interrupt functions.
 *              	 This library is based on the CMSIS and MCAL library.
 * @date		   : December 27, 2023
 ******************************************************************************
 */
#include <BatteryVoltage.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <mcalGPIO.h>
#include <mcalADC.h>


// --- ADC Common Register ---
#define ADC_CCR        (*((volatile uint32_t *)(ADC_COMMON_BASE + 0x04)))
// Im Referenzmanual oft ADC->CCR genannt; Offset 0x04 ab ADC_COMMON_BASE.

// Bitmasken für ADC_CCR
#define ADC_CCR_TSVREFE   (1UL << 23)  // Temperature sensor und V_REFINT einschalten



/* Global Variables */
static uint16_t batteryVoltageMilliVolts = 0;
static uint16_t ADCValue = 0;
const float vtemp25 = 1.43;
const float ratioTemp =	22.222;
uint16_t ADCTemp;


/*
 * InitADCGPIOPin ist dafür da die dazugehörigen GPIO Pins als analoge Eingänge zu konfigurieren.
 */
void InitADCGPIOPin(GPIO_TypeDef *port, PIN_NUM_t pin){
    gpioSelectPort(port); 									// Activate GPIO clock
	gpioSelectPinMode(port, pin, ANALOG);					// Setting the GPIO Pin to analog mode
	gpioSelectPushPullMode(port, pin, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
}

/*
 * InitADCChannels schaut nach welche ADC Channels verwendet werden und initialisiert die dazugehörigen Pins über InitADCGPIOPin
 */
void InitADCChannels(analogCh_t *analogCh){
	/* Wenn die Temperatur ebenfalls ausgelesen werden soll, wird der Channel16 auf die Channel Liste gesetzt */
	if (analogCh->tempEnable)
	{
		analogCh->chnListSize++;
		analogCh->chnList[chnListSize - 1] = ADC_CHN_16;
	}

	for (int i = 0; i < analogCh->listSize; i++) {
		switch (analogCh->chnList[i]) {
			case ADC_CHN_0:
				// ADC_CHN_0 -> PA0
				InitADCGPIOPin(GPIOA, PIN0);
				break;

			case ADC_CHN_1:
				// ADC_CHN_1 -> PA1
				InitADCGPIOPin(GPIOA, PIN1);
				break;

			case ADC_CHN_2:
				// ADC_CHN_2 -> PA2
				InitADCGPIOPin(GPIOA, PIN2);
				break;

			case ADC_CHN_3:
				// ADC_CHN_3 -> PA3
				InitADCGPIOPin(GPIOA, PIN3);
				break;

			case ADC_CHN_4:
				// ADC_CHN_4 -> PA4
				InitADCGPIOPin(GPIOA, PIN4);
				break;

			case ADC_CHN_5:
				// ADC_CHN_5 -> PA5
				InitADCGPIOPin(GPIOA, PIN5);
				break;

			case ADC_CHN_6:
				// ADC_CHN_6 -> PA6
				InitADCGPIOPin(GPIOA, PIN6);
				break;

			case ADC_CHN_7:
				// ADC_CHN_7 -> PA7
				InitADCGPIOPin(GPIOA, PIN7);
				break;

			case ADC_CHN_8:
				// ADC_CHN_8 -> PB0
				InitADCGPIOPin(GPIOB, PIN0);
				break;

			case ADC_CHN_9:
				// ADC_CHN_9 -> PB1
				InitADCGPIOPin(GPIOB, PIN1);
				break;

			case ADC_CHN_10:
				// ADC_CHN_10 -> PC0
				InitADCGPIOPin(GPIOC, PIN0);
				break;

			case ADC_CHN_11:
				// ADC_CHN_11 -> PC1
				InitADCGPIOPin(GPIOC, PIN1);
				break;

			case ADC_CHN_12:
				// ADC_CHN_12 -> PC2
				InitADCGPIOPin(GPIOC, PIN2);
				break;

			case ADC_CHN_13:
				// ADC_CHN_13 -> PC3
				InitADCGPIOPin(GPIOC, PIN3);
				break;

			case ADC_CHN_14:
				// ADC_CHN_14 -> PC4
				InitADCGPIOPin(GPIOC, PIN4);
				break;

			case ADC_CHN_15:
				// ADC_CHN_15 -> PC5
				InitADCGPIOPin(GPIOC, PIN5);
				break;

			case ADC_CHN_16:
				// Interner Temperatur-Sensor, kein externer Pin
				ADC_CCR |= ADC_CCR_TSVREFE;
				break;

			default:
				//printf("Unbekannter ADC-Kanal!\n");
				break;
		}
	}
}


/* ADC Initialization and Configuration for Polling */
void ADCInit(analogCh_t *analogCh) {
    adcSelectADC(analogCh->adc);     									// Activate ADC clock
    adcDisableADC(analogCh->adc);										// Disable the ADC to configure it
    InitADCChannels(*analogCh);
    adcSetChannelSequence(analogCh->adc, analogCh->chnList, analogCh->chnListSize);			// Select the ADC-Channel
    adcSetResolution(analogCh->adc, analogCh->chnResolution);						// Select the ADC Resolution
    adcEnableADC(analogCh->adc);										// Enable the ADC again after configuration
}

/* Reading the ADC Value with Polling and returning the calculated Voltage*/
uint16_t getBatteryMilliVolts(ADC_TypeDef *adc) {
    if (adc == NULL) {
        return 0; // Safety check
    }
    adcStartConversion(adc);
    while (!adcIsConversionFinished(adc)) {
        ; //waiting for the adc conversion to be finished
    }
    ADCValue = adcGetConversionResult(adc);
    batteryVoltageMilliVolts = ADCValue * BATTERY_VOLTAGE_COEFFICIENT;
    return batteryVoltageMilliVolts;
}
/* end of BatteryVoltage.c */


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
