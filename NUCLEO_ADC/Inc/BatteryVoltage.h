/**
 ******************************************************************************
 * @file           : BatteryVoltage.h
 * @author         : Erick Schlosser
 * @brief          : This is the header file for reading out the battery voltage with an analog input.
 *              	 It includes functions for initializing the Pin, configuring the ADC, reading the value
 *              	 as well as calculating the voltage
 * @date		   : December 19, 2024
 ******************************************************************************
 */
#ifndef BATTERYVOLTAGE_H
#define BATTERYVOLTAGE_H

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <mcalADC.h>
#include <mcalGPIO.h>

//#define halfBatVolt  (float)14
//#define emptyBatVolt (float)13
//
//typedef enum
//{
//	okBat = 0,
//	halfBat,
//	emptyBat,
//	underVolt,
//	overVolt
//} BatStat_t;

typedef struct analogSingleCh
{
	ADC_TypeDef *adc;				// ADC Adresse
	ADC_Common_TypeDef *adccommon;	// ADC_COMMON Adresse
	ADC_RESOLUTION_t chnResolution;	// Auflösung des ADC
	size_t chnListSize;				// Größe der ChannelListe
	ADC_CHANNEL_t chnList[1];		// ADC Kanal, der ausgewertet werden sollen
	int chnADCValue[1];				// ADC Rohwert des Kanals
	float chnVolt[1];				// Spannungswerte der einzelnen Kanäle
	float Prescaler[1];				// Falls ein Spannungsteiler verwendet wird, kann dieser hier eingetragen werden. Vorteilfaktor Ratio für jeden einzelnen Kanal (als Array)
	bool tempEnable;				// Für die Verwendung vom internen Temperatursensor
	bool interruptEnable;			// Für die Verwendung einer Interruptgesteuerten Abfrage
	float alpha_lowpass; 			// Faktor für den rekursiven Tiefpassfilter
	float tempValue;				// Temperaturwert in °C
} analogSingleCh_t;

typedef struct analogMultiCh
{
	ADC_TypeDef *adc;				// ADC Adresse
	ADC_Common_TypeDef *adccommon;	// ADC_COMMON Adresse
	ADC_RESOLUTION_t chnResolution;	// Auflösung des ADC
	size_t chnListSize;				// Größe der ChannelListe
	ADC_CHANNEL_t chnList[18];		// ADC Kanäle, die ausgewertet werden sollen
	int chnADCValue[18];			// ADC Rohwerte der einzelnen Kanäle
	float chnVolt[18];				// Spannungswerte der einzelnen Kanäle
	float Prescaler[18];			// Falls ein Spannungsteiler verwendet wird, kann dieser hier eingetragen werden
	bool tempEnable;				// Vorteilfaktor Ratio für jeden einzelnen Kanal (als Array)
	float scaler_lowpass; 			// Faktor für den rekursiven Tiefpassfilter
	float tempValue;				// Temperaturwert in °C
} analogMultiCh_t;

/* Functions of BatteryVoltage.c */
extern void InitADCGPIOPin(GPIO_TypeDef *port, PIN_NUM_t pin);
extern void InitADCChannels(analogSingleCh_t *analogCh);
extern void ADCInit(analogSingleCh_t *analogCh);
extern void getAnalogPinValuePolling(analogSingleCh_t *analogCh);
extern void getVoltagePinValuePolling(analogSingleCh_t *analogCh);



//extern BatStat_t getBatVolt(analogCh_t* pADChn, size_t listSize);


/* end of BatteryVoltage.h */
#endif // BATTERYVOLTAGE_H
