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

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <mcalGPIO.h>
#include <mcalADC.h>

#define halfBatVolt  (float)14
#define emptyBatVolt (float)13

typedef struct adc
{
	ADC_TypeDef *adc;
	ADC_RESOLUTION_t resolution;
	float AnalogVoltage;
	ADC_CHANNEL_t chnList[];
}adcAnalog_t;

typedef enum
{
	okBat = 0,
	halfBat,
	emptyBat,
	underVolt,
	overVolt
} BatStat_t;

typedef struct analogCh
{
	ADC_TypeDef *adc;
	ADC_RESOLUTION_t chnResolution;
	size_t chnListSize;
	ADC_CHANNEL_t chnList[18];
	int chnADCValue[18];
	float chnVolt[18];
	bool tempEnable;
	float tempValue;
} analogCh_t;

const float chnConversionCoefficient = 0.00080586080586;

const int VSense = 24;


/* Functions of BatteryVoltage.c */
extern void ADCInit(GPIO_TypeDef *port, PIN_NUM_t pin, ADC_TypeDef *adc, ADC_CHANNEL_t chnList[], size_t listSize, ADC_RESOLUTION_t resolution);
extern uint16_t getBatteryMilliVolts(ADC_TypeDef *adc);

extern void adcActivate(void);
extern BatStat_t getBatVolt(analogCh_t* pADChn, size_t listSize);


/* end of BatteryVoltage.h */
