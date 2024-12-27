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


/* Constant for Voltage Conversion */
#define BATTERY_VOLTAGE_COEFFICIENT 0.80586

/* Functions of BatteryVoltage.c */
extern void ADCInit(GPIO_TypeDef *port, PIN_NUM_t pin, ADC_TypeDef *adc, ADC_CHANNEL_t chnList[], size_t listSize, ADC_RESOLUTION_t resolution);
extern uint16_t getBatteryMilliVolts(ADC_TypeDef *adc);

/* end of BatteryVoltage.h */
