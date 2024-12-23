/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stdint.h>

//#include <stm32f401xe.h>  // select the MCU of Board NUCLEO_F401RE
#include <stm32f401xc.h>	// select the MCU for Board AZ Delivery STM32F401CCU6 V3.0
#include <system_stm32f4xx.h>

void delayMillis(uint16_t delay)
/**
* !!!Sehr schlechte Version eines Delay!!!
*/
{
	uint16_t i = 0;

	for (; delay > 0; --delay)
	{
		for (i = 0; i < 1245; ++i)
		{
			;
		}
	}
}

void blink_alert()
{


}


int main(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIOC:	Clock aktivieren
	GPIOC->MODER &= ~GPIO_MODER_MODE0_Msk; // GPIOC: PC0	reset
	GPIOC->MODER |= GPIO_MODER_MODER13_0; // GPIOC: PC13	--> Ausgang LED GREEN
	//GPIOA->MODER |= 0b010000000000; //GPIO_MODER_MODER5_;

    /* Loop forever */
	for(;;)
	{
		GPIOC->ODR |= GPIO_ODR_OD13;  // GPIOC: Bit 13	in ODR --> 1
		delayMillis(500);
		GPIOC->ODR &= ~GPIO_ODR_OD13; // GPIOC: Bit 13 in ODR --> 0
		delayMillis(500);
	}
}
