/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */



#include <bluetooth.h>
#include <stm32f401xe.h>
#include <system_stm32f4xx.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <mcalGPIO.h>
#include <mcalUsart.h>


BluetoothModule_t HM17;

char Usart2receivedChar;
bool Usart2charReceived;


int main(void)
{
	bluetoothInit(&HM17, USART2);
	Usart2charReceived = false;

	usartSetCommParams(USART2, 9600, NO_PARITY, LEN_8BIT, ONE_BIT);  //Muss aus irgend einem Grund drin bleiben
	bluetoothGetStatus();
    /* Loop forever */
	for(;;){
		bluetoothGetStatus();
		delay(500);
	}
}


