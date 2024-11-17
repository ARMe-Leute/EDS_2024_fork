/*
 * bluetooth.c
 *
 *  Created on: 11.11.2024
 *      Author:
 */

#include <bluetooth.h>
#include <stm32f401xe.h>
#include <system_stm32f4xx.h>

#include <stdint.h>
#include <stdbool.h>

#include <mcalGPIO.h>
#include <mcalUsart.h>

// Create a data type named BluetoothModule_t

void bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *USART) {

	BluetoothModule->usart = USART;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA :Bustakt aktivieren
	gpioSelectPinMode(GPIOA, PIN3, ALTFUNC); // PA2 :Modus = Alt. Funktion
	gpioSelectAltFunc(GPIOA, PIN3, AF7); // PA2 :AF7 = USART2 Rx
	gpioSelectPinMode(GPIOA, PIN2, ALTFUNC); // PA3 :Modus = Alt. Funktion
	gpioSelectAltFunc(GPIOA, PIN2, AF7); // PA3 :AF7 = USART2 Tx
	usartSelectUsart(USART);
	usartEnableUsart(USART);
	usartSetCommParams(USART, 9600, NO_PARITY, LEN_8BIT, ONE_BIT);
	usartEnableIrq(USART, USART_IRQ_RXNEIE);

	if (BluetoothModule->usart == USART2) {
		NVIC_EnableIRQ(USART2_IRQn);
	} else {
		//Todo
	}
	__enable_irq();

}

void USART2_IRQHandler(void) {
	if (USART2->SR & USART_SR_RXNE) {
		uint16_t received = USART2->DR & 0x01FF;
		if ((received >= 'A') && (received <= 'Z')) {
			usartSendByte(USART2, received);
		}
	}

}

void bluetoothGetStatus(/*BluetoothModule_t BluetoothModule*/) {

	usartSendString(USART2, (uint8_t*) "AT");

}



uint32_t bluetoothBaud2Int(BLUETOOTH_BAUD BAUD) {
	uint32_t returnValue = 0;

	switch (BAUD) {

	case 0:
		returnValue = 9600;
		break;
	case 1:
		returnValue = 19200;
		break;
	case 2:
		returnValue = 38400;
		break;
	case 3:
		returnValue = 57600;
		break;
	case 4:
		returnValue = 115200;
		break;
	case 5:
		returnValue = 4800;
		break;
	case 6:
		returnValue = 2400;
		break;
	case 7:
		returnValue = 1200;
		break;
	case 8:
		returnValue = 230400;
		break;
	default:
		returnValue=0;
	}

	return returnValue;
}

void delay(uint16_t delay) {
	uint16_t i = 0;

	for (; delay > 0; --delay) {
		for (i = 0; i < 1245; ++i) {
			;
		}
	}
}

/**
 * @brief A test function that performs a specific operation.
 *
 * This function is intended to demonstrate the structure of a Doxygen comment.
 * Currently, the implementation is incomplete.
 *
 * @return uint32_t Returns a 32-bit unsigned integer.
 */
uint32_t test() {
	for (;;)
		;
}

