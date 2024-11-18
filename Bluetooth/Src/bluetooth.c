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
		uint16_t received = USART2->DR & 0xFF;
		receivedChar = received;
		charReceived = true;
	}

}

void bluetoothGetStatus(/*BluetoothModule_t BluetoothModule*/) {

	usartSendString(USART2, (uint8_t*) "AT");
	free(bluetoothReceiveString(USART2, 2, 0));

}

/**
 * @brief Receive a given amount of characters.
 *
 * This is a blocking function, so watch out choosing the timeout.
 * If the target length is not
 * achieved after this time, the string will be returned as it is.
 * If no character got received, NULL will be returned.
 * Calling this function with no limit and no timeout will return NULL
 *
 * Don't forget to free the memory!
 *
 * @param length: How many characters should be received,
 * maximum is 64 Kibibyte -1 Byte
 * 0 means until timeout
 * @param timeout: The timeout in ms
 * maximum is 65.535 s.
 * 0 means no timeout (Not recommended)
 *
 */
char* bluetoothReceiveString(BluetoothModule_t *BluetoothModule,
		uint16_t length, uint16_t timeout) {

	if (length == 0 && timeout == 0) {
		return NULL;
	}

	//Todo: Timeout

	if (length == 0) {
		length -= 2; //Set the length to maximum by subtracting 1 (0-1==65534)
	}
	char *String = malloc((length + 1) * sizeof(char));
	if (String == NULL) {
		return NULL;
	}

	for (uint16_t emptyPosition = 0; emptyPosition < length + 1;
			emptyPosition++) {
		String[emptyPosition] = '\0';
	}

	uint16_t charCounter = 0;
	while (charCounter < length) {
		String[charCounter] = bluetoothReceiveChar(BluetoothModule);
		charCounter++;
	}

	return String;
}

char bluetoothReceiveChar(BluetoothModule_t *BluetoothModule) {

	/*	uint32_t test = BluetoothModule->usart->SR;
	 while (!(test & USART_SR_RXNE)) {
	 //Todo: Timeout
	 usartSendString(USART2, (uint8_t*) "AT");
	 }
	 return (char) (BluetoothModule->usart->DR & 0xFF);*/

	while (charReceived == false) {
		usartSendString(USART2, (uint8_t*) "AT");
	}
	charReceived = false;

	return (char) receivedChar;

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
		returnValue = 0;
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

