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


struct BluetoothModule {
// Todo: Variablen und Funktionen
	USART_TypeDef*  usart;

};

void bluetoothInit(BluetoothModule_t* BluetoothModule, USART_TypeDef* USART) {
	//Todo: Usart richtig verwenden

	BluetoothModule->usart = USART;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA :Bustakt aktivieren
	gpioSelectPinMode(GPIOA, PIN3, ALTFUNC); // PA2 :Modus = Alt. Funktion
	gpioSelectAltFunc(GPIOA, PIN3, AF7); // PA2 :AF7 = USART2 Rx
	gpioSelectPinMode(GPIOA, PIN2, ALTFUNC); // PA3 :Modus = Alt. Funktion
	gpioSelectAltFunc(GPIOA, PIN2, AF7); // PA3 :AF7 = USART2 Tx
	usartSelectUsart(BluetoothModule->usart);
	usartEnableUsart(BluetoothModule->usart);
	usartSetCommParams(BluetoothModule->usart, 9600, NO_PARITY, LEN_8BIT, ONE_BIT);


}

void bluetoothGetStatus(/*BluetoothModule_t BluetoothModule*/) {

	usartSendString(USART2, (uint8_t*) "AT");

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

