/*
 * bluetooth.c
 *
 *  Created on: 26.11.2024
 *
 */

#include <math.h>

#include <bluetooth.h>

int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *usart, uint32_t baudRate) {


	switch (BluetoothModule->initStatus) {
	case -10:
		BluetoothModule->usart = usart;
		BluetoothModule->baudRate = baudRate;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA :Bustakt aktivieren
		gpioSelectPinMode(GPIOA, PIN3, ALTFUNC); // PA2 :Modus = Alt. Funktion
		gpioSelectAltFunc(GPIOA, PIN3, AF7); // PA2 :AF7 = USART2 Rx
		gpioSelectPinMode(GPIOA, PIN2, ALTFUNC); // PA3 :Modus = Alt. Funktion
		gpioSelectAltFunc(GPIOA, PIN2, AF7); // PA3 :AF7 = USART2 Tx
		usartSelectUsart(usart);
		usartEnableUsart(usart);
		usartSetCommParams(usart, 9600, NO_PARITY, LEN_8BIT, ONE_BIT);
		usartEnableIrq(usart, USART_IRQ_RXNEIE);

		if (BluetoothModule->usart == USART2) {
			usart2BufferSize = ceil(2 * BluetoothModule->baudRate / 8 * 0.1); // Size to allocate Bytes per second * timerLength, *2 for safety
			NVIC_EnableIRQ(USART2_IRQn);
		} else {
			//Todo
		}
		__enable_irq();
		return BluetoothModule->initStatus++;

	default:
		return 0;
	}
}
