/*
 * bluetooth.c
 *
 *  Created on: 26.11.2024
 *
 */

#include <bluetooth.h>

int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *usart,
		uint32_t baudRate) {

	switch (BluetoothModule->initStatus) {
	case -10:
		BluetoothModule->usart = usart;
		BluetoothModule->baudRate = baudRate;
		BluetoothModule->available = 0;
		BluetoothModule->counter = 0;
		BluetoothModule->state = 0;
		for (uint32_t i = 0; i < USART2_BUFFER_SIZE; i++) {
			BluetoothModule->messageBuffer[i] = '\0';
		}
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA :Bustakt aktivieren
		gpioSelectPinMode(GPIOA, PIN3, ALTFUNC); // PA2 :Modus = Alt. Funktion
		gpioSelectAltFunc(GPIOA, PIN3, AF7); // PA2 :AF7 = USART2 Rx
		gpioSelectPinMode(GPIOA, PIN2, ALTFUNC); // PA3 :Modus = Alt. Funktion
		gpioSelectAltFunc(GPIOA, PIN2, AF7); // PA3 :AF7 = USART2 Tx
		usartSelectUsart(usart);
		usartEnableUsart(usart);
		usartSetCommParams(usart, 9600, NO_PARITY, LEN_8BIT, ONE_BIT);

#ifndef debugMode
		usartEnableIrq(usart, USART_IRQ_RXNEIE);
		if (BluetoothModule->usart == USART2) {
			NVIC_EnableIRQ(USART2_IRQn);
		} else {
			//Todo
		}
		__enable_irq();
#endif
		return ++BluetoothModule->initStatus;

	case -9:
		if (BluetoothModule->counter > 10) { // after 10 attempts return negative acknowledge
			return BluetoothRetryError;
		}
		bool reply;
		int16_t status;
		status = bluetoothGetStatus(BluetoothModule, &reply);
		if (status == 0 && reply == true) { // We are OK
			return BluetoothModule->initStatus = 0;
		} else if (status < 0) { // We have steps to do
			return BluetoothModule->initStatus;
		} else { // Something went wrong, try again
			BluetoothModule->counter++;
			return BluetoothModule->initStatus - 9;
		}

	default:
		return 0;
	}

}

int16_t bluetoothStateHandler(BluetoothModule_t *BluetoothModule, int16_t state) {

	if (state % 10 != 0) { // We don't start at the beginning of a procedure
		return BluetoothWrongParameter;
	}
	if (BluetoothModule->state == 0) {
		BluetoothModule->state = state;
	} else if (!(BluetoothModule->state - state < 10)) { // We request a procedure, but are still in another one
		return BluetoothBusy;
	}
	switch (BluetoothModule->state) {

	case getStatus:
		usartSendString(USART2, (char*) "AT");

#ifdef BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK
		usart2Buffer[usart2BufferIndex++] = 'O';
		usart2Buffer[usart2BufferIndex++] = 'K';
#endif //BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK

#ifdef BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR
		usart2Buffer[usart2BufferIndex++] = 'E';
		usart2Buffer[usart2BufferIndex++] = 'R';
		usart2Buffer[usart2BufferIndex++] = 'R';
		usart2Buffer[usart2BufferIndex++] = 'O';
		usart2Buffer[usart2BufferIndex++] = 'R';
#endif //BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR

		return ++BluetoothModule->state;

	case getStatus_2:
		BluetoothModule->state = BluetoothFinish; // In any case we are finished here
		if (BluetoothModule->available >= 2) { // We have more than two characters and are probably fine
			return BluetoothFinish;
		} else { // Not enough characters to be ok
			return BluetoothLengthError;
		}
	default:
		return BluetoothWrongParameter;
	}

}
/**
 * @brief
 *
 *
 * @warning: Only check what is in isOK if the returned status is equal to 0, otherwise it could be anything
 */
int16_t bluetoothGetStatus(BluetoothModule_t *BluetoothModule, bool *isOK) {
#ifdef BLUETOOTH_GET_STATUS_RETURN_OK
	*isOK = true;
	return 0;
#endif //BLUETOOTH_GET_STATUS_RETURN_OK
#ifdef BLUETOOTH_GET_STATUS_RETURN_ERROR
	*isOK = false;
	return BLUETOOTH_GET_STATUS_RETURN_ERROR;
#endif //BLUETOOTH_GET_STATUS_RETURN_ERROR

	int16_t reply = bluetoothStateHandler(BluetoothModule, getStatus);
	BluetoothModule->available = 0; // Reset the buffer
	if (reply == 0) { // do important stuff here
		if (strncmp(BluetoothModule->messageBuffer, "OK", 2) == 0) {
			*isOK = true;
		} else {
			*isOK = false;
		}
		return BluetoothFinish;
	} else if (reply < 0) {
		return reply;
	} else {
		return reply; // We have an error, just pass it and let the parent function take care of it
	}

}

bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule) {
	if (BluetoothModule->usart == USART2) {
		static uint16_t lastIndex;
		if (usart2BufferIndex == lastIndex && usart2BufferIndex != 0) { // No new characters received, message is finished
			NVIC_DisableIRQ(USART2_IRQn); // Disable the IRQ while copying
			for (uint16_t x = 0; x < usart2BufferIndex; x++) {

				BluetoothModule->messageBuffer[BluetoothModule->available] =
						usart2Buffer[x];
				BluetoothModule->available++;
			}
			BluetoothModule->messageBuffer[BluetoothModule->available] = '\0';
			usart2BufferIndex = 0;
			NVIC_EnableIRQ(USART2_IRQn);
			return true;
		} else {
			lastIndex = usart2BufferIndex;
			return false;
		}

	} else {
		// Todo: Other USARTs
		return false;
	}
}

#ifndef USART2_BUFFER_SIZE
#warning USART2_BUFFER_SIZE not defined, using 1200 Bytes buffer. This may result in lost characters
#define USART2_BUFFER_SIZE 1200 // One second at 9600 BAUD
#endif

void USART2_IRQHandler(void) {
#ifndef debugMode
	if (USART2->SR & USART_SR_RXNE) {
		usart2Buffer[usart2BufferIndex++] = USART2->DR & 0xFF;
	}
#endif

}
