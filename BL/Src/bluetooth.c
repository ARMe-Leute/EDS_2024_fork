/**
 * @file bluetooth.c
 * @brief Bluetooth library implementation for STM32 using MCAL.
 *
 * This file provides the definitions and logic for interacting with the HM17 Bluetooth module
 * using the STM32-Nucleo-F401RE board.
 *
 * Copy this file to the Src folder of your project.
 *
 * @author c0deberry
 * @author nrs00
 *
 * Created on: 26.11.2024
 */

#include <bluetooth.h>


/**
 * @brief Initializes the Bluetooth module.
 *
 * Configures USART and GPIO settings to establish communication
 * with the Bluetooth module. The initialization progresses through multiple states
 * to ensure a non-blocking implementation.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param usart Pointer to the USART instance, e.g., USART2.
 * @param baudRate Communication baud rate; the default for HM17 is 9600.
 *  @return 0 on successful initialization, negative value if not finished,
 *  or positive value if there is an error.
 */
int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *usart,
		uint32_t baudRate) {

	switch (BluetoothModule->initStatus) {
	case -10:
		BluetoothModule->usart = usart;
		BluetoothModule->baudRate = baudRate;
		BluetoothModule->available = 0;
		BluetoothModule->counter = 0;
		BluetoothModule->state = 0;

		// Clear the buffer during initialization
		for (uint32_t i = 0; i < USART2_BUFFER_SIZE; i++) {
			BluetoothModule->messageBuffer[i] = '\0';
		}
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 		// Enable GPIOA clock
		gpioSelectPinMode(GPIOA, PIN3, ALTFUNC); 	// PA3: Alternate Function Mode
		gpioSelectAltFunc(GPIOA, PIN3, AF7); 		// PA3: AF7 for USART2 Rx
		gpioSelectPinMode(GPIOA, PIN2, ALTFUNC); 	// PA2: Alternate Function Mode
		gpioSelectAltFunc(GPIOA, PIN2, AF7); 		// PA2: AF7 for USART2 Tx
		usartSelectUsart(usart);
		usartEnableUsart(usart);
		usartSetCommParams(usart, 9600, NO_PARITY, LEN_8BIT, ONE_BIT);

#ifndef debugMode
		usartEnableIrq(usart, USART_IRQ_RXNEIE);
		if (BluetoothModule->usart == USART2) {
			NVIC_EnableIRQ(USART2_IRQn);
		} else {
			// Todo: Handle other USART instances if needed
		}
		__enable_irq();
#endif
		return ++BluetoothModule->initStatus;

	case -9:
		if (BluetoothModule->counter > 10) { // Return an error after 10 failed attempts
			return BluetoothRetryError;
		}
		bool reply;
		int16_t status;
		status = bluetoothGetStatus(BluetoothModule, &reply);
		if (status == 0 && reply == true) { // HM17 replied with OK
			return BluetoothModule->initStatus = 0;
		} else if (status < 0) { // Pending steps
			return BluetoothModule->initStatus;
		} else { // Retry on error
			BluetoothModule->counter++;
			return BluetoothModule->initStatus - 9;
		}

	default:
		return 0;
	}

}

/**
 * @brief Handles the state transitions for the Bluetooth state machine.
 *
 * Executes commands by transitioning between the states definded in ::BluetoothState.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param state Initial state for processing. This must be the first step of a procedure.
 * @return Next ::BluetoothState, or a ::BluetoothError code on failure.
 */
int16_t bluetoothStateHandler(BluetoothModule_t *BluetoothModule, int16_t state) {

	// Ensure it is an initial step
	if (state % 10 != 0) {
		return BluetoothWrongParameter;
	}
	if (BluetoothModule->state == 0) { // Begin if no state is active
		BluetoothModule->state = state;
		// Ensure the called state matches the current procedure
	} else if (!(BluetoothModule->state - state < 10)) {
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
		BluetoothModule->state = BluetoothFinish; // Mark as finished
		if (BluetoothModule->available >= 2) { // Sufficient characters received
			return BluetoothFinish;
		} else { // Insufficient data
			return BluetoothLengthError;
		}
	default:
		return BluetoothWrongParameter;
	}

}

/**
 * @brief Sends the "AT" command to the Bluetooth module.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param isOK Pointer to a boolean indicating the command result.
 * @return ::BluetoothFinish on success, or a ::BluetoothError code on failure.
 *
 * @note Check isOK only if the return value is ::BluetoothFinish.
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
	BluetoothModule->available = 0; // Reset buffer
	if (reply == 0) { // We can look for an OK
		*isOK = (strncmp(BluetoothModule->messageBuffer, "OK", 2) == 0);
		return BluetoothFinish;
	} else if (reply < 0) {
		return reply; // Continue
	} else {
		return reply; // Pass error or do error handling here
	}

}

/**
 * @brief Transfers data from the global USART buffer to the module's buffer.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @return True if data transfer is complete, false otherwise.
 */
bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule) {
	if (BluetoothModule->usart == USART2) {
		static uint16_t lastIndex;
		if (usart2BufferIndex == lastIndex && usart2BufferIndex != 0) { // Check if no new characters have been received and the buffer is not empt
			NVIC_DisableIRQ(USART2_IRQn); // Disable USART2 IRQ during data transfer
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
		// Todo: Implement support for additional USART instances
		return false;
	}
}

#ifndef USART2_BUFFER_SIZE
#warning USART2_BUFFER_SIZE not defined, using 120 Bytes buffer. This may result in lost characters
#define USART2_BUFFER_SIZE 120
#endif

/**
 * @brief USART2 interrupt handler for receiving data.
 *
 * Handles incoming data on USART2 and stores it in the global interrupt buffer.
 */
void USART2_IRQHandler(void) {
#ifndef debugMode
	if (USART2->SR & USART_SR_RXNE) {
		usart2Buffer[usart2BufferIndex++] = USART2->DR & 0xFF; // Ensure 8-bit data
	}
#endif

}
