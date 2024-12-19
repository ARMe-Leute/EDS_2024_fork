/*
 * bluetooth.h
 *
 *  Created on: 26.11.2024
 *
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <string.h>

#include <mcalUsart.h>
#include <mcalGPIO.h>

#define BLUETOOTH_FETCH_TIME 50//ms
#define USART2_BUFFER_SIZE (2*1200*BLUETOOTH_FETCH_TIME)/1000

#define debugMode

#ifdef debugMode

// Let bluetoothGetStatus() directly return OK without running anything
//#define BLUETOOTH_GET_STATUS_RETURN_OK

// Let bluetoothGetStatus() directly fail without running anything
//#define BLUETOOTH_GET_STATUS_RETURN_ERROR BluetoothLengthError

//Emulates a received OK, which is then handled by bluetoothFetchBuffer()
#define BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK

//Emulates a received ERROR, which is then handled by bluetoothFetchBuffer()
//#define BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR



#if !(defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR))
#warning "You don't receive any reply, expect the program to be stuck"
#endif // !(defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR))
#endif //debugMode




typedef struct BluetoothModule {

	const USART_TypeDef *usart;
	int8_t initStatus; //Todo: Defaultwert mitgeben
	uint32_t baudRate;
	char messageBuffer[USART2_BUFFER_SIZE + 1];
	uint16_t available;
	uint8_t counter;
	bool ATInProgress;
	int16_t state;

} BluetoothModule_t;

typedef enum BluetoothError {
	BluetoothFinish = 0,
	BluetoothBusy,
	BluetoothWrongParameter,
	BluetoothRetryError,
	BluetoothLengthError
} BluetoothError_t;

enum BluetoothState {
	getStatus = -10,
	getStatus_2,
	getMacAddress = -20,
	getMacAddress_2
};

int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *USART,
		uint32_t baudRate);
bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule);
int16_t bluetoothGetStatus(BluetoothModule_t *BluetoothModule, bool *isOK);
int16_t bluetoothStateHandler(BluetoothModule_t *BluetoothModule, int16_t state);

extern volatile char usart2Buffer[];
extern volatile uint16_t usart2BufferIndex;

#endif /* BLUETOOTH_H_ */
