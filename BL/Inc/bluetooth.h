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

typedef struct BluetoothModule {

	const USART_TypeDef *usart;
	int8_t initStatus; //Todo: Defaultwert mitgeben
	uint32_t baudRate;
	char messageBuffer[USART2_BUFFER_SIZE+1];
	uint16_t available;
	uint8_t counter;
	bool ATInProgress;

}BluetoothModule_t;

typedef enum BluetoothError{
	BluetoothFinish = 0,
	BluetoothBusy,
	BluetoothWrongParameter,
	BluetoothRetryError
}BluetoothError_t;



int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *USART,
		uint32_t baudRate);
bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule);
bool bluetoothGetStatus(BluetoothModule_t *BluetoothModule);

extern volatile char usart2Buffer[];
extern volatile uint16_t usart2BufferIndex;

#endif /* BLUETOOTH_H_ */
