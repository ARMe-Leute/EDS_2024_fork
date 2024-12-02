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

/**
 * @struct BluetoothATReplyBool
 * @brief A struct that can be returned by AT command functions to return status and a boolean value.
 *
 * As using an AT-command requires two steps (Send something, receive something),
 * we need to keep track of the step. We also want that a function can return a value
 * (e.g. the MAC-Address). As functions cannot return two values in C [Citation needed],
 * we can use this struct to return the current step and a boolean.
 *
 * @var BluetoothATReplyBool::status
 * The step counter. Begins with a negative number. Once finished, it will be 0.
 * If something didn't work, it will return 0x15 (negative acknowledge).
 *
 * @var BluetoothATReplyBool::boolean
 * The boolean value a function returns
 */
typedef struct BluetoothATReplyBool{
	int8_t status;
	bool boolean;
}BluetoothATReplyBool_t;

/**
 * @struct BluetoothATReplyString
 * @brief A struct that can be returned by AT command functions to return status and a string.
 *
 * As using an AT-command requires two steps (Send something, receive something),
 * we need to keep track of the step. We also want that a function can return a value
 * (e.g. the MAC-Address). As functions cannot return two values in C [Citation needed],
 * we can use this struct to return the current step and a string with max. 20 characters.
 *
 * @var BluetoothATReplyBool::status
 * The step counter. Begins with a negative number. Once finished, it will be 0.
 * If something didn't work, it will return 0x15 (negative acknowledge).
 *
 * @var BluetoothATReplyString::string
 * The string a function returns, with max. 20 characters
 */
typedef struct BluetoothATReplyString{
	int8_t status;
	char string [20];
}BluetoothATReplyString_t;

/**
 * @struct BluetoothATReplyInt
 * @brief A struct that can be returned by AT command functions to return status and an int.
 *
 * As using an AT-command requires two steps (Send something, receive something),
 * we need to keep track of the step. We also want that a function can return a value
 * (e.g. the MAC-Address). As functions cannot return two values in C [Citation needed],
 * we can use this struct to return the current step and an int64_t.
 *
 * @var BluetoothATReplyBool::status
 * The step counter. Begins with a negative number. Once finished, it will be 0.
 * If something didn't work, it will return 0x15 (negative acknowledge).
 *
 * @var BluetoothATReplyInt::integer
 * The int value a function returns, is of type int64_t
 */
typedef struct BluetoothATReplyInt{
	int8_t status;
	int64_t integer;
}BluetoothATReplyInt_t;

int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *USART,
		uint32_t baudRate);
bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule);

extern volatile char usart2Buffer[];
extern volatile uint16_t usart2BufferIndex;

#endif /* BLUETOOTH_H_ */
