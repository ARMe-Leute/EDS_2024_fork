/**
 * @file bluetooth.h
 * @brief Bluetooth library interface for STM32 using MCAL.
 *
 * This file provides the declarations and configurations necessary to use
 * the HM17 Bluetooth module with the STM32-Nucleo-F401RE board.
 *
 * Copy this file to the Inc folder of your project.
 *
 * @author c0deberry
 * @author nrs00
 *
 * Created on: 26.11.2024
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <string.h>

#include <mcalUsart.h>
#include <mcalGPIO.h>

/**
 * @brief Interval at which the buffer is fetched, in milliseconds.
 */
#define BLUETOOTH_FETCH_TIME 50 //ms

/**
 * @brief Size of the USART2 buffer, calculated based on the fetch interval.
 *
 * The buffer size should be at least twice the fetch interval.
 *
 * @warning If the size is set too small, received data may be lost.
 */
#define USART2_BUFFER_SIZE (2*1200*BLUETOOTH_FETCH_TIME)/1000

/**
 * @brief Enable debug mode for the library.
 *
 * This allows you to inject messages and run the debugger.
 */
// Uncomment to enable debug mode
#define debugMode

#ifdef debugMode

/**
 * @brief Forces bluetoothGetStatus() to always return OK.
 */
#define BLUETOOTH_GET_STATUS_RETURN_OK

/**
 * @brief Forces bluetoothGetStatus() to fail with a specific error.
 */
#define BLUETOOTH_GET_STATUS_RETURN_ERROR BluetoothLengthError

/**
 * @brief Simulates receiving "OK" in the buffer when calling bluetoothGetStatus().
 * This is then handled by bluetoothFetchBuffer().
 */
#define BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK

/**
 * @brief Simulates receiving "ERROR" in the buffer when calling bluetoothGetStatus().
 * This is then handled by bluetoothFetchBuffer().
 */
#define BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR

#if !(defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR))
/**
 * @warning If no reply simulation is defined, the program may hang.
 */
#warning "You don't receive any reply, expect the program to be stuck"
#endif // !(defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR))
#endif //debugMode

/**
 * @brief Structure representing a Bluetooth module instance.
 */
typedef struct BluetoothModule
    {
	const USART_TypeDef *usart; /**< USART instance for communication. */
	int8_t initStatus; /**< Initialization status of the module. */
	uint32_t baudRate; /**< Baud rate for the USART. */
	char messageBuffer[USART2_BUFFER_SIZE + 1]; /**< Buffer for received messages. */
	uint16_t available; /**< Number of available characters in the buffer. */
	uint8_t counter; /**< Counter for internal operations, e.g., retry count. */
	int16_t state; /**< Current state of the state machine. */

    } BluetoothModule_t;

/**
 * @brief Enum representing possible Bluetooth errors and status codes.
 */
typedef enum BluetoothError
    {
    BluetoothFinish = 0, /**< Operation completed successfully. */
    BluetoothBusy, /**< Module is busy with another command. */
    BluetoothWrongParameter, /**< Invalid parameter provided. */
    BluetoothRetryError, /**< Retry operation error. */
    BluetoothLengthError /**< Reply length error length error. */
    } BluetoothError_t;

/**
 * @brief Enum defining states for the Bluetooth state machine.
 */
enum BluetoothState
    {
    getStatus = -10, /**< Initial state for getStatus command. */
    getStatus_2, /**< Secondary state for getStatus command. */
    getMacAddress = -20, /**< Initial state for getMacAddress command. */
    getMacAddress_2 /**< Secondary state for getMacAddress command. */
    };

int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *USART,
	uint32_t baudRate);
bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule);
int16_t bluetoothGetStatus(BluetoothModule_t *BluetoothModule, bool *isOK);
int16_t bluetoothStateHandler(BluetoothModule_t *BluetoothModule, int16_t state);

/**
 * @brief Global buffer for USART2 communication.
 *
 * This buffer is used by the USART2_IRQHandler() to store incoming characters
 */
extern volatile char usart2Buffer[];

/**
 * @brief Index for the USART2 buffer.
 *
 * Indicates the next position in the buffer to write to.
 */
extern volatile uint16_t usart2BufferIndex;

#endif /* BLUETOOTH_H_ */
