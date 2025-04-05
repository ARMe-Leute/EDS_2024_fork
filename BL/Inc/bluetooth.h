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
#include <stdio.h>

#include <mcalUsart.h>
#include <mcalGPIO.h>
#include <mcalDMAC.h>

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
#define USART2_RX_BUFFER_SIZE (2*1200*BLUETOOTH_FETCH_TIME)/1000

#define USART2_MIN_TX_BUFFER_SIZE 128

#define BLUETOOTH_TRANSMIT_TIME 100 //ms

#define BLUETOOTH_MAX_NAME_LENGTH 20

#define BLUETOOTH_NUMBER_OF_LOG_ENTRYS 2

#define BLUETOOTH_PIO0 GPIOA, 4

/**
 * @brief Enable debug mode for the library.
 *
 * This allows you to inject messages and run the debugger.
 */
// Uncomment to enable debug mode
//#define debugMode
#ifdef debugMode

/**
 * @brief Forces bluetoothGetStatus() to always return OK.
 */
//#define BLUETOOTH_GET_STATUS_RETURN_OK

/**
 * @brief Forces bluetoothGetStatus() to fail with a specific error.
 */
#define BLUETOOTH_GET_STATUS_RETURN_ERROR BluetoothLengthError

/**
 * @brief Simulates receiving "OK" in the buffer when calling bluetoothGetStatus().
 * This is then handled by bluetoothFetchBuffer().
 */
//#define BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK

/**
 * @brief Simulates receiving "ERROR" in the buffer when calling bluetoothGetStatus().
 * This is then handled by bluetoothFetchBuffer().
 */
//#define BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR

#if !(defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR))
/**
 * @warning If no reply simulation is defined, the program may hang.
 */
#warning "You don't receive any reply, expect the program to be stuck"
#endif // !(defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR))
#endif //debugMode

/**
 * @brief Enum representing possible Bluetooth errors and status codes.
 */
typedef enum BluetoothError
   {
   BluetoothFinish = 0, /**< Operation completed successfully. */
   BluetoothBusy, /**< Module is busy with another command. */
   BluetoothWrongParameter, /**< Invalid parameter provided. */
   BluetoothRetryError, /**< Retry operation timeout error. */
   BluetoothLengthError, /**< Reply length mismatch. */
   BluetoothTXBusy /**< TX is busy, no data can be sent.*/
   } BluetoothError_t;

/**
 * @brief Enum defining states for the Bluetooth state machine.
 */
enum BluetoothState
   {
   getStatus = -10, /**< Initial state for getStatus command. */
   getStatus_2, /**< Secondary state for getStatus command. */
   getMacAddress = -20, /**< Initial state for getMacAddress command. */
   getMacAddress_2, /**< Secondary state for getMacAddress command. */
   setBaudRate = -30,
   setBaudRate_2,
   setBaudRate_3,
   resetModule = -40,
   resetModule_2,
   resetModule_3
   };
/**
 * @brief Enum defining BAUD rates as stated in the datasheet
 */
typedef enum BluetoothBaudRate
   {
   bluetoothBaud_9600 = 0,
   bluetoothBaud_19200,
   bluetoothBaud_38400,
   bluetoothBaud_57600,
   bluetoothBaud_115200,
   bluetoothBaud_4800,
   bluetoothBaud_2400,
   bluetoothBaud_1200,
   bluetoothBaud_230400

   } BluetoothBaudRate_t;

/**
 * @brief Enum defining valid logging types
 *
 * This type is used by ::BluetoothLogEntry identify of which type the pointer is.
 */
typedef enum BluetoothLogEntryType
   {
   BluetoothLogEntryType_bool = 0,
   BluetoothLogEntryType_int8_t,
   BluetoothLogEntryType_uint8_t,
   BluetoothLogEntryType_int16_t,
   BluetoothLogEntryType_uint16_t,
   BluetoothLogEntryType_int32_t,
   BluetoothLogEntryType_uint32_t,
   BluetoothLogEntryType_int64_t,
   BluetoothLogEntryType_uint64_t,
   BluetoothLogEntryType_float,
   BluetoothLogEntryType_double,
   BluetoothLogEntryType_char,
   BluetoothLogEntryType_string
   } BluetoothLogEntryType_t;

/**
 * @brief Type that defines a log entry
 *
 * Every log entry has a name, the type of data it logs and a pointer to that.
 * The name is send every time a connection is etablished. As the pointer to the data is stored in a union, we need to know wwhich type the pointer has.
 */
typedef struct BluetoothLogEntry
   {
      char name[BLUETOOTH_MAX_NAME_LENGTH]; /**< Name of the logged variable*/
      BluetoothLogEntryType_t type; /**< Type of the logged variable*/
      union
         {
            bool *bool_ptr;
            int8_t *int8_ptr;
            uint8_t *uint8_ptr;
            int16_t *int16_ptr;
            uint16_t *uint16_ptr;
            int32_t *int32_ptr;
            uint32_t *uint32_ptr;
            int64_t *int64_ptr;
            uint64_t *uint64_ptr;
            float *float_ptr;
            double *double_ptr;
            char *char_ptr;
            char **string_ptr;

         } data; /**< Pointer to the variable */
   } BluetoothLogEntry_t;

/**
 * @brief Mode of the bluetooth module
 *
 * This type is used for indicating in which mode the bluetooth module is. If it is not connected, it can be configerd,
 * therefore its mode is ::bluetoothConfigure. While beeing connected, every data is transmitted, therefore its mode is ::bluetoothTransmit.
 */
typedef enum bluetoothMode
   {
   bluetoothConfigure = 0, /**< Module not connected, configuration possible*/
   bluetoothTransmit /**< Module connected, transmitting data*/

   } bluetoothMode_t;

/**
 * @brief Structure representing a Bluetooth module instance.
 */
typedef struct BluetoothModule
   {
      const USART_TypeDef *usart; /**< USART instance for communication. */
      int8_t initStatus; /**< Initialization status of the module. */
      BluetoothBaudRate_t baudRate; /**< Baud rate for the USART. */
      char messageBufferRX[USART2_RX_BUFFER_SIZE + 1]; /**< Buffer for received messages. */
      volatile char *messageBufferTX;
      uint16_t available; /**< Number of available characters in the buffer. */
      uint8_t counter; /**< Counter for internal operations, e.g., retry count. */
      int16_t state; /**< Current state of the state machine. */
      volatile bool *TXComplete; /**< TX data finished transmitting @warning Not working yet*/
      bluetoothMode_t mode; /**< Connection status*/
      BluetoothLogEntry_t logEntrys[BLUETOOTH_NUMBER_OF_LOG_ENTRYS]; /**< Array containing all log entrys*/
      bool bluetoothSendLogTitle; /**< Indicates that the log title needs to be send*/

   } BluetoothModule_t;

extern int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *USART,
      BluetoothBaudRate_t baudRate, volatile char txMessageBuffer[]);
extern bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule);
extern int16_t bluetoothGetStatus(BluetoothModule_t *BluetoothModule, bool *isOK);
extern int16_t bluetoothStateHandler(BluetoothModule_t *BluetoothModule, int16_t state);

extern uint32_t bluetoothBaudToInt(BluetoothBaudRate_t baudRate);
extern int16_t bluetoothSetBaudRate(BluetoothModule_t *BluetoothModule, uint8_t fromBaud,
      uint8_t toBaud);
extern uint16_t bluetoothSendLog(BluetoothModule_t *BluetoothModule);
extern int16_t bluetoothResetModule(BluetoothModule_t *BluetoothModule);
extern void bluetoothSendLogTitle(BluetoothModule_t *BluetoothModule);

extern BluetoothError_t dmacUsartSendString(BluetoothModule_t *BluetoothModule, char *data);
extern DMA_Stream_TypeDef* dmacGetStreamFromUSARTTX(USART_TypeDef *usart);

/**
 * @brief Global buffer for USART2 communication.
 *
 * This buffer is used by the USART2_IRQHandler() to store incoming characters
 */
extern volatile char usart2BufferRX[USART2_RX_BUFFER_SIZE + 1];

/**
 * @brief Global UART2 TX buffer
 *
 * The buffer where the TX message is stored. The message is then send via DMA over USART2.
 * The buffer size is calculated regarding the maximum log size, which is 20 bytes (printed int64) plus 1 byte (semicolon)
 * times the number of entrys (::BLUETOOTH_NUMBER_OF_LOG_ENTRYS) plus two bytes (\\n and \0).
 */
#if ((20 + 1) * BLUETOOTH_NUMBER_OF_LOG_ENTRYS + 2) > USART2_MIN_TX_BUFFER_SIZE
extern volatile char usart2BufferTX[ (20 + 1) * BLUETOOTH_NUMBER_OF_LOG_ENTRYS + 2];
#else
extern volatile char usart2BufferTX[USART2_MIN_TX_BUFFER_SIZE];
#warning Using USART2_MIN_TX_BUFFER_SIZE as buffer size
#endif

/**
 * @brief Index for the USART2 buffer.
 *
 * Indicates the next position in the buffer to write to.
 */
extern volatile uint16_t usart2BufferIndex;

/**
 * @brief USART2 Transfer finish
 *
 * Indicates that transfer over USART2 with DMA is finishd
 *
 * @note Interrupt doesn't work, so his does nothing
 */
extern volatile bool usart2TXComplete;

#endif /* BLUETOOTH_H_ */
