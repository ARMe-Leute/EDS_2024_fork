/**
 * @file bluetooth.h
 * @brief Bluetooth library interface for STM32 using MCAL.
 *
 * This file provides the declarations and configurations necessary to use
 * the HM17 Bluetooth module with the STM32-Nucleo-F401RE board.
 *
 * Copy this file to the `Inc` folder of your project.
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
 * @brief Interval for fetching data from the USART buffer, in milliseconds.
 *
 * This defines how frequently the buffer is polled for new data during Bluetooth communication.
 */
#define BLUETOOTH_FETCH_TIME 50 //ms

/**
 * @brief Size of the USART2 RX buffer based on the fetch interval.
 *
 * The buffer size is calculated to ensure sufficient space for incoming data
 * during each fetch cycle. It must be at least twice the fetch interval to avoid
 * data loss due to overflow.
 *
 * @warning Setting this value too low may result in dropped or corrupted data.
 */
#define USART2_RX_BUFFER_SIZE (2*1200*BLUETOOTH_FETCH_TIME)/1000

/**
 * @brief Minimum size of the USART2 TX buffer used for transmitting data.
 *
 * This defines the smallest buffer size allocated for outgoing Bluetooth messages.
 */
#define USART2_MIN_TX_BUFFER_SIZE 128

/**
 * @brief Transmission interval for Bluetooth data, in milliseconds.
 *
 * This value specifies how often the log is sent over Bluetooth communication.
 */
#define BLUETOOTH_TRANSMIT_TIME 500 //ms

/**
 * @brief Character limit for Bluetooth log entry identifiers.
 *
 * Restricts the maximum length of names assigned to log entries to ensure
 * efficient packet sizes and readable formatting on receiving devices.
 */
#define BLUETOOTH_MAX_NAME_LENGTH 20

/**
 * @brief Maximum number of simultaneous log entries supported.
 *
 * Defines how many distinct variables can be tracked and transmitted
 * through the Bluetooth logging system.
 */
#define BLUETOOTH_NUMBER_OF_LOG_ENTRYS 2

/**
 * @brief GPIO pin definition for Bluetooth module PIO0 control line.
 *
 * Maps the physical connection between the STM32 and the Bluetooth module's
 * programmable I/O pin 0 for hardware-level control functions.
 */
#define BLUETOOTH_PIO0 GPIOA, 4

/**
 * @brief Development and testing configuration flag.
 *
 * When defined, enables diagnostic features including message injection
 * and state simulation for simplified debugging.
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
   BluetoothBusy, /**< Module is processing another command; retry after appropriate delay. */
   BluetoothWrongParameter, /**< Function received invalid or out-of-range parameters. */
   BluetoothRetryError, /**< Operation failed after exhausting maximum retry attempts. */
   BluetoothLengthError, /**< Response length inconsistency detected; possible data corruption. */
   BluetoothTXBusy /**< Transmission hardware busy; new transmission must wait for completion. */
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
   setBaudRate = -30,/**< Initial state for baud rate configuration. */
   setBaudRate_2,/**< Second state for baud rate configuration. */
   setBaudRate_3,/**< Final state for baud rate configuration. */
   resetModule = -40,/**< Initial module reset request state. */
   resetModule_2,/**< Second module reset request state. */
   resetModule_3/**< Final module reset request state. */
   };

/**
 * @brief Supported baud rate configurations for the Bluetooth module.
 *
 * Provides symbolic constants for all valid communication speeds
 * as specified in the HM17 module datasheet.
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
 * @brief Data type specifications for log entry variables.
 *
 * Defines all supported data types that can be monitored and transmitted
 * through the Bluetooth logging system, enabling proper formatting and memory handling.
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
 * @brief Log entry descriptor structure for variable monitoring.
 *
 * Defines a complete entry in the logging system, containing both metadata
 * and a typed pointer to the monitored variable for runtime access.
 */
typedef struct BluetoothLogEntry
   {
      char name[BLUETOOTH_MAX_NAME_LENGTH]; /**< Human-readable identifier for the logged variable. */
      BluetoothLogEntryType_t type; /**< Data type specifier for proper serialization and formatting. */
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

         } data; /**< Type-safe access to the monitored variable. */
   } BluetoothLogEntry_t;

/**
 * @brief Operational modes for the Bluetooth module.
 *
 * Defines the distinct operational states that determine available
 * functionality and communication behavior.
 */
typedef enum bluetoothMode
   {
   bluetoothConfigure = 0, /**< Configuration mode - enabled when disconnected for setup operations. */
   bluetoothTransmit /**< Transmission mode - active when connected for data streaming. */
   } bluetoothMode_t;

/**
 * @brief Complete Bluetooth module instance descriptor.
 *
 * Consolidates all state information, configuration parameters, and
 * communication buffers for a single Bluetooth module instance.
 */
typedef struct BluetoothModule
   {
      const USART_TypeDef *usart; /**< USART instance for communication. */
      int8_t initStatus; /**< Initialization status of the module. */
      BluetoothBaudRate_t baudRate; /**< Baud rate for the USART. */
      char messageBufferRX[USART2_RX_BUFFER_SIZE + 1]; /**< Buffer for received messages. */
      volatile char *messageBufferTX;/**< Pointer to transmission buffer for outgoing data. */
      uint16_t available; /**< Number of available characters in the buffer. */
      uint8_t counter; /**< Counter for internal operations, e.g., retry count. */
      int16_t state; /**< Current state of the state machine. */
      volatile bool *TXComplete; /**< Transmission completion flag. @warning Implementation incomplete. */
      bluetoothMode_t mode; /**< Current operational mode (configure/transmit). */
      BluetoothLogEntry_t logEntrys[BLUETOOTH_NUMBER_OF_LOG_ENTRYS]; /**< Registered variables for monitoring. */
      bool bluetoothSendLogTitle; /**< Flag indicating log headers need transmission. */

   } BluetoothModule_t;

extern int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *USART,
      BluetoothBaudRate_t baudRate, volatile char txMessageBuffer[]);
extern bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule);
extern void bluetoothParser(BluetoothModule_t *BluetoothModule);
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
 * @brief Global buffer for USART2 transmission.
 *
 * Stores outgoing messages for DMA-based transmission. Size is dynamically
 * calculated based on maximum log requirements or minimum threshold.
 *
 * Buffer sizing considers: 20 bytes per value (max print width) + 1 separator byte
 * per entry, multiplied by entry count, plus line termination characters.
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
 * @brief DMA transmission completion flag for USART2.
 *
 * Indicates when a DMA-based transmission has completed.
 *
 * @note Currently not fully implemented - interrupt functionality pending.
 */
extern volatile bool usart2TXComplete;

#endif /* BLUETOOTH_H_ */
