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

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
#include <bluetooth.h>

/**
 * @brief Initializes the Bluetooth module with non-blocking implementation.
 *
 * Establishes communication with the HM17 Bluetooth module by configuring:
 * - USART interface for serial communication
 * - GPIO pins for control signals
 * - DMA for efficient data transfer
 * - Interrupt handlers for asynchronous operation
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param usart Pointer to the USART instance, e.g., USART2.
 * @param baudRate Communication baud rate; the default for HM17 is 9600.
 *  @return 0 on successful initialization, negative value if not finished,
 *  or positive value if there is an error.
 */
int8_t bluetoothInit(BluetoothModule_t *BluetoothModule, USART_TypeDef *usart,
      BluetoothBaudRate_t baudRate, volatile char *txMessageBuffer)
   {

      switch (BluetoothModule->initStatus)
         {
         case -10:
            BluetoothModule->usart = usart;
            BluetoothModule->baudRate = baudRate;
            BluetoothModule->available = 0;
            BluetoothModule->counter = 0;
            BluetoothModule->state = 0;
            BluetoothModule->messageBufferTX = txMessageBuffer;
            BluetoothModule->mode = bluetoothConfigure;

            // Clear the buffer during initialization
            for (uint32_t i = 0; i < USART2_RX_BUFFER_SIZE; i++)
               {
                  BluetoothModule->messageBufferRX[i] = '\0';
               }
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 		// Enable GPIOA clock
            gpioSelectPinMode(BLUETOOTH_PIO0, OUTPUT);
            gpioSetPin(BLUETOOTH_PIO0);
            gpioSelectPinMode(GPIOA, PIN3, ALTFUNC); // PA3: Alternate Function Mode
            gpioSelectAltFunc(GPIOA, PIN3, AF7); 	// PA3: AF7 for USART2 Rx
            gpioSelectPinMode(GPIOA, PIN2, ALTFUNC); // PA2: Alternate Function Mode
            gpioSelectAltFunc(GPIOA, PIN2, AF7); 	// PA2: AF7 for USART2 Tx
            usartSelectUsart(usart);
            usartEnableUsart(usart);
            usartSetCommParams(usart, bluetoothBaudToInt(baudRate), NO_PARITY, LEN_8BIT, ONE_BIT);

            usartSetDmaTxMode(USART2, DMA_TRANSMIT_ON);
            usartResetIrqFlag(USART2, USART_TC_FLG);

            dmacSelectDMAC(DMA1);
            dmacDisableStream(DMA1_Stream6);
            dmacAssignStreamAndChannel(DMA1_Stream6, DMA_CHN_4);
            dmacSetMemoryAddress(DMA1_Stream6, MEM_0, (uint32_t) BluetoothModule->messageBufferTX);
            dmacSetPeripheralAddress(DMA1_Stream6, (uint32_t) &USART2->DR);
            dmacSetDataFlowDirection(DMA1_Stream6, MEM_2_PER);
            dmacSetNumData(DMA1_Stream6, strlen(BluetoothModule->messageBufferTX));

            // Configure data format and increment modes
            dmacSetMemoryDataFormat(DMA1_Stream6, BYTE);
            dmacSetPeripheralDataFormat(DMA1_Stream6, BYTE);
            dmacSetMemoryIncrementMode(DMA1_Stream6, INCR_ENABLE);
            dmacSetPeripheralIncrementMode(DMA1_Stream6, INCR_DISABLE);

            // Set priority and enable transfer complete interrupt
            dmacSetPriorityLevel(DMA1_Stream6, PRIO_MEDIUM);
            dmacEnableInterrupt(DMA1_Stream6, TX_COMPLETE);
            NVIC_EnableIRQ(DMA1_Stream6_IRQn);

            // Clear any pending flags before enabling the stream
            dmacClearAllStreamIrqFlags(DMA1, DMA1_Stream6);

#ifndef debugMode
            usartEnableIrq(usart, USART_IRQ_RXNEIE);
            if (BluetoothModule->usart == USART2)
               {
                  NVIC_EnableIRQ(USART2_IRQn);
                  BluetoothModule->TXComplete = &usart2TXComplete;
               }
            else
               {
                  // Todo: Handle other USART instances if needed
               }
            __enable_irq();
#endif
            *(BluetoothModule->TXComplete) = true;
            return ++BluetoothModule->initStatus;

         case -9:
            if (BluetoothModule->counter > 10)
               { // Return an error after 10 failed attempts
                  return BluetoothRetryError;
               }
            bool reply;
            int16_t status;
            status = bluetoothGetStatus(BluetoothModule, &reply);
            if (status == 0 && reply == true)
               { // HM17 replied with OK
                  return BluetoothModule->initStatus = 0;
               }
            else if (status < 0)
               { // Pending steps
                  return BluetoothModule->initStatus;
               }
            else
               { // Retry on error
                  BluetoothModule->counter++;
                  return BluetoothModule->initStatus - 9;
               }

         default:
            return 0;
         }

   }

/**
 * @brief Resets the Bluetooth module to factory defaults.
 *
 * Performs a hardware reset of the HM17 module by controlling the PIO0 pin
 * according to the module's reset sequence requirements.
 *
 * @param BluetoothModule Pointer to the module configuration structure
 * @return BluetoothFinish on successful reset, or appropriate error code
 *
 * @warning The HM17 datasheet contains contradictory information about the reset procedure.
 * If this reset fails, you might need to adjust the timing or pins.
 * After a reset, disconnect and reconnect power to the HM17 module before continuing.
 */
int16_t bluetoothResetModule(BluetoothModule_t *BluetoothModule)
   {

      int16_t reply = bluetoothStateHandler(BluetoothModule, resetModule);
      if (reply == 0)
         {
            return BluetoothFinish;
         }
      else if (reply < 0)
         {
            return reply;
         }
      else
         {
            return reply;
         }

   }

/**
 * @brief Manages the Bluetooth state machine for non-blocking operations.
 *
 * This is the core of the non-blocking implementation, allowing complex multi-step
 * procedures to execute without halting the main program. It handles state transitions
 * based on the current state and specified operation, executing each step in sequence
 * across multiple function calls.
 *
 * Each procedure (e.g., getting status, setting baud rate) is divided into steps
 * that can be executed incrementally. The function returns after performing a single
 * step, allowing other operations to occur between steps.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param state Initial state for the requested procedure (must be a state divisible by 10)
 * @return Next state value for continuation, ::BluetoothFinish on completion,
 *         or appropriate error code on failure
 */
int16_t bluetoothStateHandler(BluetoothModule_t *BluetoothModule, int16_t state)
   {

      // Ensure it is an initial step
      if (state % 10 != 0)
         {
            return BluetoothWrongParameter;
         }
      if (BluetoothModule->state == 0)
         { // Begin if no state is active
            BluetoothModule->state = state;
            // Ensure the called state matches the current procedure
         }
      else if (!(BluetoothModule->state - state < 10))
         {
            return BluetoothBusy;
         }
      switch (BluetoothModule->state)
         {

         case getStatus:
            dmacUsartSendString(BluetoothModule, "AT");

#ifdef BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK
	usart2BufferRX[usart2BufferIndex++] = 'O';
	usart2BufferRX[usart2BufferIndex++] = 'K';
#endif //BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_OK

#ifdef BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR
	usart2BufferRX[usart2BufferIndex++] = 'E';
	usart2BufferRX[usart2BufferIndex++] = 'R';
	usart2BufferRX[usart2BufferIndex++] = 'R';
	usart2BufferRX[usart2BufferIndex++] = 'O';
	usart2BufferRX[usart2BufferIndex++] = 'R';
#endif //BLUETOOTH_STATE_HANDLER_GET_STATUS_RECEIVE_ERROR

            return ++BluetoothModule->state;

         case getStatus_2:
            BluetoothModule->state = BluetoothFinish; // Mark as finished
            if (BluetoothModule->available >= 2)
               { // Sufficient characters received
                  return BluetoothFinish;
               }
            else
               { // Insufficient data
                  return BluetoothLengthError;
               }
         case setBaudRate:
            usartSetCommParams(BluetoothModule->usart,
                  bluetoothBaudToInt(BluetoothModule->baudRate), NO_PARITY, LEN_8BIT, ONE_BIT);
            return ++BluetoothModule->state;

         case setBaudRate_2:
            char *command = "AT+BAUD ";
            command[7] = BluetoothModule->baudRate + '0';
            dmacUsartSendString(BluetoothModule, command);
            return ++BluetoothModule->state;

         case setBaudRate_3:
            usartSetCommParams(BluetoothModule->usart,
                  bluetoothBaudToInt(BluetoothModule->baudRate), NO_PARITY, LEN_8BIT, ONE_BIT);
            BluetoothModule->state = BluetoothFinish;
            return BluetoothFinish;

         case resetModule:
            gpioSetPin(BLUETOOTH_PIO0);
            return ++BluetoothModule->state;

         case resetModule_2:
            gpioResetPin(BLUETOOTH_PIO0);
            return ++BluetoothModule->state;

         case resetModule_3:
            gpioSetPin(BLUETOOTH_PIO0);
            BluetoothModule->state = BluetoothFinish;
            return BluetoothFinish;

         default:
            return BluetoothWrongParameter;
         }

   }

/**
 * @brief Sends the "AT" command to the Bluetooth module.
 *
 * Sends the "AT" command to the HM17 module and checks for an "OK" response,
 * confirming that the module is properly connected and responsive.
 *
 * This function utilizes the state machine architecture to send the command
 * and process the response asynchronously.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param isOK Pointer to a boolean indicating the command result.
 * @return ::BluetoothFinish on success, or a ::BluetoothError code on failure.
 *
 * @note Check isOK only if the return value is ::BluetoothFinish.
 */
int16_t bluetoothGetStatus(BluetoothModule_t *BluetoothModule, bool *isOK)
   {
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
      if (reply == 0)
         { // We can look for an OK
            *isOK = (strncmp(BluetoothModule->messageBufferRX, "OK", 2) == 0);
            return BluetoothFinish;
         }
      else if (reply < 0)
         {
            return reply; // Continue
         }
      else
         {
            return reply; // Pass error or do error handling here
         }

   }
/**
 * @brief Set the BAUD rate of the bluetooth module
 *
 * This function changes the baud rate of both the microcontroller's USART interface
 * and the HM17 module itself. It follows a three-step process:
 * 1. First, configures the USART to communicate at the current baud rate (::fromBaud)
 * 2. Then sends the command to change the module's baud rate to the target value (::toBaud)
 * 3. Finally, reconfigures the USART to match the new baud rate
 *
 * @warning Changing baud rates may cause communication issues after module restart.
 * If problems occur, try resetting the module to restore proper communication.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param fromBaud Current baud rate enum value for initial communication
 * @param toBaud Target baud rate enum value to set
 * @return ::BluetoothFinish on success, or a ::BluetoothError code on failure.
 */
int16_t bluetoothSetBaudRate(BluetoothModule_t *BluetoothModule, uint8_t fromBaud, uint8_t toBaud)
   {

      /*
       * If ::BluetoothModule::state is 0 means that currently now step is done, which also means that this is the first function call.
       * If we would be in a different step, we would get an error, so this is really the first call that will work.
       */
      if (BluetoothModule->state == 0)
         {
            BluetoothModule->baudRate = fromBaud;
         }
      else
         {
            BluetoothModule->baudRate = toBaud;
         }

      int16_t reply = bluetoothStateHandler(BluetoothModule, setBaudRate);
      if (reply == 0)
         {
            return BluetoothFinish;
         }
      else
         {
            return reply;

         }

   }

/**
 * @brief Transmits a string via USART using DMA
 *
 * Uses Direct Memory Access (DMA) to send data to the USART peripheral,
 * freeing the CPU to perform other tasks during transmission. This function
 * configures the DMA controller to transfer data from the specified buffer
 * to the USART data register automatically.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param data Null-terminated string to transmit
 * @return ::BluetoothFinish on successful transmission start, ::BluetoothTXBusy if previous transmission is still in progress
 *
 * @warning The function does not verify if the destination buffer can hold
 * the entire string. The caller must ensure that data length does not exceed
 * the buffer size defined in the configuration.
 *
 */
BluetoothError_t dmacUsartSendString(BluetoothModule_t *BluetoothModule, char *data)
   {
      if (BluetoothModule->TXComplete == false)
         {
            return BluetoothTXBusy;
         }
      DMA_Stream_TypeDef *dmaStream = dmacGetStreamFromUSARTTX(BluetoothModule->usart);
      strcpy(BluetoothModule->messageBufferTX, data);
      while (dmaStream->CR & DMA_SxCR_EN);

      dmacDisableStream(dmaStream);
      dmacSetMemoryAddress(dmaStream, MEM_0, (uint32_t) BluetoothModule->messageBufferTX);
      dmacSetNumData(dmaStream, strlen(BluetoothModule->messageBufferTX));
      dmacClearAllStreamIrqFlags(DMA1, dmaStream);
      dmacEnableStream(dmaStream);
      // BluetoothModule->TXComplete = false;
      return BluetoothFinish;
   }

/**
 * @brief Converts baud rate enumeration values (::BluetoothBaudRate) to their actual integer rates.
 *
 * Translates the internal enumeration values defined in ::BluetoothBaudRate_t
 * to the corresponding numerical baud rate values needed for USART configuration.
 * @param baudRate Enumeration value representing the desired baud rate
 * @return Actual integer baud rate value (e.g., 9600, 115200), or 0 if invalid
 */
uint32_t bluetoothBaudToInt(BluetoothBaudRate_t baudRate)
   {
      switch (baudRate)
         {
         case bluetoothBaud_9600:
            return 9600;
         case bluetoothBaud_19200:
            return 19200;
         case bluetoothBaud_38400:
            return 38400;
         case bluetoothBaud_57600:
            return 57600;
         case bluetoothBaud_115200:
            return 115200;
         case bluetoothBaud_4800:
            return 4800;
         case bluetoothBaud_2400:
            return 2400;
         case bluetoothBaud_1200:
            return 1200;
         case bluetoothBaud_230400:
            return 230400;
         default:
            return 0;

         }
   }

/**
 * @brief Transmits headers for logged data variables.
 *
 * Prepares and sends a CSV-formatted header line containing the names of all
 * variables that will be logged. This provides column headers for the data
 * that will be sent through subsequent calls to ::bluetoothSendLog.
 *
 * The headers are semicolon-separated (;) and terminated with a newline,
 * making the output directly compatible with CSV format.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 */
void bluetoothSendLogTitle(BluetoothModule_t *BluetoothModule)
   {

      for (uint8_t i = 0; i < BLUETOOTH_NUMBER_OF_LOG_ENTRYS; i++)
         {
            strcat(BluetoothModule->messageBufferTX, BluetoothModule->logEntrys[i].name);
            strcat(BluetoothModule->messageBufferTX, (char*) ";");
         }
      strcat(BluetoothModule->messageBufferTX, (char*) "\n");
      dmacUsartSendString(BluetoothModule, BluetoothModule->messageBufferTX);
   }

/**
 * @brief Formats and transmits log data for all registered variables.
 *
 * Creates a CSV-formatted line containing the current values of all variables
 * registered in the logEntrys array. Each value is converted to a string
 * representation according to its data type, and values are separated by
 * semicolons (;) with a newline terminator.
 *
 * Supports multiple data types including boolean, integers of various sizes,
 * floating point values, characters, and strings.
 *
 * @note The magic number 22 is derived from the maximum length of a formatted 64-bit integer (21 characters) plus
 * the null terminator (`\0`). This ensures sufficient space for the largest possible value representation in decimal form.
 *
 * @warning This function assumes all pointers in logEntrys are valid. Null
 * pointers for string types are handled safely, but other types may cause
 * unexpected behavior if invalid.
 *
 * @warning The return value of `snprintf()` is not checked for errors. If an error occurs during formatting, the cast to `uint16_t`
 * may result in invalid memory writes, potentially corrupting data or causing undefined behavior. This issue arises because
 * `snprintf()` returns a negative value on failure, which is incorrectly treated as an unsigned offset.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @return Number of send bytes
 *
 */
uint16_t bluetoothSendLog(BluetoothModule_t *BluetoothModule)
   {

      uint16_t offset = 0;

      // Iterate over all entrys
      for (uint8_t i = 0; i < BLUETOOTH_NUMBER_OF_LOG_ENTRYS; i++)
         {
            switch (BluetoothModule->logEntrys[i].type)
               {
               case BluetoothLogEntryType_bool:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%s;", *(BluetoothModule->logEntrys[i].data.bool_ptr) ? "true;" : "false;");
                  break;
               case BluetoothLogEntryType_int8_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%d;", *(BluetoothModule->logEntrys[i].data.int8_ptr));
                  break;
               case BluetoothLogEntryType_uint8_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%u;", *(BluetoothModule->logEntrys[i].data.uint8_ptr));
                  break;
               case BluetoothLogEntryType_int16_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%d;", *(BluetoothModule->logEntrys[i].data.int16_ptr));
                  break;
               case BluetoothLogEntryType_uint16_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%u;", *(BluetoothModule->logEntrys[i].data.uint16_ptr));
                  break;
               case BluetoothLogEntryType_int32_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%ld;", *(BluetoothModule->logEntrys[i].data.int32_ptr));
                  break;
               case BluetoothLogEntryType_uint32_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%lu;", *(BluetoothModule->logEntrys[i].data.uint32_ptr));
                  break;
               case BluetoothLogEntryType_int64_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%lld;", *(BluetoothModule->logEntrys[i].data.int64_ptr));
                  break;
               case BluetoothLogEntryType_uint64_t:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%llu;", *(BluetoothModule->logEntrys[i].data.uint64_ptr));
                  break;
               case BluetoothLogEntryType_float: // mcu settings
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%f;", *(BluetoothModule->logEntrys[i].data.float_ptr));
                  break;
               case BluetoothLogEntryType_double:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%f;", *(BluetoothModule->logEntrys[i].data.double_ptr));
                  break;
               case BluetoothLogEntryType_char:
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                        "%c;", *(BluetoothModule->logEntrys[i].data.char_ptr));
                  break;
               case BluetoothLogEntryType_string:
                  // For strings, check for NULL to avoid segmentation fault
                  if (*(BluetoothModule->logEntrys[i].data.string_ptr) != NULL)
                     {
                        offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 22,
                              "%s;", *(BluetoothModule->logEntrys[i].data.string_ptr));
                     }
                  else
                     {
                        offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 2,
                              ";");
                     }
                  break;
               default:
                  // Unknown type, add an empty field
                  offset += (uint16_t) snprintf(BluetoothModule->messageBufferTX + offset, 2, ";");
                  break;
               }

         }
      strcat(BluetoothModule->messageBufferTX, (char*) "\n");
      dmacUsartSendString(BluetoothModule, BluetoothModule->messageBufferTX);
      return offset;
   }

/**
 * @brief Transfers received data from the global interrupt buffer to the module buffer.
 *
 * Safely copies data from the global USART receive buffer (filled by the interrupt
 * handler) to the module's internal message buffer. This transfer is performed
 * when either:
 * - No new characters have been received since the last check, indicating
 *   the transmission has paused or completed
 * - A newline character is detected, indicating a complete message
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @return true if data was transferred and processing can continue,
 *         false if transfer was not performed (waiting for more data)
 */
bool bluetoothFetchBuffer(BluetoothModule_t *BluetoothModule)
   {
      if (BluetoothModule->usart == USART2)
         {
            static uint16_t lastIndex;
            if ((usart2BufferIndex == lastIndex && usart2BufferIndex != 0)
                  || strchr(usart2BufferRX, '\n') != NULL)
               { // Check if no new characters have been received and the buffer is not empty
                  NVIC_DisableIRQ(USART2_IRQn); // Disable USART2 IRQ during data transfer
                  for (uint16_t x = 0; x < usart2BufferIndex; x++)
                     {

                        BluetoothModule->messageBufferRX[BluetoothModule->available] =
                              usart2BufferRX[x];
                        BluetoothModule->available++;
                     }
                  BluetoothModule->messageBufferRX[BluetoothModule->available] = '\0';
                  usart2BufferIndex = 0;
                  NVIC_EnableIRQ(USART2_IRQn);
                  return true;
               }
            else
               {
                  lastIndex = usart2BufferIndex;
                  return false;
               }

         }
      else
         {
            // Todo: Implement support for additional USART instances
            return false;
         }
   }
/**
 * @brief Processes incoming messages from the Bluetooth module.
 *
 * Analyzes the content of the received buffer to identify and handle
 * special commands and connection status changes. The function recognizes:
 * - "OK+CONN" to detect when a Bluetooth connection is established
 * - "OK+LOST" to detect when a connection is terminated
 *
 * When a connection is established, the module transitions to transmission mode
 * and can begin sending log data.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 *
 * @note Custom command parsing can be implemented in the bluetoothTransmit mode
 * section to handle driving commands.
 */
void bluetoothParser(BluetoothModule_t *BluetoothModule)
   {
      if (strstr(BluetoothModule->messageBufferRX, (char*) "OK+CONN") != NULL)
         {
            BluetoothModule->messageBufferTX[0] = '\0';
            BluetoothModule->bluetoothSendLogTitle = true;
            BluetoothModule->mode = bluetoothTransmit;
            BluetoothModule->available = 0;
         }
      if (strstr(BluetoothModule->messageBufferRX, (char*) "OK+LOST") != NULL)
         {
            BluetoothModule->messageBufferTX[0] = '\0';
            BluetoothModule->mode = bluetoothConfigure;
            BluetoothModule->available = 0;
         }
      if (BluetoothModule->mode == bluetoothTransmit)
         {
            /*
             * Do command parsing here
             */
         }
   }

/**
 * @brief Maps USART peripherals to their corresponding DMA streams for transmission.
 *
 * Provides the correct DMA stream for each supported USART peripheral based on
 * the STM32F4 microcontroller's DMA channel mapping. This abstraction allows
 * the library to work with multiple USART interfaces without requiring the user
 * to know the specific hardware connections.
 *
 * Currently supports:
 * - USART1 → DMA2_Stream7
 * - USART2 → DMA1_Stream6
 * - USART6 → DMA2_Stream6
 *
 * @param usart Pointer to the USART peripheral instance
 *
 * @return Pointer to the corresponding DMA stream, or NULL if unsupported
 */
DMA_Stream_TypeDef* dmacGetStreamFromUSARTTX(USART_TypeDef *usart)
   {
      if (usart == USART1)
         {
            return DMA2_Stream7;
         }
      else if (usart == USART2)
         {

            return DMA1_Stream6;
         }
      else if (usart == USART6)
         {
            return DMA2_Stream6;
         }
      else
         {
            return NULL;
         }
   }


/**
 * @brief USART2 interrupt service routine for handling received data.
 *
 * Triggered when data is available in the USART receive data register.
 * Reads the incoming byte and stores it in the global receive buffer,
 * automatically appending a null terminator to maintain a valid string.
 *
 * @warning If the buffer becomes full (reaches USART2_RX_BUFFER_SIZE),
 * additional incoming data will be ignored to prevent buffer overflow.
 */
void USART2_IRQHandler(void)
   {
#ifndef debugMode
      if (USART2->SR & USART_SR_RXNE && usart2BufferIndex < USART2_RX_BUFFER_SIZE)
         {
            usart2BufferRX[usart2BufferIndex++] = USART2->DR & 0xFF; // Ensure 8-bit data
            usart2BufferRX[usart2BufferIndex] = '\0'; // Append terminator
         }
#endif

   }

/**
 * @brief DMA1 Stream 6 interrupt service routine for handling transmit completion.
 *
 * Triggered when the DMA controller has finished transferring data from memory
 * to the USART transmit register. This indicates that the transmission of a
 * message has completed.
 *
 * Sets the usart2TXComplete flag to true, allowing the application to know
 * when it's safe to begin another transmission.
 *
 * @warning Not working yet
 */
void DMA1_Stream6_IRQHandler(void)
   {
      // Check if transfer complete flag is set
      if (dmacGetHighInterruptStatus(DMA1) & (1 << 21)) // Check TC flag for Stream6
         {

            // Clear the transfer complete flag
            usart2TXComplete = true;
            dmacClearInterruptFlag(DMA1, DMA1_Stream6, TX_COMPLETE);

            // Additional processing after transfer completion could go here
         }
   }
