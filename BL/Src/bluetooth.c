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
 * @brief Handles the state transitions for the Bluetooth state machine.
 *
 * Executes commands by transitioning between the states definded in ::BluetoothState.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param state Initial state for processing. This must be the first step of a procedure.
 * @return Next ::BluetoothState, or a ::BluetoothError code on failure.
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
 * Set the BAUD rate of the module. In the procedure the BAUD rate of the microcontroller is first set to ::fromBAUD.
 * At this speed the command for the new speed is sent. Then the microcontroller is switched to the new address.
 *
 * @warning While teststing, switching the BAUD rate resulted in strange behavior after restarting the HM17. Somehow resetting it got it working again,
 * but I don't know exactly what action resetted it.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param fromBAUD The BAUD rate at which the command is send
 * @param toBAUD The desired BAUD rate
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
 * @brief Send a string over UART with DMA
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @param data The string to be send
 * @return ::BluetoothFinish on success, ::BluetoothTXBusy when previous transmission isn't finished
 *
 * @warning ATM it is not checked, if the TX buffer is large enough. The string to be send MUST be smaller
 *
 * For more details about the process, have a look at R. Jesse, STM32: ARM-Mikrocontroller programmieren für Embedded Systems: das umfassende Praxisbuch. Frechen: mitp, 2021.
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
 * @brief Convert the BAUD rate from ::BluetoothBaudRate to an int
 * @param baudRate The BAUD rate to convert
 * @return The BAUD as an int
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
 * @brief Send variable names of log entrys
 *
 * The title of all logged variables are send, separated with a semicolon and terminated with a newline.
 * This is esentially for the outputted csv file.
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
 * @brief Create a log and send it
 *
 * This function iterates over all log entrys in ::BluetoothModule::logEntrys[], prints the variables to the ::BluetoothModule::messageBufferTX
 * and then sends it. The values are seperated with a semicolon and terminated with a newline-character. This makes it easy to save the raw transmitted
 * log to a csv file that can then further be proccessed (Separator: ';', Decimal separator: '.').
 *
 * @note The magic number 22 comes from the comes from the maximum length a (u)int64_t has in printed form (which is 21) plus the terminator
 * @warning The function does not check if the pointer is a nullpointer. If the pointer doesn't contain anything, it should just output nonesense.
 *
 * @warning There are no checks that make sure that snprintf() went well. As the return value is casted to uint, if an error happend,
 * the next entry is written somwhere into the memory.
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
 * @brief Transfers data from the global USART buffer to the module's buffer.
 *
 * The transfer is completed if either no new characters are received since the last fetch or there is a newline character.
 *
 * @param BluetoothModule Pointer to the ::BluetoothModule instance.
 * @return True if data transfer is complete, false otherwise.
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
 * @brief Return DMA stream for given USART TX
 *
 * @param usart The USART channel
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

#ifndef USART2_BUFFER_SIZE
#warning USART2_BUFFER_SIZE not defined, using 120 Bytes buffer. This may result in lost characters
#endif


/**
 * @brief USART2 interrupt handler for receiving data.
 *
 * Handles incoming data on USART2 and stores it in the global interrupt buffer.
 * The terminator is added after every new character received so that the parsers (e. g. strchr) can parse the data correctly.
 *
 * @warning If the buffer is full, new data is ignored
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
 * @brief DMA1 Stream 6 interrupt handler
 *
 * Handles DMA1 Stream 6 interrupts, which at the moment is the tranfer complete flag
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
