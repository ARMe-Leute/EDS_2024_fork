/*
 * bluetooth.h
 *
 *  Created on: 11.11.2024
 *      Author:
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include <stm32f401xe.h>
#include <system_stm32f4xx.h>

#include <stdint.h>
#include <stdbool.h>

#include <mcalGPIO.h>
#include <mcalUsart.h>

#include <bluetooth_typedef.h>

extern void bluetoothGetStatus(BluetoothModule_t *BluetoothModule);
extern BluetoothMessageReply_t bluetoothGetAddress(BluetoothModule_t *BluetoothModule);
extern void bluetoothInit(BluetoothModule_t *BluetoothModule,
		USART_TypeDef *USART);
extern void USART2_IRQHandler(void);
extern char* bluetoothReceiveString(BluetoothModule_t *BluetoothModule,
		uint16_t length, uint16_t timeout);
extern char bluetoothReceiveChar(BluetoothModule_t *BluetoothModule);
extern BluetoothMessageReply_t bluetoothParseReply(char *inputString);

extern void delay(uint16_t delay); // For testing purpose

extern uint32_t bluetoothBaud2Int(BLUETOOTH_BAUD BAUD);

extern bool Usart2charReceived;
extern char Usart2receivedChar;

#endif /* INC_BLUETOOTH_H_ */
