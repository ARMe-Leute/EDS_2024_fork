/**
 * @file
 * @brief Includes all TypeDefs that are used in the bluetooth library
 */
#ifndef INC_BLUETOOTH_TYPEDEF_H_
#define INC_BLUETOOTH_TYPEDEF_H_

typedef struct BluetoothModule {
// Todo: Variablen und Funktionen
	USART_TypeDef *usart;
	char receivedChar;
	bool charReceived;
	uint16_t timeout

}BluetoothModule_t;

typedef enum
{
    BAUD_9600 = 0,
    BAUD_19200,
	BAUD_38400,
	BAUD_57600,
	BAUD_115200,
	BAUD_4800,
	BAUD_2400,
	BAUD_1200,
	BAUD_230400
} BLUETOOTH_BAUD;




#endif /* INC_BLUETOOTH_TYPEDEF_H_ */
