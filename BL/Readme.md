# Bluetooth library using MCAL

This library allows the communication between a STM32-Nucleo-F401RE and a HM17 Bluetooth module.
At the moment, you only can only request a getStatus.

# Architecture
This library uses the approach of a finite state machine. This means that the functions don't block for a long time, instead they do one task, return and continue later. For this to work, you need some timers that guarantee regular calls. You can use ```main.c``` as an example on how it should be constructed.
@warning You need to call functions a couple of times.

With each call the functions enters the next state. The return values indicate the next step. If a 0 or ```::BluetoothFinish``` is returned, then a funtion is finished and the values it modified can be read. A return value greater than 0 indicates an error, as it can be seen in ```#BluetoothError```. 

# Debug mode
This library has a debug mode. This can help you if you would like to use the debugger or test failures.
If you are using a debugger, ```::USART2_IRQHandler()``` wont work as expected anymore. If you halt your code, only the first character that is in the usart line will get received. This would make the debugger unusable, as you never receive a full string. To prevent this problem, you can activate the debug mode. The debug mode allows you to inject strings into the buffer to fake a received string. 
@warning The debug mode disables the IRQHandler so that it won't interfere with the buffer. This also means that you cannot receive anything over usart anymore. 

# Adding a new AT command

Adding a new AT command consists of two major steps:
 - Entry in the stateHandler
 - get/set-function

## stateHandler entry

As the library uses a state-machine approach, you first need to create the needed steps for the state machine in ```::BluetoothState```. The first step gets a value assigned, that is divisible by ten, e. g. ```getMacAddress = -20```. The following steps get the same name with an underscore added like this: ```getMacAddress_2```. You don't need to assign a value to this one, as it gets automatically incrementet by one, which is what we want. Note that a procedure consisting more than ten steps needs a modification deeper in the code in ```::bluetoothStateHandler()```.

Then you can add the corresponding code entrys in ```::bluetoothStateHandler()``` like this:

```
case getMacAddress:
	usartSendString(USART2, (char*) "AT+ADDR?");
	return ++BluetoothModule->state;

case getMacAddress_2:
	BluetoothModule->state = BluetoothFinish; 
	if (BluetoothModule->available) { 
		return BluetoothFinish;
	} else { 
		return BluetoothLengthError;
	}
```
The first step probably consists only of sending the AT command, while in the second on you already can do part of the processing or checking against errors. @warning You should *not* return ```BluetoothModule->state``` in the last step. In the last step, it need's to be set to ```BluetoothFinish```, otherwise no other command can be sent. So make sure that ```BluetoothModule->state``` only has values below one and return a ```::BluetoothError_t```-code.

If you'd like to create a debug mode (which is recommended) you can do this in the first step like this:

```
case getMacAddress:
#ifdef BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_OK
	usart2Buffer[usart2BufferIndex++] = 'O';
	usart2Buffer[usart2BufferIndex++] = 'K';
	usart2Buffer[usart2BufferIndex++] = '+';
	usart2Buffer[usart2BufferIndex++] = '0';
	usart2Buffer[usart2BufferIndex++] = '1';
	usart2Buffer[usart2BufferIndex++] = '2';
	usart2Buffer[usart2BufferIndex++] = '3';
	usart2Buffer[usart2BufferIndex++] = '4';
	usart2Buffer[usart2BufferIndex++] = '5';
	usart2Buffer[usart2BufferIndex++] = '6';
	usart2Buffer[usart2BufferIndex++] = '7';
	usart2Buffer[usart2BufferIndex++] = '8';
	usart2Buffer[usart2BufferIndex++] = '9';
	usart2Buffer[usart2BufferIndex++] = 'A';
	usart2Buffer[usart2BufferIndex++] = 'B';
	usart2Buffer[usart2BufferIndex++] = 'C';

#endif //BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_OK
	usartSendString(USART2, (char*) "AT+ADDR?");
```
It is reccommended to also add one that throws an error. You should also include a check in ```bluetooth.h``` so you get a warning if nothing is injectet. 

```
#if !(defined(BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_ERROR))
#warning "You don't receive any reply, expect the program to be stuck"
#endif // !(defined(BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_ERROR))

```

## Get/Set-function:
This functions can be called outside the library. It calles the stateHandler, and does the work with the received strings.

You can use the following as a blueprint:

```
int16_t bluetoothGetCommand(BluetoothModule_t *BluetoothModule, * size_t reply) {
	int16_t state = bluetoothStateHandler(BluetoothModule, getCommand);
	BluetoothModule->available = 0; // Reset the buffer
	if (state == 0) { 
		/* Do all thing here that need to be done if communication was successfull*/
		*reply = 0;
		return BluetoothFinish; 
	} else if (state < 0) { 
		return state;
	} else { 
		/* Do error handling here if neccessary */
		return state; 
	}
}
```
This things need to be adjustet:
 - ```* size_t reply```: Pointer to what the function "returns".
  As you cannot return two variables, this is the approach. In the parent function, you create a variable with the desired type, e. g. ```uint8_t [8] macAddress;```. You then pass the address the address to the function, e. g. ```bluetoothGetMac(&HM17, &macAddress) ```
 - ```getCommand```: Entry step for the statHandler, e. g. ```getMacAddress```
 - ```if (state == 0)```: Code after all steps are finished.
  Put here the code that needs to be done if all steps are finished successfully. This probably includes reading something of buffer and comparing / manipulating it, e. g. convert the MAC address from chars to ints. In the ```else```-statement goes the error handling, if it needs to be done here, else let it be handeld by the parent function.
 

# Configure the board

To use the module on USART2, you need to adjust the solder bridges on the bottom of the board.

SB63 and SB62 need to be bridged
SB13 and SB14 need to be removed

This disables the possibility to communicate over the Virtual COM port. You can leave SB13 installed to monitor outgoing messages.

