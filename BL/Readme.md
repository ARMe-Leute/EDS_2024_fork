# Bluetooth Library Using MCAL

This library facilitates communication between an STM32-Nucleo-F401RE and an HM17 Bluetooth module. Currently, it only supports requesting the `getStatus` command.

---

## Architecture

This library implements a finite state machine (FSM) approach. Instead of blocking functions for extended periods, each function performs a single task, returns, and resumes execution later. For this to work, the system requires timers to ensure regular function calls. Refer to `main.c` for an example of the recommended setup.

@note Functions must be called multiple times to progress through their states.

With each invocation, the function transitions to the next state. Return values guide subsequent actions:

- A return value of `0` or `::BluetoothFinish` indicates the function has completed, and its results are ready for use.
- A positive return value signals an error, as defined in `::BluetoothError`.

---

## Debug Mode

The library includes a debug mode to facilitate debugging and failure testing. When using a debugger, `::USART2_IRQHandler()` may not behave as expected: halting the code will only capture the first character from the USART line, preventing full string reception. To address this, enable debug mode, which allows string injection into the buffer to simulate received strings.

@warning Debug mode disables the IRQ handler to avoid interference with the buffer. Consequently, data cannot be received over USART in this mode.

---

## Adding a new AT Command

Adding a new AT command involves two major steps:

1. Creating an entry in the `::bluetoothStateHandler()`.
2. Implementing the `get` or `set` function.

### Step 1: State Handler Entry

The FSM requires defining the necessary steps for handling the new command in `::BluetoothState`:

1. Assign an initial step value divisible by 10, e.g., `getMacAddress = -20`.
2. Define subsequent steps by appending underscores to the name, e.g., `getMacAddress_2`. These steps do not need explicit values as they are automatically incremented.

@note For procedures requiring more than ten steps, modify `::bluetoothStateHandler()` to accommodate them.

Next, add the corresponding code in `::bluetoothStateHandler()` as follows:

```c
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

- **First step:** Typically sends the AT command.
- **Subsequent steps:** Process the response or handle errors.

@warning Ensure the final step sets `::BluetoothModule->state` to `::BluetoothFinish`. Returning any other value will prevent subsequent commands from executing. Always return a valid `::BluetoothError_t` code in the last step.

To implement debug mode for a command (recommended), you can use a conditional preprocessor directive:

```c
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
#endif
    usartSendString(USART2, (char*) "AT+ADDR?");
```

Also include a safety check in `bluetooth.h` to warn if no response is injected:

```c
#if !(defined(BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_OK) || defined(BLUETOOTH_STATE_HANDLER_GET_MacAddress_RECEIVE_ERROR))
#warning "No response defined. Expect the program to get stuck."
#endif
```

### Step 2: Get/Set Function

These functions are callable from outside the library. They invoke the `::bluetoothStateHandler()` and process the received strings. Use the following blueprint:

```c
int16_t bluetoothGetCommand(BluetoothModule_t *BluetoothModule, size_t *reply) {
    int16_t state = bluetoothStateHandler(BluetoothModule, getCommand);
    BluetoothModule->available = 0; // Reset the buffer
    
    if (state == 0) {
        // Actions upon successful communication
        *reply = 0;
        return BluetoothFinish;
    } else if (state < 0) {
        return state;
    } else {
        // Optional error handling
        return state;
    }
}
```

Adjust the following elements:

- **`size_t *reply`**: Pointer for output variables (e.g., `uint8_t macAddress[8];`). Pass its address to the function (e.g., `bluetoothGetMac(&HM17, &macAddress);`).
- **`getCommand`**: Replace with the entry step of the `stateHandler` (e.g., `getMacAddress`).
- **Successful completion block:** Add code for post-processing results, such as converting the MAC address from characters to integers.

---

## Configuring the Board

To use the module with USART2, adjust the solder bridges on the board's underside as follows:

- Bridge `SB63` and `SB62`.
- Remove `SB13` and `SB14`.

@note This disables communication over the Virtual COM port. You may leave `SB13` intact to monitor outgoing messages.


---
This documentation was reviewed and improved with the assistance of ChatGPT.