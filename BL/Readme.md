# Bluetooth Library Using MCAL

This library facilitates seamless communication between an STM32-Nucleo-F401RE microcontroller and an HM17 Bluetooth Low Energy (BLE) module. It's designed around a non-blocking finite state machine architecture that enables reliable communication while maintaining responsive system operation.

Key features include:

 - AT command implementation for comprehensive module configuration
 - Event-driven state machine for non-blocking operations
 - Real-time data logging capabilities for monitoring and debugging
 - Interactive menu system for module configuration and testing
 - Support for various baud rates with visual configuration wizard

@warning The HM17 module's datasheet contains significant inconsistencies and errors that can lead to implementation failures. Exercise caution when developing with this module:
- **Command Response Mismatches:** At least some documented responses don't match actual module behavior.
    - Example: AT+ADDR? is documented to return OK+ADDR:12345678910 but actually returns OK+Get:12345678910
    - Always verify actual responses rather than assuming datasheet accuracy
- **Firmware Version Confusion:** The datasheet references features requiring "firmware version > 500," but
    - the installed HM17 firmware version is 112 (with latest being 116)
    - Version numbers above 500 actually refer to HM10 and HM11 modules
    - Website confirmation: http://www.jnhuamao.cn/bluetooth.asp?id=1
- **Development Recommendations:**
    - Test each command and document actual responses before implementation
    - Use logic analyzer to capture exact response strings
    - Try different bluetooth module type

---

## Architecture

This library implements a finite state machine (FSM) approach. Instead of blocking functions for extended periods, each function performs a single task, returns, and resumes execution later. For this to work, the system requires timers to ensure regular function calls. Refer to `main.c` for an example of the recommended setup.

@note Functions must be called multiple times to progress through their states.

With each invocation, the function transitions to the next state. Return values guide subsequent actions:

- A return value of `0` or `::BluetoothFinish` indicates the function has completed, and its results are ready for use.
- A positive return value signals an error, as defined in `::BluetoothError`.

---

## Logging
The logging functionality enables real-time transmission of application variables from the microcontroller to a connected Bluetooth device. This is especially useful for monitoring, debugging, and data collection during runtime.

### What can be done with the logging functionality?

- **Log variables of various types:**  
  Supported types include:
  - Boolean (`bool`)
  - Signed and unsigned integers of 8, 16, 32, and 64 bits (`int8_t`, `uint8_t`, `int16_t`, `uint16_t`, `int32_t`, `uint32_t`, `int64_t`, `uint64_t`)
  - Floating point (`float`, `double`)
  - Single characters (`char`)
  - Strings (up to 20 characters)

- **Log multiple variables at once:**  
  The maximum number of logged variables is set by `::BLUETOOTH_NUMBER_OF_LOG_ENTRYS`. This value is configurable at compile time and determines the number of variables that can be transmitted in each log message.

- **Customizable logging interval:**  
  The interval at which data is logged and transmitted is defined by `::BLUETOOTH_TRANSMIT_TIME` (default: 500 ms). Both the number of variables and the interval should be chosen according to the selected baud rate, as higher data rates allow more frequent and larger transmissions.

### Logging Format

- **CSV output:**  
  Data is transmitted in CSV (Comma-Separated Values) format for easy import into spreadsheet or analysis tools.  
  - **Header:** When a device connects, a header line with variable names is sent automatically.
  - **Data lines:** On each logging interval, a new line with the current values is sent.

- **Formatting details:**  
  - Values are separated by semicolons (`;`).
  - Decimal numbers use a dot (`.`) as the decimal separator.
  - Each line ends with a newline character (`\n`).
  - Example:
    ```
    Runtime;Rot-Pos;
    500;5;
    1000;6;
    1500;8;
    ```
### Adding a Logging Entry

To add variables to the Bluetooth data logging system:

1. **Define variables to be logged**  
   Ensure these variables remain accessible for the entire logging period.

2. **Set the number of entries**  
   Adjust `::BLUETOOTH_NUMBER_OF_LOG_ENTRYS` in your configuration to match the number of variables you wish to log.

3. **Configure each entry in the `::HM17_1.logEntrys[]` array:**  
   Assign a name, select the correct type, and provide a pointer to the variable:
```c
strcpy(HM17_1.logEntrys[0].name, "VariableName");  // Assign descriptive name
HM17_1.logEntrys[0].type = BluetoothLogEntryType_uint32_t;  // Set data type
HM17_1.logEntrys[0].data.uint32_ptr = &myVariable;  // Connect to variable
```
Supported types are selected from the `::BluetoothLogEntryType` enum, e.g.:
- `::BluetoothLogEntryType_int32_t`
- `::BluetoothLogEntryType_float`
- `::BluetoothLogEntryType_string`
- etc.

4. **Automatic transmission:**  
- The header (variable names) is automatically sent when a device connects.
- Values are transmitted at the interval defined by `::BLUETOOTH_TRANSMIT_TIME`.
- Logging is only active when the module is in transmit mode (i.e., when a Bluetooth device is connected).


### Limitations

- **String length:**  
Logged strings are limited to 20 characters (including the null terminator).
- **Message size:**  
The total size of a log message is limited by the TX buffer size, which is calculated to accommodate the maximum number of entries and their maximum possible length.
- **Interval and variable count:**  
The number of variables and the logging interval must be chosen so that all data can be transmitted within the available bandwidth. If too many variables are logged or the interval is too short for the selected baud rate, data loss or overflow may occur (not automatically detected).
- **No automatic type or bounds checking:**  
The library does not automatically verify that pointers are valid or that string lengths fit the buffer. Care must be taken to avoid overflows or invalid memory access.

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

## Setup

### Configuring BLE

To enable Bluetooth communication with the HM17 module, you first need to pair your device with the module. Unlike some BLE modules, the HM17 does not provide a virtual serial port by default, so additional software is required to bridge BLE to a serial interface.

One effective solution is [ble-serial](https://github.com/Jakeler/ble-serial), which is compatible with Linux, macOS, and Windows. In our setup, only Windows was tested. Note that, as mentioned in the ble-serial README, the Windows installation requires extra steps. The process can be a bit tricky; in our experience, downloading the .exe installer (rather than the .zip archive) was crucial for a successful setup.

After completing the installation, you should be able to connect to the HM17 module as follows:

```
# Scan for available BLE devices. The HM17 should appear as `DSD TECH`.
ble-scan

# Connect to the module using its MAC address.
ble-serial -d 01:23:45:67:89:10
```

Once connected, a new serial port will appear on your system. You can use any terminal program to interact with the module over this port. We recommend [hterm](https://www.der-hammer.info/pages/terminal.html) due to its rich feature set, including advanced parameter control and the ability to easily save output to a file. This makes it straightforward to log data directly to a CSV file for further analysis.


### Configuring the Board

To use the module with USART2, adjust the solder bridges on the board's underside as follows:

- Bridge `SB63` and `SB62`.
- Remove `SB13` and `SB14`.

@note This disables communication over the Virtual COM port. You may leave `SB13` intact to monitor outgoing messages.

# Menu library

The menu library provides a hierarchical navigation system for embedded interfaces using rotary encoder input and ST7735 TFT displays. Designed for STM32 microcontrollers, it enables creation of complex menu structures.

## Key Features
- **Hierarchical Pages**: Create multi-level menus with parent/child relationships
- **Quadrant Layout**: 2x2 grid system (Top-Left, Top-Right, Bottom-Left, Bottom-Right)
- **Dynamic Highlighting**: Visual feedback for selected menu items
- **Color Customization**: Per-entry color definitions
- **Back Navigation**: Automatic return path handling via `::feldBack` entry

## Adding a Menu Page
1. **Define Page Structure**:
```c
MenuPage_t demoSettingsPage;
```
2. **Configure Page Entries**
```c
demoSettingsPage.TL = &feldBack; // Mandatory back navigation
demoSettingsPage.TR = &topRightEntry;
demoSettingsPage.BL = &bottomLeftEntry;
demoSettingsPage.BR = &bottomRightEntry;
```
3. **Link to Parent Menu**
```c
// In main menu configuration
mainMenu.BR = &demoSettingsEntry;

// Corresponding menu entry
MenuEntry_t demoSettingsEntry = {
.color = tft_BLUE,
.title = "DEMO",
.type = Page,
.page = &demoSettingsPage
};
```
4. **Set Initial Page**
```c
menuManager_1.activePage = &mainMenu;
menuManager_1.activeMode = Page;
```

## Adding an Entry Page
1. **Define Entry Structure**
```c
MenuEntry_t topRightEntry = {
.color = tft_CYAN,
.title = "TR",
.type = Entry,
.page = NULL
};
```
2. **Add Entry Handling**
```c
// In main loop's activeEntry switch-case
if (menuManager_1.activeEntry == &systemInfoEntry) {
    // Display dynamic system information
    tftPrint("Top Right Entry", 0, 50, 0);

    // Handle return to menu
    if (getRotaryPushButton()) {
        menuManager_1.activeMode = Page;
        showMenuPage(&menuManager_1, menuManager_1.currentPosition);
    }
}
```
3. **Configure Refresh Behavior**
```c
// Set faster update rate
// Default: 200ms
systickSetTicktime(&MenuTimer, 100); // 100ms refresh
```
## Best Practices

- **Always include `::feldBack` in subpages**
- **Limit menu titles to 10 characters**
- **Avoid Circular Menu Structures:**
 The menu system's navigation history can be corrupted by circular references. The current implementation stores only the immediate parent page in `::MenuPage_t.lastMenu`. Circular structures like:
 `Main Menu → Menu1 → Menu2 → Menu1` will not work.

# Known Issues and possible improvements

## Baud Rate Configuration

- When increasing the HM17 module's baud rate from the default 9600 to 115200, communication initially works. However, after a restart, the HM17 stops responding to AT commands and appears to default to 1200 baud, as described in the datasheet. This requires a module reset to restore normal operation

- After switching to 1200 baud, attempts to set a higher baud rate are acknowledged by the module, but it continues to operate at 1200 baud. Even after resetting or power cycling, the baud rate does not change, indicating a possible firmware or hardware limitation or an undocumented behavior in the module

## USART2 Transmission Complete Flag
- The ::usart2TXComplete flag does not function as expected. The UART transmission complete interrupt does not fire, preventing reliable detection of completed transmissions

- **Workaround:** Currently, the implementation avoids checking this flag and proceeds without confirmation of transmission completion.


## Buffer Management
- The function ::dmacUsartSendString lacks input validation for buffer boundaries. If the input string exceeds the allocated buffer size, a buffer overflow may occur, potentially leading to undefined behavior or memory corruption.

- **Potential Risk:** Sending strings longer than the buffer size will overwrite adjacent memory, which can cause system instability.

**Suggested Improvement:** \
Implement length checking in ::dmacUsartSendString to ensure that the input string does not exceed the buffer capacity. If the input is too long, either truncate the string or return an error code. This will prevent buffer overflows and improve system robustness.


## Button Handling Issue
- Occasionally, the button remains registered as pressed, causing the system to skip steps. This issue is unlikely to be caused by switch bouncing, as the button state is checked at 200 ms intervals.

## RX Buffer Handling
- The current implementation fetches the RX buffer by copying data, which is inefficient.

- Proposed Solution: Instead of copying, store a pointer to the original buffer (as is done for the TX buffer). Use a boolean flag to indicate whether new characters have been received or if a message was terminated with `\n`.

**Suggested Improvement:** \
Refactor the RX buffer logic to use buffer pointers and flags, reducing memory usage and processing time. This approach will also simplify the logic for detecting complete messages and buffer availability.

## Log Buffer Design
- **Idea:** Implement a log buffer (e.g., with 100 entries) to store log messages in a burst or ring buffer. Transmit the entire log at a trigger event (such as when the balancer angle exceeds 5°).

- **Benefit:** This would enable more detailed logging, which is valuable for controller tuning and debugging.

---
PS: If you want something to laugh, have a look at [this](https://docs.qq.com/doc/DUEVrZVhwbWZQdXNm) support document [https://www.deshide.com/product-details.html?pid=1663325&_t=1670572585]

---
This documentation was reviewed and improved with the assistance of ChatGPT.