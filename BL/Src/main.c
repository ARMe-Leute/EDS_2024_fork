/**
 * @file main.c
 * @brief  Demo program for the Bluetooth library.
 *
 * This program demonstrates the use of the Bluetooth library. After setup, pressing the button
 * on the rotary encoder lights it up blue while checking the HM17 module's status. If the status
 * is OK, the button lights up green; otherwise, it lights up red. After two seconds, the light turns off.
 *
 * @author c0deberry
 * @author nrs00
 */

#include <stdint.h>
#include <stdbool.h>

#include <mcalSysTick.h>
#include <mcalUsart.h>
#include <mcalGPIO.h>

#include <bluetooth.h>
#include <RotaryPushButton.h>
#include <ST7735.h>

/**
 * @brief Time interval for Bluetooth setup steps (in milliseconds).
 *
 * Each step in the Bluetooth setup process is separated by this interval.
 * The USART pins produce noise when activated, which can confuse the HM17 module.
 * A delay of about 500 ms is recommended to avoid this issue.
 */
#define BLUETOOTH_SETUP_TIME 500 //ms

/**
 * @brief Application modes.
 *
 * Used to differentiate between initialization and the main loop.
 */
typedef enum
    {
    MAIN_INIT = 0,
    MAIN_LOOP
    } MAIN_MODE;

/**
 * @brief Timer trigger flag, updated in the SysTick interrupt handler.
 *
 * This flag indicates that the base time interval has elapsed, and
 * all timers need to be updated.
 */
bool timerTrigger;

volatile char usart2Buffer[USART2_BUFFER_SIZE];
volatile uint16_t usart2BufferIndex = 0;

int main(void)
    {

    uint32_t BluetoothTimer = 0UL; 	// Timer for Bluetooth setup steps.
    uint32_t BluetoothFetchTimer = 0UL; // Timer for calling bluetoothFetchBuffer().
    uint32_t Button = 0UL;// Timer for button polling, helps with debouncing.
    uint32_t ButtonLEDOff = 0UL;	// Timer for turning off the button LED.
    uint32_t *timerList[] =
	{
	&BluetoothTimer, &BluetoothFetchTimer, &Button, &ButtonLEDOff
	};
    uint8_t arraySize = sizeof(timerList) / sizeof(timerList[0]);

    BluetoothModule_t HM17_1;	// Bluetooth module instance.

    MAIN_MODE mode = MAIN_INIT;

    bool buttonPressed = false; // Tracks whether the button is pressed.
    for (;;)
	{
	switch (mode)
	    {
	// Initialization mode
	case MAIN_INIT:
	    systickInit(SYSTICK_1MS); // Configure SysTick with a 1 ms interval.
	    bool setupFinished = false;

	    int8_t init1Status = -127; // Tracks Bluetooth initialization status.
	    HM17_1.initStatus = -10;

	    initRotaryPushButton();
	    initRotaryPushButtonLED();
	    systickSetTicktime(&Button, 20);

	    while (setupFinished == false)
		{
		if (timerTrigger == true)
		    {
		    systickUpdateTimerList((uint32_t*) timerList, arraySize);

		    }
		if (isSystickExpired(BluetoothTimer))
		    {
		    init1Status = bluetoothInit(&HM17_1, USART2, 9600);
		    systickSetTicktime(&BluetoothTimer, BLUETOOTH_SETUP_TIME);
		    }
		if (isSystickExpired(BluetoothFetchTimer))
		    {
		    bluetoothFetchBuffer(&HM17_1);

		    systickSetTicktime(&BluetoothFetchTimer,
		    BLUETOOTH_FETCH_TIME);
		    }

		if (init1Status == 0)
		    {
		    setupFinished = true; // Initialization complete
		    }
		if (init1Status > 0)
		    {
		    setRotaryColor(LED_YELLOW); // Indicate a failed setup
		    }
		}

	    mode = MAIN_LOOP; // Transition to main loop

	    break;

	    // Main application loop
	case MAIN_LOOP:
	    if (timerTrigger == true)
		{
		systickUpdateTimerList((uint32_t*) timerList, arraySize);

		}
	    if (isSystickExpired(BluetoothTimer))
		{

		if (buttonPressed == true)
		    {
		    bool reply;
		    int16_t status;
		    status = bluetoothGetStatus(&HM17_1, &reply);
		    if (status == 0)
			{ // Command successfully executed.
			buttonPressed = false;
			systickSetTicktime(&ButtonLEDOff, 2000); // 2-second timer for LED off.
			if (reply == true)
			    {
			    setRotaryColor(LED_GREEN); // Indicate success

			    }
			else
			    {
			    setRotaryColor(LED_RED);   // Indicate failure
			    }
			}
		    else if (status > 0)
			{
			systickSetTicktime(&ButtonLEDOff, 2000);
			buttonPressed = false;
			setRotaryColor(LED_RED);
			}
		    }

		systickSetTicktime(&BluetoothTimer, 200); // Shorter interval after setup.
		}
	    if (isSystickExpired(BluetoothFetchTimer))
		{
		bluetoothFetchBuffer(&HM17_1);

		systickSetTicktime(&BluetoothFetchTimer,
		BLUETOOTH_FETCH_TIME);
		}
	    if (isSystickExpired(Button))
		{

		if (getRotaryPushButton() == true)
		    {
		    buttonPressed = true;
		    setRotaryColor(LED_BLUE); // Indicate button press.
		    }

		systickSetTicktime(&Button, 20);
		}
	    if (isSystickExpired(ButtonLEDOff))
		{

		setRotaryColor(LED_BLACK); // Turn off LED after timeout.
		}

	    break;

	    }
	}
    }
