/**
 ******************************************************************************
 * @file           : main.cpp
 * @author         : Linus Blanke & Christoph Lederbogen
 * @brief		   : This is an example how to use the rotary push button module
 * 					 with STM. This program uses the "RotaryPushButton"-library
 * 					 from the "Balancer" project.
 * @date		   : December 27, 2023
 ******************************************************************************
 */

/* Includes */
#include <cstdint>

#include <RotaryPushButtonCpp.hpp>

/* simple delay function */
void delayMillis(uint16_t delay);


/*
 * @function:	 main
 *
 * @brief:       This is the main function and executes the simple example.
 * 				 It serves as the entry point for program execution.
 */
int main(void)
{
	/* initialize the rotary push button module */
	RotaryPushButton rotaryPushButton = RotaryPushButton();

    /* Loop forever */
	for(;;)
	{
		/* gets the current state of the push button and the current rotary position
		 * (the rotary position is just there to show the functionality and is not used for further purpose) */
		bool pressed = rotaryPushButton.getRotaryPushButton();
		int position = rotaryPushButton.getRotaryPosition();

		/* if the push button was pressed, the led will turn to green for one loop */
		if (pressed == true)
		{
			rotaryPushButton.setRotaryColor(LED_OFF, LED_ON, LED_OFF);
		}
		else
		{
			rotaryPushButton.setRotaryColor(LED_ON, LED_OFF, LED_OFF);
		}

		/* delay of approx. 1 second */
		delayMillis(1000);
	}
}


/*
 * @function:	 delayMillis
 *
 * @brief:       Simple delay function
 *
 * @args:		 uint16_t delay:  delay in milliseconds
 */
void delayMillis(uint16_t delay)
{
	uint16_t i = 0;

	for (; delay > 0; --delay)
	{
		for (i = 0; i < 1245; ++i)
		{
			;
		}
	}
}



