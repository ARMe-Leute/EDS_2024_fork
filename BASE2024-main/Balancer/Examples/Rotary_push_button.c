/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Linus Blanke & Christoph Lederbogen
 * @brief		   : This is an example how to use the rotary push button module
 * 					 with STM. This program uses the "RotaryPushButton"-library
 * 					 from the "Balancer" project.
 * @date		   : December 27, 2023
 ******************************************************************************
 */

/* Includes */
#include <stdint.h>
#include <stdbool.h>

#include <RotaryPushButton.h>

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
	initRotaryPushButton();

	// define variables
	bool pressed = false;
	int position = 0;
	int oldPosition = 0;

    /* Loop forever */
	for(;;)
	{
		/* gets the current state of the push button and the current rotary position
		 * (the rotary position is just there to show the functionality and is not used for further purpose) */
		pressed = getRotaryPushButton();
		oldPosition = position;
		position = getRotaryPosition();

		/* if the push button was pressed, the led will turn to green for one loop */
		if (pressed == true)
		{
			setRotaryColor(LED_GREEN);
		}
		else
		{
			setRotaryColor(LED_RED);
		}

		//check rot direction
		if (position > oldPosition)
		{
			setRotaryColor(LED_MAGENTA);
		}
		else if(position < oldPosition)
		{
			setRotaryColor(LED_CYAN);
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



