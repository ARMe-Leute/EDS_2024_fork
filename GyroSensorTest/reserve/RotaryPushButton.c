/**
 ******************************************************************************
 * @file           : RotaryPushButton.c
 * @author         : Linus Blanke & Christoph Lederbogen
 * @brief          : This is the C source code for controlling the rotary encoder with a built-in push button.
 *              	 It includes the implementations of the functions defined in "RotaryPushButton.h."
 *              	 Additionally, this source code contains the implementation of the interrupt functions.
 *              	 This library is based on the CMSIS and MCAL library.
 * @date		   : December 27, 2023
 ******************************************************************************
 */


/* Includes */
#include <stdint.h>
#include <stdbool.h>

#include <stm32f4xx.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalEXTI.h>
#include <mcalTimer.h>
#include <mcalRCC.h>
#include "BALO.h"
#include "RotaryPushButton.h"


/* Global variable to store the current position of the rotary encoder */
int32_t rotoryPosition = 0;

/* Global flag to indicate the state of the push button */
bool pushButtonFlag = false;

/* init bool variables */
bool initRotaryPushButtonDone = false;
bool initRotaryPushButtonLEDDone = false;


/*
 * @function:	 initRotaryPushButton
 *
 * @brief: 		 Initializes the Rotary Push Button.
 *               This function disables interrupts, configures GPIO pins,
 *               and sets up EXTI and NVIC for interrupt handling.
 *               It enables interrupts before returning.
 *               this function must be called to be able to use the rotary push button module.
 */
void initRotaryPushButton()
{
	/* Initialize GPIO ports for Rotary Push Button (usage of MCAL-Function) */
	gpioInitPort(GPIOA);
	gpioInitPort(GPIOB);
	gpioInitPort(GPIOC);
	gpioInitPort(GPIOD);

	/* Disable interrupts */
	__disable_irq();

	/* Enable clock for System Configuration (SYSCFG) */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* Rotary pin configuration: ROT A */
	gpioSelectPinMode(ROTARY_A_ADR, INPUT);
	gpioSelectPushPullMode(ROTARY_A_ADR, PULLUP);

	/* Rotary pin configuration: ROT B */
	gpioSelectPinMode(ROTARY_B_ADR, INPUT);
	gpioSelectPushPullMode(ROTARY_B_ADR, PULLUP);

	/* Push-button pin configuration */
	gpioSelectPinMode(ROTARY_SW_ADR, INPUT);
	gpioSelectPushPullMode(ROTARY_SW_ADR, PULLUP);

	/* Configure EXTI (external interrupt) */
	extiInit();
	extiConfigIrq(ROTARY_A_ADR);	//EXTI on ROT A
	extiConfigIrq(ROTARY_SW_ADR);	//EXTI on PUSH BUTTON
	extiEnableIrq(ROTARY_A_IRQ);
	extiEnableIrq(ROTARY_SW_IRQ);
	extiSetTriggerEdge(ROTARY_A_IRQ, FALLING_EDGE);
	extiSetTriggerEdge(ROTARY_SW_IRQ, RISING_EDGE);

	/* Enable EXTI and NVIC interrupts */
	NVIC_EnableIRQ(ROTARY_A_IRQn);
	NVIC_EnableIRQ(ROTARY_SW_IRQn);

	/* Enable all interrupts */
	__enable_irq();

	initRotaryPushButtonDone = true;
}

/*
 * @function:	 initRotaryPushButtonLED
 *
 * @brief: 		 Initializes the LEDs of the Rotary Push Button.
 *               After initializing the LEDs of the Rotary Push Button can be used.
 */
void initRotaryPushButtonLED(void)
{
	/* Initialize GPIO ports for Rotary Push Button (usage of MCAL-Function) */
	gpioInitPort(GPIOA);
	gpioInitPort(GPIOB);
	gpioInitPort(GPIOC);
	gpioInitPort(GPIOD);

	/* RED LED pin configuration */
	gpioSelectPinMode(LED_RED_ADR, OUTPUT);

	/* GREEN LED pin configuration */
	gpioSelectPinMode(LED_GREEN_ADR, OUTPUT);

	/* BLUE LED pin configuration */
	gpioSelectPinMode(LED_BLUE_ADR, OUTPUT);

	/* Turn off all LEDs initially */
	gpioSetPin(LED_RED_ADR);
	gpioSetPin(LED_GREEN_ADR);
	gpioSetPin(LED_BLUE_ADR);

	initRotaryPushButtonLEDDone = true;
}


/*
 * @function:	 setRotaryColor
 *
 * @brief: 		 Sets the color of the RGB LED in the Rotary Encoder module.
 *        		 The color is controlled by specifying the state of each color
 *        		 component (red, green, blue).
 *
 * @args:		 LED_OUTPUT_t color:	 color for LED
 */
void setRotaryColor(LED_COLOR_t color)
{
	if (initRotaryPushButtonLEDDone == false)
	{
		initRotaryPushButtonLED();
	}

	switch(color)
	{
		/* Set LED color to BLACK */
		case LED_BLACK:
			gpioSetPin(LED_RED_ADR);
			gpioSetPin(LED_GREEN_ADR);
			gpioSetPin(LED_BLUE_ADR);
			break;

		/* Set LED color to RED */
		case LED_RED:
			gpioResetPin(LED_RED_ADR);
			gpioSetPin(LED_GREEN_ADR);
			gpioSetPin(LED_BLUE_ADR);
			break;

		/* Set LED color to GREEN */
		case LED_GREEN:
			gpioSetPin(LED_RED_ADR);
			gpioResetPin(LED_GREEN_ADR);
			gpioSetPin(LED_BLUE_ADR);
			break;

		/* Set LED color to BLUE */
		case LED_BLUE:
			gpioSetPin(LED_RED_ADR);
			gpioSetPin(LED_GREEN_ADR);
			gpioResetPin(LED_BLUE_ADR);
			break;

		/* Set LED color to MAGENTA */
		case LED_MAGENTA:
			gpioResetPin(LED_RED_ADR);
			gpioSetPin(LED_GREEN_ADR);
			gpioResetPin(LED_BLUE_ADR);
			break;

		/* Set LED color to CYAN */
		case LED_CYAN:
			gpioSetPin(LED_RED_ADR);
			gpioResetPin(LED_GREEN_ADR);
			gpioResetPin(LED_BLUE_ADR);
			break;

		/* Set LED color to YELLOW */
		case LED_YELLOW:
			gpioResetPin(LED_RED_ADR);
			gpioResetPin(LED_GREEN_ADR);
			gpioSetPin(LED_BLUE_ADR);
			break;

		/* Set LED color to WHITE */
		case LED_WHITE:
			gpioResetPin(LED_RED_ADR);
			gpioResetPin(LED_GREEN_ADR);
			gpioResetPin(LED_BLUE_ADR);
			break;
	}
}


/*
 * @function:	 getRotaryPushButton
 *
 * @brief: 		 Returns the state of the Push Button.
 *        		 Returns true if the button is pressed, false otherwise.
 *        		 Additionally, it clears the internal flag if the button was pressed.
 */
int getRotaryPushButton()
{
	if (initRotaryPushButtonDone == false)
	{
		initRotaryPushButton();
		return ERROR_VALUE;
	}

	/* Copy the current state of the push button */
	int pushButton = (int)pushButtonFlag;

	/* clear the internal flag if it was pressed */
	if (pushButtonFlag == true)
	{
		pushButtonFlag = false;
	}

	/* Returns true if the button is pressed, false otherwise*/
	return pushButton;
}


/*
 * @function:	 getRotaryPosition
 *
 * @brief: 		 Gets the rotary encoder position.
 *        		 Returns the current position as an integer.
 */
int getRotaryPosition()
{
	if (initRotaryPushButtonDone == false)
	{
		return ERROR_VALUE;
	}

	/* Returns the current position as an integer.*/
	return rotoryPosition;
}

uint16_t setRotaryPosition(int32_t position)
{
	if (initRotaryPushButtonDone == false)
	{
		return ERROR_VALUE;
	}

	rotoryPosition = position;

	return 0;
}


/*
 * @function:	 ROTARY_A_HANDLER
 * 				 (the correct handler is linked as a define according to the used controller)
 *
 * @brief: 		 EXTI Interrupt Handler for ROT A.
 *               Checks ROT B and updates the position accordingly.
 */





void ROTARY_A_HANDLER(void)
{
	/* Check ROT B and update position */
	if (gpioGetPinState(ROTARY_B_ADR) == true)
	{
		rotoryPosition += 1;
	}
	else
	{
		rotoryPosition -= 1;
	}

	/* Clear the interrupt flag for ROT A */
	if (EXTI->PR & ROTARY_A_PR)
	{
		EXTI->PR |= ROTARY_A_PR;
	}
}


/*
 * @function:	 ROTARY_SW_HANDLER
 * 				 (the correct handler is linked as a define according to the used controller)
 *
 * @brief: 		 EXTI Interrupt Handler for PUSH BUTTON.
 *        		 Sets the PUSH BUTTON flag.
 */
void ROTARY_SW_HANDLER(void)
{
	/* Set PUSH BUTTON flag */
	pushButtonFlag = true;

	/* Clear the interrupt flag for PUSH BUTTON */
	if (EXTI->PR & ROTARY_SW_PR)
	{
		EXTI->PR |= ROTARY_SW_PR;
	}
}
