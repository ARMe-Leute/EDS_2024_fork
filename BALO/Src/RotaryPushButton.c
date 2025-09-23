/**
 ******************************************************************************
 * @file    RotaryPushButton.c
 * @author  Linus Blanke & Christoph Lederbogen
 * @brief   This is the header file for controlling the rotary encoder with a built-in push button.
 *          It includes functions for initializing the rotary push button,
 *          reading the rotary position and button state, as well as controlling the module's LED-color.
 * @date	December 27, 2023
 * @author	V1.1ff Prof Flaemig <https://github.com/tflaemig68/>
 * @brief 	sorry, until now, there are partial doxygen compatible comments
 * @date	V1.1 Sept. 2025
 ******************************************************************************
 * @attention This software is licensed based on CC BY-NC-SA 4.0
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

#include "RotaryPushButton.h"


static LEDio_t LED;
static PuBio_t RoPuBu;
/**
 *
 Define, LEDs IO-Pinning
					BALA2024	Piggyback
LED_RED	 		GPIOB, PIN15	GPIOB, PIN3
LED_GREEN		GPIOB, PIN14	GPIOB, PIN4
LED_BLUE		GPIOB, PIN13	GPIOB, PIN1
*/
const LEDio_t LEDpgb = {
	.PORT = GPIOB,		//! IO POrt LEDs
	.red = PIN3,		//! Pin red
	.green = PIN4,		//! Pin green
	.blue = PIN10,		//! Pin blue
};

const LEDio_t LEDbala = {
	.PORT = GPIOB,		//! IO POrt LEDs
	.red = PIN15,		//! Pin red
	.green = PIN14,		//! Pin green
	.blue = PIN13,		//! Pin blue
};

const PuBio_t PuBio_pgb= {
	 .PORT_AB = GPIOC,			//! RotPushButtom IO-Port
	 .PinA = PIN0,				//! Encoder Signal A
	 .RotA_IRQ = EXTI_PIN0,
	 .RotA_IRQn = EXTI0_IRQn,
	 .PinB = PIN1,				//! Encoder Signal B
	 .PORT_SW = GPIOA,
	 .PinSW = PIN10,				//! Switch
	 .RotSW_IRQ = EXTI_PIN10,
	 .RotSW_IRQn = EXTI15_10_IRQn,
	 .ccw = 1,						// clockwise or counter clock wise
};

const PuBio_t PuBio_bala= {
	 .PORT_AB = GPIOB,			//! RotPushButtom IO-Port
	 .PinA = PIN0,				//! Encoder Signal A
	 .RotA_IRQ = EXTI_PIN0,
	 .RotA_IRQn = EXTI0_IRQn,
	 .PinB = PIN1,				//! Encoder Signal B
	 .PORT_SW = GPIOA,
	 .PinSW = PIN10,				//! Switch
	 .RotSW_IRQ = EXTI_PIN10,
	 .RotSW_IRQn = EXTI15_10_IRQn,
	 .ccw = 1,
};


int32_t rotoryPosition = 0; //! Global variable to store the current position of the rotary encoder


bool pushButtonFlag = false; //! Global flag to indicate the state of the push button

/**
 *  init bool variables
 */
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
void initRotaryPushButton(const PuBio_t* pPuBioset)
{
	RoPuBu.PORT_AB = pPuBioset->PORT_AB;
	RoPuBu.PinA = pPuBioset->PinA;
	RoPuBu.PinB = pPuBioset->PinB;
	RoPuBu.PORT_SW = pPuBioset->PORT_SW;
	RoPuBu.PinSW   = pPuBioset->PinSW;
	RoPuBu.RotA_IRQ   = pPuBioset->RotA_IRQ;
	RoPuBu.RotA_IRQn   = pPuBioset->RotA_IRQn;
	RoPuBu.RotSW_IRQ = pPuBioset->RotSW_IRQ;
	RoPuBu.RotSW_IRQn = pPuBioset->RotSW_IRQn;
	RoPuBu.ccw   = pPuBioset->ccw;

	/* Initialize GPIO ports for Rotary Push Button (usage of MCAL-Function) */
	gpioInitPort(RoPuBu.PORT_AB);
	gpioInitPort(RoPuBu.PORT_SW);

	// ROTARY_A_ADR = RoPuBu.PORT_AB, RoPuBu.PinA

	/* Disable interrupts */
	__disable_irq();

	/* Enable clock for System Configuration (SYSCFG) */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* Rotary pin configuration: ROT A */
	gpioSelectPinMode(RoPuBu.PORT_AB, RoPuBu.PinA, INPUT);
	gpioSelectPushPullMode(RoPuBu.PORT_AB, RoPuBu.PinA, PULLUP);

	/* Rotary pin configuration: ROT B */
	gpioSelectPinMode(RoPuBu.PORT_AB, RoPuBu.PinB, INPUT);
	gpioSelectPushPullMode(RoPuBu.PORT_AB, RoPuBu.PinB, PULLUP);

	/* Push-button pin configuration */
	gpioSelectPinMode(RoPuBu.PORT_SW, RoPuBu.PinSW, INPUT);
	gpioSelectPushPullMode(RoPuBu.PORT_SW, RoPuBu.PinSW, PULLUP);

	/* Configure EXTI (external interrupt) */
	extiInit();
	extiConfigIrq(RoPuBu.PORT_AB, RoPuBu.PinA);	//EXTI on ROT A
	extiConfigIrq(RoPuBu.PORT_SW, RoPuBu.PinSW);	//EXTI on PUSH BUTTON
	extiEnableIrq(RoPuBu.RotA_IRQ);
	extiEnableIrq(RoPuBu.RotSW_IRQ);
	extiSetTriggerEdge(RoPuBu.RotA_IRQ, FALLING_EDGE);
	extiSetTriggerEdge(RoPuBu.RotSW_IRQ, RISING_EDGE);

	/* Enable EXTI and NVIC interrupts */
	NVIC_EnableIRQ(RoPuBu.RotA_IRQn);
	NVIC_EnableIRQ(RoPuBu.RotSW_IRQn);

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
void initLED(const LEDio_t* pLEDset)
{
	LED.PORT = pLEDset->PORT;
	LED.red = pLEDset->red;
	LED.green = pLEDset->green;
	LED.blue = pLEDset->blue;
	/* Initialize GPIO ports for LED Rotary Push Button (usage of MCAL-Function) */
	gpioInitPort(LED.PORT);

	/* RED LED pin configuration */
	gpioSelectPinMode(LED.PORT,LED.red, OUTPUT);

	/* GREEN LED pin configuration */
	gpioSelectPinMode(LED.PORT,LED.green, OUTPUT);

	/* BLUE LED pin configuration */
	gpioSelectPinMode(LED.PORT,LED.blue, OUTPUT);

	/* Turn off all LEDs initially */
	gpioSetPin(LED.PORT,LED.red);
	gpioSetPin(LED.PORT,LED.green);
	gpioSetPin(LED.PORT,LED.blue);

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
void setLED(LED_COLOR_t color)
{
	if (initRotaryPushButtonLEDDone == false)
	{
		initLED(&LEDpgb);
	}

	switch(color)
	{
		/**
		 *  Set only RED LED */
		case RED_on:
			gpioSetPin(LED.PORT,LED.red);
		break;
		case RED_off:
			gpioResetPin(LED.PORT,LED.red);
		break;

		/**
		 *  Set only LED GREEN */
		case GREEN_on:
			gpioResetPin(LED.PORT,LED.green);
		break;
		case GREEN_off:
					gpioSetPin(LED.PORT,LED.green);
				break;
		/**
		 *  Set only LED BLUE */
		case BLUE_on:
			gpioResetPin(LED.PORT,LED.blue);
		break;
		case BLUE_off:
			gpioSetPin(LED.PORT,LED.blue);
		break;

		/**
		 *  Set LEDs color RGB- matrix mix */
		default:
		{
			if ((color & 0b000001) != 0 )
			{gpioResetPin(LED.PORT,LED.red);}
			else {gpioSetPin(LED.PORT,LED.red);	}
			if ((color & 0b000010) != 0 )
			{gpioResetPin(LED.PORT,LED.green);}
			else {gpioSetPin(LED.PORT,LED.green);}
			if ((color & 0b000100) != 0 )
			{gpioResetPin(LED.PORT,LED.blue);}
			else {gpioSetPin(LED.PORT,LED.blue);}
		}
		break;
	}
}


/**
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
		//initRotaryPushButton();
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


/**
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

/**
 *    @function	 helper for ROTARY_A_HANDLER interrupt
 *    @brief Checks ROT B and updates the position accordingly.
 */
void modifPositionCount(void)
{
	/* Check ROT B and update position */
	if (gpioGetPinState(RoPuBu.PORT_AB, RoPuBu.PinB) == true)
	{
		rotoryPosition += RoPuBu.ccw;
	}
	else
	{
		rotoryPosition -= RoPuBu.ccw;
	}
}

/**
 * @function	 EXTI0_IRQHandler
 * 				  *
 * @brief		 EXTI Interrupt Handler for Pin0
 */
void EXTI0_IRQHandler(void)
{
	modifPositionCount(); //! Check ROT B and update position
	/* Clear the interrupt flag for ROT A */
	if (EXTI->PR & EXTI_PR_PR0)
	{
		EXTI->PR |= EXTI_PR_PR0;
	}
}


/**
 * @function:	 EXTI15_10_IRQHandler
 * 				 *
 * @brief: 		 EXTI Interrupt Handler for PUSH BUTTON.
 *        		 Sets the PUSH BUTTON flag.
 */
void EXTI15_10_IRQHandler(void)
{
	/* Set PUSH BUTTON flag */
	pushButtonFlag = true;
	/* Clear the interrupt flag for PUSH BUTTON */
	if (EXTI->PR & EXTI_PR_PR10)
	{
		EXTI->PR |= EXTI_PR_PR10;
	}
}


