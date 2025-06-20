/**
 ******************************************************************************
 * @file           : RotaryPushButton.h
 * @author         : Linus Blanke & Christoph Lederbogen
 * @brief          : This is the header file for controlling the rotary encoder with a built-in push button.
 *              	 It includes functions for initializing the rotary push button,
 *              	 reading the rotary position and button state, as well as controlling the module's color.
 * @date		   : December 27, 2023
 ******************************************************************************
 */

#ifndef ROTARYPUSHBUTTON_H
#define ROTARYPUSHBUTTON_H

/* Includes */
#include <stdbool.h>
#include <Balancer.h>

#define ERROR_VALUE 		(int16_t)-65535

//Define, if BALA2024 controller is used
#ifdef BALA2024
#define LED_RED_ADR 		GPIOB, PIN15
#define LED_GREEN_ADR		GPIOB, PIN14
#define LED_BLUE_ADR		GPIOB, PIN13

//define rotA adress, IRQ
#define ROTARY_A_ADR		GPIOB, PIN0
#define ROTARY_A_IRQ		EXTI_PIN0
#define ROTARY_A_IRQn		EXTI0_IRQn
#define ROTARY_A_HANDLER 	EXTI0_IRQHandler
#define ROTARY_A_PR			EXTI_PR_PR0

//define rotB adress
#define ROTARY_B_ADR		GPIOB, PIN1

//define switch adress, IRQ
#define ROTARY_SW_ADR		GPIOA, PIN10
#define ROTARY_SW_IRQ		EXTI_PIN10
#define ROTARY_SW_IRQn		EXTI15_10_IRQn
#define ROTARY_SW_HANDLER 	EXTI15_10_IRQHandler
#define ROTARY_SW_PR		EXTI_PR_PR10

//else, if BALA2024 controller is not used
#else
//define LED adresses
#define LED_RED_ADR 		GPIOB, PIN3
#define LED_GREEN_ADR		GPIOB, PIN4
#define LED_BLUE_ADR		GPIOB, PIN10

//define rotA adress, IRQ
#define ROTARY_A_ADR		GPIOC, PIN1
#define ROTARY_A_IRQ		EXTI_PIN1
#define ROTARY_A_IRQn		EXTI1_IRQn
#define ROTARY_A_HANDLER 	EXTI1_IRQHandler
#define ROTARY_A_PR			EXTI_PR_PR1

//define rotB adress
#define ROTARY_B_ADR		GPIOC, PIN0

//define switch adress, IRQ
#define ROTARY_SW_ADR		GPIOA, PIN10
#define ROTARY_SW_IRQ		EXTI_PIN10
#define ROTARY_SW_IRQn		EXTI15_10_IRQn
#define ROTARY_SW_HANDLER 	EXTI15_10_IRQHandler
#define ROTARY_SW_PR		EXTI_PR_PR10
#endif /* BALA2024 */

/* Enum for the state of the LED */
typedef enum
{
	LED_BLACK 	= 0,  // LED is BLACK
	LED_RED  	= 1,  // LED is RED
	LED_GREEN  	= 2,  // LED is GREEN
	LED_BLUE  	= 3,  // LED is BLUE
	LED_MAGENTA = 4,  // LED is MAGENTA
	LED_CYAN  	= 5,  // LED is CYAN
	LED_YELLOW  = 6,  // LED is YELLOW
	LED_WHITE  	= 7   // LED is WHITE
} LED_COLOR_t;

/* Initializes the whole rotary push button module*/
extern void initRotaryPushButton(void);

/* Initializes the LEDs*/
extern void initRotaryPushButtonLED(void);

/* Sets the color of the RGB LED in the rotary encoder*/
extern void setRotaryColor(LED_COLOR_t color);

/* Gets the state of the push button */
extern int getRotaryPushButton(void);

/* Gets the rotary encoder position */
extern int getRotaryPosition(void);


extern uint16_t setRotaryPosition(int32_t position);

#endif /* ROTARYPUSHBUTTON_H */
