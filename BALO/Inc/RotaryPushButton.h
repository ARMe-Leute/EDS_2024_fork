/**
 ******************************************************************************
 * @file    RotaryPushButton.h
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

#ifndef ROTARYPUSHBUTTON_H
#define ROTARYPUSHBUTTON_H

/* Includes */
#include <stdbool.h>
#include <mcalEXTI.h>

#define ERROR_VALUE 		(int16_t)-65535


typedef struct LEDio LEDio_t; //! ******* Predeclare LED IO-Interface ********************

typedef struct PuBio PuBio_t; //! ******* Predeclare RotPushButtom IO-Interface ********************

/**
* @brief IO-Interface for RGB LEDs
* using C with a struct as OOP (object oriented program) style
*
*/
struct LEDio
{
	 GPIO_TypeDef* PORT;		//! LED IO-Port
	 PIN_NUM_t red;					//! RED
	 PIN_NUM_t green;				//! GREEN
	 PIN_NUM_t blue;				//! BLUE
};




/**
* @brief IO-Interface for a 3 wire Rotation Push Buttom
* using C with a struct as OOP (object oriented program) style
*
*/
struct PuBio
{
	 GPIO_TypeDef* PORT_AB;			//! Rot-Buttom IO-Port
	 PIN_NUM_t PinA;				//! Encoder Signal A
	 EXTI_IRQ_NUM RotA_IRQ;
	 IRQn_Type RotA_IRQn;
	 PIN_NUM_t PinB;				//! Encoder Signal B
	 GPIO_TypeDef* PORT_SW;			//! Push-Buttom IO-Port
	 PIN_NUM_t PinSW;				//! Switch
	 EXTI_IRQ_NUM RotSW_IRQ;
	 IRQn_Type RotSW_IRQn;
	 int16_t ccw;					//! orientation counterclockwise or clockwise
};



/**
 *  Enum for the state of the RGB-LEDs */
typedef enum
{
	RED_off = 0,
	RED_on  = 1,
	GREEN_on = 2,
	BLUE_off = 3,
	BLUE_on = 4,
	GREEN_off = 5,

	BLACK 	= 8,  //! LED is BLACK
	RED  	= 9,  //! LED is RED
	GREEN  	= 10,  //! LED is GREEN
	YELLOW 	= 11,  //! LED is
	BLUE 	= 12,  //! LED is BLUE
	MAGENTA = 13,  //! LED is
	CYAN	= 14,  //! LED is
	WHITE  	= 15,  //! LED is WHITE
} LED_COLOR_t;

/**
 *  Initializes the whole rotary push button module
 *  */
extern void initRotaryPushButton(const PuBio_t* pPuBioset);
/**
* @brief exsamples of IO-connected RotPushButtom
*
*/
extern const PuBio_t PuBio_pgb, PuBio_bala;


/**
 *	@function initLED
 *  @brief Initializes the LEDs
 *  */
extern void initLED(const LEDio_t* _pLED);
/**
 * @brief configurations for Piggyback and Balancer
 */
extern const LEDio_t LEDpgb, LEDbala;

/* Sets the color of the RGB LED in the rotary encoder*/
extern void setLED(LED_COLOR_t color);

/* Gets the state of the push button */
extern int getRotaryPushButton(void);

/* Gets the rotary encoder position */
extern int getRotaryPosition(void);


extern uint16_t setRotaryPosition(int32_t position);

#endif /* ROTARYPUSHBUTTON_H */
