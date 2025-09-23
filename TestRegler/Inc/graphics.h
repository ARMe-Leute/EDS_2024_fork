#ifndef GRAPHICS_H_
#define GRAPHICS_H_

/**
 ******************************************************************************
 * @file	graphics.h
 * @author	Prof Flaemig <https://github.com/tflaemig68/>
 * @brief	TFT Graphics V1.0
 * @date	Aug 2025
 ******************************************************************************
 * @attention Graphic function for TFT Display
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 *
 ******************************************************************************
 */

#include <stdint.h>


/**
 * @attention Selection of used Hardware Connectors
 * use "#define BALA2024" for HW DemoBoard Balancer V1.0
 * or
 * use "#define PIGGYBAG" for Board with TFT and Rot-Push-Buttom
 *
 */

//#define BALA2024
#define PIGGYBAG

#define DELAY_red      (900)
#define DELAY_green    (1500)
#define DELAY_blue     (700)


#ifdef BALA2024
// LED_red_pin
#define 	LED_GPIO		GPIOB
#define 	LED_red			PIN15
#define		LED_green		PIN14
#define		LED_blue		PIN13

// default HW PiggyBag with TFT RotBPush Buttom V1.0
#else
#define 	LED_GPIO		GPIOB
#define 	LED_red			PIN3
#define		LED_green		PIN4
#define		LED_blue		PIN10
#endif

#define LED_red_on 		gpioResetPin(LED_GPIO, LED_red)
#define LED_red_off		gpioSetPin(LED_GPIO, LED_red)
#define LED_green_on	gpioResetPin(LED_GPIO, LED_green)
#define LED_green_off	gpioSetPin(LED_GPIO, LED_green)
#define LED_blue_on		gpioResetPin(LED_GPIO, LED_blue)
#define LED_blue_off	gpioSetPin(LED_GPIO, LED_blue)

extern void BALOsetup(void);
extern void i2cActivate(void);


// ******************** Predeclare Oszi Type ********************
typedef struct ScopeFrame Scope_t;

/**
* @brief Scope Frame Structure to display a fast 2 Channel Oszi
* using C with a struct as OOP (object oriented program) style
*/
struct ScopeFrame
{
	int16_t PosY;		//! Oszi Y-Position on TFT Display for Oszi Y = Zero default: 73
	int16_t AmpY;		//! Oszi Y-Amplitude on TFT Display for Oszi Amplitude default: 47
	int16_t TimeX;		//! Oszi Time Length on TFT Display for Oszi  default: 159 (maximum)
	int16_t TimePos;	//! Current Time Pos
	uint16_t ColBG; 	//! BackgroundColor default: tft_GREY;
	uint16_t ColA; 		//! Color of A Channel Line default: tft_YELLOW;
	uint16_t ColB;  	//! Color of A Channel Line default: tft_MAGENTA;
	void (*init)(Scope_t*, int16_t, int16_t, int16_t, uint16_t, uint16_t, uint16_t);  //!< Initialize Oszi
	int16_t (*run)(Scope_t*, float*);   				//!< run Oszi with Input float AB[2]
};

/**
* @brief reference to the prototype of Scope_t
*/
extern const Scope_t Scope;

/**
* @brief reference to non OOP Oszi-function
*/
extern uint16_t AlBeOszi(float *AlphaBeta);

#endif /* GRAPHICS_H_ */
