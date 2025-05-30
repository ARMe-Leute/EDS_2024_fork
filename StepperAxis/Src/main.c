/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Prof Flämig
 * @brief          : Main program for control of amis stepper IC for linear axis
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/*
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include <regler.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>

//using I2C Interface
#include <mcalI2C.h>

// ST7735 TFT Display Ansteuerung eines TFT Display ueber SPI.
#include <mcalSPI.h>
#include <ST7735.h>

// used Rot-Push Buttom
#include <RotaryPushButton.h>
#include <BALO.h>


/* declaration for the timer events to schedule the process (-es)
*/
bool timerTrigger = false;

// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	Timer1 = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    I2C_Timer = 0UL;


// StepTask Time for the main process with the period of xx ms
#define StepTaskTime 6


int main(void)
{
	/**
	 *  I2C Variables

		uint8_t        scanAddr = 0x7F;  //7Bit Adresse
		I2C_TypeDef   *i2c  = I2C1;
		I2C_TypeDef   *i2c2  = I2C2;
	*/
	uint32_t   i2cTaskTime = 50UL;

	float AlphaBeta[2];  // Wertepaar


	// This is the Array, of all Timer-Variables
    uint32_t *timerList[] = { &I2C_Timer, &ST7735_Timer /*, additional timer */ };
	// size of the array  are calculated
	size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);

	    BALOsetup();
	    LED_red_on;

	    struct PIDContr PID_Pos;


		// Inits needed for TFT Display
	    // Initialisiert den Systick-Timer
		systickInit(SYSTICK_1MS);
		spiInit();
		tftInitR(INITR_REDTAB);

		//display setup
	    tftSetRotation(LANDSCAPE);
	    tftSetFont((uint8_t *)&SmallFont[0]);
	    tftFillScreen(tft_BLACK);

	    /* initialize the rotary push button module */
	    initRotaryPushButton();

	    systickSetMillis(&I2C_Timer, i2cTaskTime);

	    LED_red_off;
	    tftPrintColor((char *)"StepperAxis \0",0,0,tft_MAGENTA);

	    //initPID(&PID_Pos, 0.5, 0.5, 0.1, (float)0.0001*i2cTaskTime);  // Init der PID-Koeffizienten des Positions-Regler

	    while (1)
	    {
		   if (true == timerTrigger)
		   {
				systickUpdateTimerList((uint32_t *) timerList, arraySize);
		   }
		   if (isSystickExpired(I2C_Timer))
		   {
			   systickSetTicktime(&I2C_Timer, i2cTaskTime);

			   if (getRotaryPushButton() != 0)
			   {
				   setRotaryPosition(0);
				   //initPID(&PID_Pos, 0.5, 0.5, 0.1, (float)0.0001*i2cTaskTime);
			   }

			   AlphaBeta[0] = (float)getRotaryPosition()/50;
			   LED_blue_on;
			   //AlphaBeta[1] = runPID(&PID_Pos,AlphaBeta[0]);
			   LED_blue_off;
			   if ((AlphaBeta[1] < -1) || (AlphaBeta[1] > 1))
			   {
				   setRotaryColor(LED_RED);
			   }
			   else
			   {
				   setRotaryColor(LED_YELLOW);
			   }
			   AlBeOszi(AlphaBeta);


		   } // end if systickexp
	    } //end while
	    return 0;




}
