/**
 ******************************************************************************
 * @file	main.c
 * @author	Prof Flaemig <https://github.com/tflaemig68/>
 * @brief	Example main routine for regler.c V2.1
 * @date	Aug 2025
 ******************************************************************************
 * @attention Functions for Closed Loop Control (PID) and Signal Filter using Recursive Lowpass Function
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 *
 ******************************************************************************
 */
#define SwVersion "PID Demo V2.1 (c)Fl"
#define _KP 0.5
#define _KI 0.5
#define _KD 0.1
#define _OSZIScale 0.8  // Scale of Scope Size

/*
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>

// ST7735 TFT Display Ansteuerung eines TFT Display ueber SPI.
#include <mcalSPI.h>
#include <ST7735.h>

// used Rot-Push Buttom
#include <RotaryPushButton.h>
#include <graphics.h>
#include <regler.h>

/* declaration for the timer events to schedule the process (-es)
*/
bool timerTrigger = false;

// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	Timer1 = 0UL;
uint32_t    ST7735_Timer = 0UL;


int main(void)
{
	const uint32_t   TaskTime = 50UL; // TaskTime for the main process with the period of xx ms
	char strT[32];

	float AlphaBeta[2];  // Calc Values Alpha for Target and Beta for PID out

	// This is the Array, of all Timer-Variables
    uint32_t *timerList[] = { &Timer1, &ST7735_Timer /*, additional timer */ };
	// size of the array  are calculated
	size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);

	HWsetup();		// using this function from graphics.c for the hardware setup
	LED_red_on;			// switch LED to RED

	struct PIDContr PID_Demo;  	// instance for PID Controller
	struct ScopeFrame Scope_Demo;	// Instance for Demo Scope

	// Initialisiert den Systick-Timer
	systickInit(SYSTICK_1MS);

	// Inits needed for TFT Display

	IOspiInit(&ST7735pgb);
	tftInitR(INITR_REDTAB);

	//display setup
	tftSetRotation(LANDSCAPE_FLIP);
	tftSetFont((uint8_t *)&SmallFont[0]);
	tftFillScreen(tft_BLACK);

	// initialize the rotary push button module
	initLED(&LEDpgb);
	initRotaryPushButton(&PuBio_pgb);
	systickSetMillis(&Timer1, TaskTime);	// setting of the Task Timer

	LED_red_off;
	tftPrintColor((char *)SwVersion,0,0,tft_MAGENTA);

	PID.init(&PID_Demo, _KP, _KI, _KD, (float)0.0001*TaskTime);  // Init der PID-Koeffizienten des Positions-Regler

	sprintf(strT, "P:%2.1f I:%2.1f D:%2.1f", PID_Demo.KP,PID_Demo.KI,PID_Demo.KD);
	tftPrintColor((char *)strT, 0 , 14, tft_GREEN);
	Scope.init(&Scope_Demo, _TFTX-Scope.TimeX*_OSZIScale ,_TFTY-Scope.AmpY*_OSZIScale, Scope.AmpY*_OSZIScale, Scope.TimeX*_OSZIScale, Scope.ColBG, Scope.ColA, Scope.ColB);

	while (1)
	{
	   if (true == timerTrigger)
	   {
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
	   }
	   if (isSystickExpired(Timer1))
	   {
		   systickSetTicktime(&Timer1, TaskTime);

		   if (getRotaryPushButton() != 0)
		   {
			   setRotaryPosition(0);
			   PID.init(&PID_Demo, _KD, _KI, _KD, (float)0.0001*TaskTime);
		   }
		   AlphaBeta[0] = (float)getRotaryPosition()/Scope_Demo.AmpY;		// Scale Rot-Pos Value to
		   //setLED(BLUE_on);							// Switch Blue LED ON/OFF for Time Measurement of PID.run
		   AlphaBeta[1] = PID.run(&PID_Demo,AlphaBeta[0]);
		   //setLED(BLUE_off);
		   setLED( (LED_COLOR_t)(fabs(AlphaBeta[1])*7+8));
		   /*if ((AlphaBeta[1] <= -1) || (AlphaBeta[1] >= 1))
		   {
			   setLED(RED);
		   }
		   else
		   {
			   setLED(GREEN_on);
		   }
*/
		   Scope.run(&Scope_Demo, AlphaBeta);

	   } // end if systickexp
	} //end while
	return 0;




}
