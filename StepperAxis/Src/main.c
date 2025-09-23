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
#define SWVerTxt "StAxis1.2 \0"
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
#include <mcalADC.h>
#include "i2cAMIS.h"


#define i2cAddr_mot 0x60
#define i2cAddr_mot1 0x61
struct Stepper Step;
const uint8_t iHold = 6;			// Stiffnes of Axis to Body rotation
const int16_t rad2step =  520;		// Ratio step-counts (200 Full-Steps div 1/16 Steps) per rotation at rad:  509.4 =  200* 16 / (2 PI) or 1600/PI
//#define stepPosMax  200

I2C_TypeDef   *i2c  = I2C1;
#define StepPaCount 5
char StepPaTitle[StepPaCount][5] = {"iRun", "iHold",	"vMin",	 "vMax",	"accel"};
uint8_t StepPaValue[StepPaCount] =  { 15, 		5, 		2, 		9,  	3 };		//Parameterset for NEMEA32 20mm length
const uint8_t stepMode = 0;				// 0 Half; 1 1/4; ..
const bool stepRotDir = true;



typedef enum
{
	ParamInit = 0,
	I2C_Scan,
	ResetPos,
	InitRun,
	AutoRun,
	ManualRun
} TaskModus;


/* declaration for the timer events to schedule the process (-es)
*/
bool timerTrigger = false;

// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	Timer1 = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    Timer100ms = 0UL;

float adcMeas(ADC_TypeDef   *adc)
{

	static float adcValue = 0;
	const uint16_t adcMax = 4095;
	const float kFilt = 0.05;
	uint16_t adcRaw;
	char strT[8];

	if (adcIsConversionFinished(adc))
		{
			adcRaw = adcGetConversionResult(adc);
			adcStartConversion(adc);
			adcValue *= (1-kFilt);
			adcValue += ((float)adcRaw * kFilt/ adcMax);
		}
		//sprintf(strT, "%3.2f", adcValue);
		sprintf(strT, "%2.0f%%", 100*adcValue);
		tftPrintColor((char *)strT,78,0,tft_YELLOW);
		return(adcValue);
}
// TaskRoutine with 100ms cycletime
TaskModus Task100ms(TaskModus RunMode)
{

	static float ADC_offset = 0;
	ADC_TypeDef   *adc    = ADC1;
	const int16_t minStepPos = -32000;
	const int16_t secPos = 00;
	const int16_t manStep = 800;			// viertel Umdrehung
	const float maxStep = 32000;
	static int16_t setPos = 0, setPosOld = 0;
	char strT[16];
	static int texty =1;
	static uint8_t i2cAddr = i2cAddr_mot;
	float ADC_0;
	float AlphaBeta[2];  // Wertepaar
	uint8_t foundAddr = 0;
	switch (RunMode)
    {
   	   case ParamInit:  //ADC Init
   	   {

   		   setRotaryColor(LED_BLUE);
   		   ADC_CHANNEL_t chnList[] = { ADC_CHN_0 };

   		    // Anzahl der Listenelemente berechnen
   		    size_t         listSize = sizeof(chnList) / sizeof(chnList[0]);

   		    adcSelectADC(adc);                     // ADC1: Bustakt aktivieren

   		    // Konfiguration der Sequenz und Eintrag der Laenge von chnList[]
   		    adcSetChannelSequence(adc, chnList, listSize);
   		    adcEnableADC(adc);
   		    adcStartConversion(adc);
   			RunMode = I2C_Scan;
   	   }
   	   break;
   	   case I2C_Scan:
   	   {
   		   foundAddr = i2cFindSlaveAddr(i2c, i2cAddr);
   		   if ((foundAddr == i2cAddr_mot)||( foundAddr == i2cAddr_mot1))
   		   {
   			   tftPrint((char *)"Active\0",110,0,0);
   			   //StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, 							rotDir, acceleration, securePosition)
   			   StepperInit(&Step, i2c, i2cAddr,StepPaValue[0], StepPaValue[1], StepPaValue[2],StepPaValue[3],stepMode,(uint8_t)!stepRotDir,StepPaValue[4], secPos);
   			   stepper.pwmFrequency.set(&Step, 0);

   			RunMode = ResetPos;
   			setRotaryColor(LED_YELLOW);
   		   }
   		   else
   		   {
   			  setRotaryColor(LED_CYAN);
   			  if  ( foundAddr == 0 )
   			  {
   				sprintf(strT, "missed I2C %3x", i2cAddr);
   				tftPrintColor((char *)strT,0,10*texty,tft_CYAN);
   			  }
   			  else
   			  {
   				sprintf(strT, ">found I2C %3x", i2cAddr);
   				tftPrintColor((char *)strT,0,10*texty++,tft_GREEN);
  			  }
   			  if (++i2cAddr > 0x7f) { i2cAddr = 0; texty = 1;}
   		   }
   	   }
   	   break;
   	   case ResetPos: // Set Motorpos from ADC
	   {
			StepperSetPos(&Step, minStepPos);
			RunMode = InitRun;
			setRotaryColor(LED_RED);
			tftPrintColor("INIT  \0",110,0,tft_RED);

	   }
	   break;
	   case InitRun: // Set MotorPos to minimal Pos. for calibration run
	   {
			if (getRotaryPushButton() != 0)
			{
				setRotaryPosition(0);
				setRotaryColor(LED_MAGENTA);
				tftPrintColor("Manual\0",110,0,tft_MAGENTA);
				StepperSoftStop (&Step);
				//StepperResetPosition(&Step);
				RunMode = ManualRun;

			}
			if (StepperGetPos(&Step) == minStepPos)
			{
				setRotaryColor(LED_GREEN);
				tftPrint((char *)"Active\0",110,0,0);
				RunMode = AutoRun;
			}
	   }
	   break;

   	   case AutoRun: // Set Motoraxis-position from ADC (Range 0..4095) from minStepPos to maxStep
	   {
			if (getRotaryPushButton() != 0)
			{
			  setRotaryPosition((int16_t)setPos/manStep);
			  setRotaryColor(LED_MAGENTA);
			  tftPrintColor("Manual\0",110,0,tft_MAGENTA);
			  RunMode = ManualRun;
			}

			ADC_0 = adcMeas(adc);
			setPos = (int16_t)((ADC_0-ADC_offset)*(maxStep-minStepPos))+minStepPos;
			StepperSetPos(&Step, setPos);
			AlphaBeta[0] = ADC_0*2-1;			// AD Value ist display form Minimim -1 till Max +1 OSzi
			AlphaBeta[1] = (float)StepperGetPos(&Step)/maxStep;
			AlBeOszi(AlphaBeta);
	   }
	   break;

   	   case ManualRun: // Reset Motorpos with Manual Rot
   	   {
			if (getRotaryPushButton() != 0)
			{
			  tftPrint((char *)"Active\0",110,0,0);
			  setRotaryColor(LED_GREEN);
			  RunMode = AutoRun;

			}
			setPos = (int16_t)getRotaryPosition()*manStep;
			if (setPos != setPosOld)
			{
				StepperSetPos(&Step, setPos);
				setPosOld = setPos;
				sprintf(strT, "%+6i", setPos);
				tftPrintColor((char *)strT,110,0,tft_MAGENTA);
				//tftPrintLong(setPos, 80, 0, 0);
			}
			ADC_0 = adcMeas(adc);

			AlphaBeta[0] = (float)getRotaryPosition()/40;
			AlphaBeta[1] = (float)StepperGetPos(&Step)/maxStep;
			AlBeOszi(AlphaBeta);

	   }
   	   break;

   	   default:
	   {


	   }


    }
    return(RunMode);
}


int main(void)
{
	/**
	 *  I2C Variables

		uint8_t        scanAddr = 0x7F;  //7Bit Adresse
		I2C_TypeDef   *i2c  = I2C1;
		I2C_TypeDef   *i2c2  = I2C2;
	*/
	uint32_t   TaskTime100ms = 100UL;
	int RunMode = 0;



	// This is the Array, of all Timer-Variables
    uint32_t *timerList[] = { &Timer100ms, &ST7735_Timer /*, additional timer */ };
	// size of the array  are calculated
	size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);

	    BALOsetup();
	    LED_red_on;


	   // struct PIDContr PID_Pos;


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

	    systickSetMillis(&Timer100ms, TaskTime100ms);

	    LED_red_off;
	    tftPrintColor((char *)SWVerTxt,0,0,tft_RED);

	    //initPID(&PID_Pos, 0.5, 0.5, 0.1, (float)0.0001*i2cTaskTime);  // Init der PID-Koeffizienten des Positions-Regler

	    while (1)
	    {
		   if (true == timerTrigger)
		   {
				systickUpdateTimerList((uint32_t *) timerList, arraySize);
		   }
		   if (isSystickExpired(Timer100ms))
		   {
			   systickSetTicktime(&Timer100ms, TaskTime100ms);
			   RunMode= Task100ms(RunMode);
		   }
	    } //end while
	    return 0;




}
