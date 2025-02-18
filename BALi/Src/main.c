
/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Prof Flaemig
 * @brief          : Balancer main Function
 ******************************************************************************
 * @attention
 * Functions for Closed Loop Control for balancing two wheel demo
 * Copyright (c) 2025 Prof T Flämig.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************

 */
#define SwVersion "DHBW Bala-V1.2a(c)Fl"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
//#include <stm32f4xx.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <mcalADC.h>
#include <ST7735.h>
#include <RotaryPushButton.h>
#include <BALO.h>
#include <i2cMPU.h>
#include <i2cAMIS.h>
#include <regler.h>
#include "route.h"

#include "i2cDevices.h"


/* uncomment the following line #define oszi
 * for display pitzch value
*/
//#define Oszi


bool timerTrigger = false;


// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	DispTaskTimer = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    I2C_Timer = 0UL;

#ifdef Oszi
	#define StepTaskTime 20
#else
	#define StepTaskTime 7			// the communicatin to stepper takes <1ms therefore total StepTaskTime = 7ms
#endif
#define DispTaskTime 700			// Task for Postion control and Display Status


/* Private function prototypes -----------------------------------------------*/
void test_ascii_screen(void);
void test_graphics(void);
void StepperFollowsPitch(bool StepLenable, bool StepRenable);
void dispMPUBat(MPU6050_t* MPU1, analogCh_t* pADChn);


uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr);
#define i2cAddr_RFID	0x50
#define i2cAddr_LIDAR	0x29
bool enableRFID = false;
bool enableLIDAR = false;


/*------------ MPU6050 Sensor -----------------*/

MPU6050_t MPU1;



/* ------ Def and Parameter for stepper motors ------ */

#define i2cAddr_motL 0x61
#define i2cAddr_motR 0x60

struct Stepper StepL, StepR;
const uint8_t iHold = 6;			// Stiffnes of Axis to Body rotation
const int16_t rad2step =  520;		// Ratio step-counts (200 Full-Steps div 1/16 Steps) per rotation at rad:  509.4 =  200* 16 / (2 PI) or 1600/PI
//#define stepPosMax  200

#define StepPaCount 5
char StepPaTitle[StepPaCount][5] = {"iRun", "iHold",	"vMin",	 "vMax",	"accel"};
uint8_t StepPaValue[StepPaCount] =  { 15, 		7, 		2, 		5,  	4 };		//Parameterset for DEKI Motor 35mm length
const uint8_t stepMode = 3;
const bool stepRotDir = true;

/**
 * void StepperIHold(bool OnSwitch)
 * @param  OnSwitch == true;  IHold active;
 *					== false; IHold reduced to minimum
 * @returns ---
 */
void StepperIHold(bool OnSwitch)
{
	static bool OldStatus = false;
	const uint8_t iOff = 0x01;
	if (OnSwitch != OldStatus)			// commands only active of OnSwitch Status changed
	{
		if (OnSwitch)
		{
			stepper.iHold.set(&StepL, StepPaValue[2]);
			stepper.iHold.set(&StepR, StepPaValue[2]);
			setRotaryColor(LED_YELLOW);
		}
		else
		{
			setRotaryColor(LED_RED);
			stepper.iHold.set(&StepL, iOff);
			stepper.iHold.set(&StepR, iOff);
		}
		OldStatus = OnSwitch;
	}
}
/* Balancer assistant routines and parameters
 *
 */

#define ParamCount 13

enum
{
							a_Cour =0,	a_DEBG, a_posTol,a_phiZ,	a_GyAc,	a_HwLP,	a_LP,	a_piKP,	a_piKI,	a_piKD,	a_raRo,	a_maRo,	a_raTr
} argParam;



char ParamTitle[ParamCount][5] ={"Cour", "DEBG", "poTo", "phiZ","GyAc",	"HwLP",		"LP  ",	"piKP",	"piKI",	"piKD", 			"raRo",		"maRo",		"raTr"};
float ParamValue[ParamCount] =  {0,  	0,			0.0, 	-0.004,		0.98, 		5, 		0.36,  	0.75, 	0.058, 	0.27, 		0.002, 		10, 		0.01};
//								{1, 	0, 			0.0, 	0,		0.98, 		5, 		0.36,  	0.5, 	0.056, 	0.27, 		0.01, 		0.02, 		0.0)   //
float ParamScale[ParamCount] = 	{1,  	1, 			0.2,   	500, 	100, 		1,		500, 	100, 	500,  	100,		500, 		1,  		500};			//  increment stepsize is 1/Value


struct Parameter
{
	char Title[5];
	float Value;
	float Min;
	float Max;
	float manInc;
} Param[ParamCount];

struct RegPameter
{
	float KP;				// Proportional Faktor
	float KD;				// Differential Faktor
	float Rot;
} RegPa;


PIDContr_t 	PID_phi; 		// Pitch controll



void SetRegParameter(MPU6050_t* MPUa)
{
	MPUlpbw tableLPFValue[7] = { LPBW_260, LPBW_184, LPBW_94, LPBW_44, LPBW_21, LPBW_10, LPBW_5};
	MPUa->pitchZero = ParamValue[a_phiZ];
	MPUa->swLowPassFilt = ParamValue[a_LP];
	MPUa->pitchFilt = ParamValue[a_GyAc];
	initPID(&PID_phi, ParamValue[a_piKP],ParamValue[a_piKI],ParamValue[a_piKD], 1);
	/*
	PID_phi.KP = ParamValue[4];
	PID_phi.KI = ParamValue[5];
	PID_phi.KD = ParamValue[6];
	 */
	if (ParamValue[a_HwLP] <0 ) { ParamValue[a_HwLP] =0;}
	if (ParamValue[a_HwLP] >6 ) { ParamValue[a_HwLP] =6;}
	MPUa->LowPassFilt = tableLPFValue[(uint)ParamValue[a_HwLP]];
	mpuSetLpFilt(MPUa);
	if (ParamValue[a_Cour] <0 ) { ParamValue[a_Cour] =0;}
	if (ParamValue[a_Cour] > routeNumMax-1) { ParamValue[a_Cour] = routeNumMax-1;}
	routeNum = ParamValue[a_Cour];
	posTol = ParamValue[a_posTol];
}



void ParamEdit()
{
	int ButtPos;
	static int oldButtPos = 0;
	static int modif = 0;
	char strT[32];
	ButtPos = getRotaryPosition();
	if (getRotaryPushButton())
	{
		if (++modif >= ParamCount)		{	modif = 0;	}
		sprintf(strT, "%s :" , ParamTitle[modif]);
		tftPrintColor((char *)strT,10,20,tft_GREEN);
		sprintf(strT, "%+5.3f  ", ParamValue[modif]);
		tftPrintColor((char *)strT,60,20,tft_GREEN);
		ButtPos = (int)ParamScale[modif]*ParamValue[modif];
		oldButtPos = ButtPos;
		setRotaryPosition(ButtPos);

	}

	if (ButtPos != oldButtPos)
	{
		ParamValue[modif] = ((float)ButtPos/ParamScale[modif]);
		sprintf(strT, "%+5.3f  ", ParamValue[modif]);
		tftPrintColor((char *)strT,60,20,tft_YELLOW);
		oldButtPos = ButtPos;
		SetRegParameter(&MPU1);
	}
}




/**
 *
 */
void rotControl(int16_t* setPos, float* targetPos, float phi, int16_t motPosL, int16_t motPosR)
{


	/*static float phi_old =0;
	/int _iTargetPos = ((int)(rad2step)*(RegPa.KP* tan(phi) + (RegPa.KD* (phi - phi_old))));
	phi_old = phi;
	*/
}


int main(void)
{
/**
 *  I2C Variables  */

	uint8_t        scanAddr = 0x7F;  //7Bit Adresse
	I2C_TypeDef   *i2c  = I2C1;
	//I2C_TypeDef   *i2c2  = I2C2;

/**
*	MPU6050 parameter */

	int8_t MPU6050ret=-1;
	uint32_t   i2cTaskTime = 50UL;
	bool MPU6050enable = false;
	//float MPUfilt[3] = {0,0,0};

	bool StepLenable = false;
	bool StepRenable = false;
	bool resetStepL = true,
		 resetStepR = true;


	const float DivTimeTask= 0.01;   // StepTask div PosTask 7ms / 700ms

    uint8_t MotionVar = 0;

	char strX[8],strY[8],strZ[8],strT[32];
/**
 * ADC Measure Battery Voltage
 */
	/*------------ ADC Channels  -----------------*/

	analogCh_t adChn;
	BatStat_t BatStatus;
	adChn.adc = ADC1;



	//float BatVolt = 0;

/**	Menue for the Filter
 *
 */

	int16_t posMotR=0, posMotL=0,
			curMotR, curMotL;
	int16_t deltaTra, deltaRot;

	float 	tarPosL, tarPosR,
			targetTra, targetRot,
			incRot =0,
			rampRot = 0;		// Ramp to increase speed for translation and rotation of the Balancer

	int pxPos, pyPos;
	bool activeMove = false;
	uint16_t tft_color;
	float AlphaBeta[2];

	static uint8_t RunMode = 1;
	static bool RunInit = true;

       // Dies ist das Array, das die Adressen aller Timer-Variablen enthaelt.
       // Auch die Groesse des Arrays wird berechnet.

       uint32_t *timerList[] = { &I2C_Timer, &ST7735_Timer , &DispTaskTimer /*, weitere Timer */ };
       size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);


    //BALOsetup();

    ledActivate();		// at BALO.c
    i2cActivate();		// at BALO.c
    LED_red_on;
    adcActivate();

	//Inits needed for TFT Display
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

    tftPrintColor((char *)"I2C Scanner running \0",0,0,tft_MAGENTA);

    //tftPrint((char *)"Select I2C Connector \0",0,14,0);

    while (1)
    {
	   if (true == timerTrigger)
	   {
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
	   }


	   if (isSystickExpired(I2C_Timer))
	   {
		   systickSetTicktime(&I2C_Timer, i2cTaskTime);
		   //LED_blue_off;
		   switch (RunMode)
		   {
		   	   case 0:  //I2C Scan
		   	   {

		   		   //setting at BALO.c BALOsetup() --> i2cActive
		   		   //i2cSetClkSpd(i2c,  I2C_CLOCK_400);  // for RFID Reader reduced to 200KHz AMIS 400kHz
		   		   //i2cSetClkSpd(i2c2,  I2C_CLOCK_1Mz);  // Sensor runs fast
		   		   BatStatus = getBatVolt(&adChn);
		   		   if (okBat == BatStatus)
		   		   {
		   			   tft_color  = tft_GREEN;
		   		   }
		   		   else
		   		   {
		   			   tft_color  = tft_YELLOW;
		   		   }
		   		   sprintf(strT, "Battery: %3.1f V", adChn.BatVolt);
		   		   tftPrintColor((char *)strT, 0 , 0, tft_color);
		   		   RunMode  = 1;
		   	   }
		   	   break;
		   	   case 1:  //I2C Scan
		   	   {
		   		setRotaryColor(LED_MAGENTA);
		   		   if ( I2C_SCAN(i2c, scanAddr) != 0)
				   {
					   LED_green_off;
					   switch (scanAddr)
					   {
					   	   case i2cAddr_motL:
						   {
							   StepLenable = true;
							   tftPrint((char *)"<-Left STEP\0",0,110,0);
								//StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, 							rotDir, acceleration, securePosition)
							    StepperInit(&StepL, i2c, i2cAddr_motL,StepPaValue[0], StepPaValue[1], StepPaValue[2],StepPaValue[3],stepMode,(uint8_t)!stepRotDir,StepPaValue[4], 0);
							    stepper.pwmFrequency.set(&StepL, 0);

						   }
						   break;
					   	   case i2cAddr_motR:
						   {
							   StepRenable = true;
							   tftPrint((char *)"Right->\0",94,110,0);
								//StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, 							rotDir, acceleration, securePosition)
							   StepperInit(&StepR, i2c, i2cAddr_motR,StepPaValue[0], StepPaValue[1], StepPaValue[2],StepPaValue[3],stepMode,(uint8_t)stepRotDir, StepPaValue[4], 0);
							   stepper.pwmFrequency.set(&StepR, 0);

						   }
						   break;
					   	   case i2cAddr_RFID:
						   {
							   enableRFID = true;
							   tftPrint((char *)"RFID connected \0",0,65,0);
							   RFID_LED(i2c,true);
						   }
						   break;
						   case i2cAddr_LIDAR:
						   {
							   enableLIDAR = true;
							   tftPrint((char *)"TOF/LIDAR\0",0,80,0);
						   }
						   break;
						   case i2cAddr_MPU6050:
						   {
							   MPU6050enable = true;
							   tftPrint((char *)"MPU6050 \0",0,95,0);
						   }
						   break;
					   }
				   }

		   		   if ((scanAddr == 0) && (MPU6050enable))
				   {
					   LED_blue_on;
					   scanAddr = 0x7F;
					   RunMode = 4;
					   //i2cTaskTime = 200;

				   }
				   if ((scanAddr == 0))
				   {
					   scanAddr = 0x7F;
					   if (i2c == I2C1)
					   {
						   i2c = I2C2;
					   }
					   else
					   {
						   i2c = I2C1;
						   tftFillScreen(tft_BLACK);
					   }
				       RunMode = 0;
				   }
				   else
				   {
					   scanAddr -=1;
				   }
				}
		   	    break;
	// 3DG Sensor function
		   	 	case 4:  // 3DGInit Init
		   	 	{
		   	 		if ((MPU6050enable) && (MPU6050ret <0))
					{
		   				MPU6050ret = mpuInit(&MPU1, i2c, i2cAddr_MPU6050, FSCALE_250, ACCEL_2g, LPBW_184, NO_RESTART);
		   			}
		   			else
		   			{ MPU6050ret = 0; }

					if  (MPU6050ret == 0)									// MPU6050 init-procedure finished
					{
						if ((StepRenable)&& (StepLenable))
						{
							RunMode = 8;
							i2cTaskTime = StepTaskTime;								// Tasktime for Stepper Balancing ca 8ms
							RunInit = true;
							LED_blue_off;
							LED_green_on;

						}
						else
						{
							i2cTaskTime = 70;									// Tasktime for display 70ms
							RunMode = 7;
							LED_green_on;
							LED_blue_off;
						}
					}
				}
				break;
		   		case 7:  // read MPU Data
		   		{

		   			sprintf(strT, "%+3.2f", mpuGetTemp(&MPU1));
		   			tftPrint((char *)strT,40,40,0);

		   			//i2cLIS3DH_XYZ(i2c,(int16_t *) XYZraw);
		   			mpuGetAccel(&MPU1);
  					sprintf(strX, "%+6.3f", MPU1.accel[0]);
		   			tftPrint((char *)strX,20,50,0);
		   			sprintf(strY, "%+6.3f", MPU1.accel[1]);
		   			tftPrint((char *)strY,20,60,0);
		   			sprintf(strZ, "%+6.3f", MPU1.accel[2]);
		   			tftPrint((char *)strZ,20,70,0);

		   		 dispMPUBat(&MPU1,&adChn);
					/*if ((timeTMode5--) > 0)

					{
						RunMode = 8;
						tftFillScreen(tft_BLACK);
						tftPrint("T:    MPU6050 (C)23Fl",0,0,0);
						i2cTaskTime = 100;
						LED_blue_off;

					}*/
				}
		   		break;
		   		case 8:  // Stepper Closed loop Control
				{
					if (RunInit)
					{

						tftSetRotation(LANDSCAPE_FLIP);
						tftFillScreen(tft_BLACK);
						tftSetColor(tft_RED, tft_WHITE);
						tftPrint(SwVersion,0,0,0);
						//tftPrint("DHBW BALA %s (c)Fl\0",0,0,0);
						tftSetColor(tft_GREEN, tft_BLACK);
						dispMPUBat(&MPU1,&adChn);
						StepperIHold(true);										//IHold switched on
						StepperResetPosition(&StepL);  		//resetPosition
						StepperResetPosition(&StepR);
						incRot = 0;
						tarPosR = 0;
						SetRegParameter(&MPU1);
						// set MPU assemble
						MPU1.RPY[0]= 2;				// MPU y Axis goes to the front
						MPU1.RPY[1]= 3;				// MPU z-Axis goes to the left side
						MPU1.RPY[2]= -1;			// MPU x-Axis goes down
						MPU1.timebase = (float) (StepTaskTime+1) * 10e-4;  			// CycleTime for calc from Gyro to angle  fitting statt 10-3 wird 10-4 gesetzt
						initPID(&PID_phi, ParamValue[a_piKP],ParamValue[a_piKI],ParamValue[a_piKD], 1);
						RunInit = false;
					}
					gpioResetPin(LED_RED_ADR);
					mpuGetPitch(&MPU1);
					gpioSetPin(LED_RED_ADR);
					AlphaBeta[1] = MPU1.pitch;
					AlphaBeta[0] = MPU1.pitchAccel;

#ifdef Oszi
            // Display angle values on the oscilloscope
            AlBeOszi(AlphaBeta);
#endif

        			gpioResetPin(LED_RED_ADR);
					if (fabs(AlphaBeta[1]) > 0.35)  // tilt angle more than  0.2 pi/4 = 30deg  -shut off Stepper control and reduce the IHold current and power consumption -> save the planet ;-)
					{
						activeMove = false;
						initPID(&PID_phi, ParamValue[a_piKP],ParamValue[a_piKI],ParamValue[a_piKD], 1);
						StepperIHold(false);
						incRot = 0;
						tarPosR = 0;
						tarPosL = 0;
					    resetStepL = true;
					    resetStepR = true;
						routeNum = ParamValue[a_Cour];
						routeStep = 0;
						MotionVar = 0;
						rampRot = 0;
						/*if (AlphaBeta[1] > 0)
						{  tftSetRotation(LANDSCAPE_FLIP);	}
						else
						{  tftSetRotation(LANDSCAPE); }
						*/

					}
					else
					{
						if (fabs((AlphaBeta[1])) < 0.05)
						{
							setRotaryColor(LED_GREEN);
							activeMove = true;
						}
						else
						{
							setRotaryColor(LED_YELLOW);
							StepperIHold(true);
						}
						if (activeMove == true)
						{
							float setPitch = (rad2step)* runPID(&PID_phi, MPU1.pitch);
							if (rampRot < 1)
							{
								rampRot += ParamValue[a_raRo];
							}
						/*	Translation has to be controlled by pitch
						 * if (rampTra < 1)
							{
								rampTra += ParamValue[a_raTr];
							}
						*/
							//setPitch = 0;
							if (StepRenable)
							{
								gpioSetPin(LED_RED_ADR);									// RED LED OFF
								if (!resetStepR)
								{
									curMotR = (float)StepperGetPos(&StepR);							// 1,177ms
								}
								else
								{
									curMotR = 0;
									StepperResetPosition(&StepR);
									resetStepR = false;
								}
								tarPosR = curMotR + incRot*rampRot;
								posMotR = (int16_t)(setPitch + tarPosR);
								StepperSetPos(&StepR, posMotR); //setPosition;
								StepRenable = false;

								gpioResetPin(LED_RED_ADR);									//RED LED ON
							}
							else
							{
								if (!resetStepL)
								{
									curMotL = (float)StepperGetPos(&StepL);							// 1,177ms
								}
								else
								{
									curMotL = 0;
									StepperResetPosition(&StepL);
									resetStepL = false;
								}
								tarPosL = curMotL - incRot*rampRot;
								posMotL = (int16_t)(setPitch + tarPosL);
								StepperSetPos(&StepL, posMotL); //setPosition;
								StepRenable = true;
							}
						}
					}
					ParamEdit();  // run routine if Push Buttom activated
					gpioSetPin(LED_RED_ADR);
				}
				break;
		   		case 9:  // Stepper Position follow the tilt angle
				{
					//i2cTaskTime = StepTaskTime;								// Tasktime for Stepper Control 50ms
					StepperFollowsPitch(StepLenable, StepRenable);
				}
				break;
		   		default:
				{
					RunMode = 0;
				}
		   }  //end switch (RunMode)
	    } // end if(isSystickExpired(I2C_Timer))

		/*
		* Routine for Motion Control
		*/
		if (isSystickExpired(DispTaskTimer))
		{
			systickSetTicktime(&DispTaskTimer, DispTaskTime);   // Reset Disp timer
			if ((activeMove == true) && (RunMode == 8 ))
			{
				switch (MotionVar)
				{
					case 0:  // Set TagetPos
					{
					  targetTra = route[routeNum][routeStep][0];
					  targetRot = route[routeNum][routeStep][1];
					  rampRot = 0;
					  //rampTra = 0;
					  resetStepL = true;
					  resetStepR = true;
					  //incTra = targetTra * DivTimeTask;
					  incRot = targetRot * DivTimeTask;
					  if (incRot > ParamValue[a_maRo]) { incRot = ParamValue[a_maRo];}
					  if (incRot < -ParamValue[a_maRo]) { incRot = -ParamValue[a_maRo];}


					  MotionVar = 1;
					  sprintf(strT, "N%2i,%+5.0f,%+5.0f",routeNum, targetRot, targetTra);
					  pxPos = 4;   pyPos = 40;	  tft_color = tft_YELLOW;
					  tftSetColor(tft_GREEN, tft_BLACK);
					  tftPrint((char *)strT, pxPos, pyPos, 0);


					}
					break;

					case 1:	// wait and check until target arrived
					{

					  if (
							 ((fabs((float)((posMotL+posMotR)/2)- route[routeNum][routeStep][0]) < posTol)||(posTol <= 0)) &&
							 ((fabs((float)(posMotR-posMotL) - route[routeNum][routeStep][1]) < rotTol))
						 )
					  {
						 if (++routeStep >= routeStepMax)
						 {
							 routeStep = 0;
							 dispMPUBat(&MPU1, &adChn);

						 }
						 MotionVar = 0;
					  }  		// End position reached
					  else
					  {

						  deltaTra = (+targetTra - ((posMotL + posMotR))/2);
						  //incTra = (float)deltaTra * DivTimeTask;
						  deltaRot = (+targetRot - (curMotR - curMotL));		// changed to curMotX
						  incRot = (float)deltaRot  * DivTimeTask;
						  if (incRot > ParamValue[a_maRo])
						  { incRot = ParamValue[a_maRo];}
						  if (incRot < -ParamValue[a_maRo])
						  { incRot = -ParamValue[a_maRo];}
					  }

					  sprintf(strT, "S%2i,%+6i,%+6i  ",routeStep,deltaRot, deltaTra);
					  //sprintf(strT, "S%2i,%+5i",routeStep,deltaRot);
					  pxPos = 4;   pyPos = 60;	  tft_color = tft_WHITE;
					  if (ParamValue[a_DEBG] >0)
					  {
						  tftPrintColor((char *)strT, pxPos, pyPos, tft_color);
					  }
					}
					break;
					default:
					{
					  MotionVar = 0;
					}
				}		//end switch (MotionVar)
			}  //end if ((activeMove == true) && (RunMode == 8 ))

			//if ((activeMove == false) && (RunMode == 8 ))

			if ((RunMode == 5)|| ((RunMode == 8)&&(activeMove != true)))
			{
				 dispMPUBat(&MPU1,&adChn);
			}
			if ((MotionVar == 0) && (RunMode == 8 ))
			{
/*
				sprintf(strT, "%+6i", pos_motL);
				pxPos = 0;
				pyPos = 60;
				tft_color = tft_WHITE;
				tftPrintColor((char *)strT, pxPos, pyPos, tft_color);

				sprintf(strT, "%+6i", pos_motR);
				pxPos = ST7735_TFTWIDTH - 30;
				pyPos = 60;
				tft_color = tft_WHITE;
				tftPrintColor((char *)strT, pxPos, pyPos, tft_color);
*/

		   }
		}  // end if (isSystickExpired(DispTaskTimer))
    } //end while
    return 0;
}


void dispMPUBat(MPU6050_t* pMPU1, analogCh_t* pADChn)
{
	//ADC_TypeDef   *adc= ADC1;
	char strT[20];
	float Temp = mpuGetTemp(pMPU1);
	BatStat_t BatStatus;
	BatStatus = getBatVolt(pADChn);

    if (BatStatus == halfBat)
    {
    	tftSetColor(tft_BLACK, tft_YELLOW);
    }
	else
    {
	   //tft_color  = tft_YELLOW;
	   tftSetColor(tft_WHITE, tft_RED);
    }
    if (BatStatus == okBat)
	{
	   tftSetColor(tft_GREEN, tft_BLACK);
	}
	sprintf(strT, "T:%+3.1f Bat:%3.1fV", Temp,pADChn->BatVolt);
	int pxPos = 10;// ST7735_TFTWIDTH/2-10;
	int pyPos = 110;
	tftPrint((char *)strT, pxPos, pyPos,0);
}



/* scanAdr. 7Bit Adresse value
 * return	0 if no device found on scanAdr
 *			if yes  return the scanAdr.
 *			and display on the ST7735 Display
 *
 *
 */



uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr)
{
	uint8_t 	*outString2 = (uint8_t *) "Addr at: \0";
	uint8_t     port, *result;
#define yPosBase 18
	uint8_t foundAddr = 0;
	static int xPos[2] = {0,100};
	static int yPos[2] = {yPosBase, yPosBase};

	if (i2c == I2C1)
    {
	   port = 0;
    }
    else
    {
	   port = 1;
    }
    if (scanAddr == 0)
    {
    yPos[0] = yPosBase;
    yPos[1] = yPosBase;
    }

	foundAddr = i2cFindSlaveAddr(i2c, scanAddr);
	if (yPos[port] == 0)
	{
		tftPrint((char *)outString2,xPos[port],yPos[port],0);
		yPos[port] = 66;
	}
	result = convDecByteToHex(scanAddr);
	if (foundAddr != 0)
	{
		//outString = outString2;
		tftPrint((char *)result,xPos[port],yPos[port],0);
		yPos[port] = (int) 14 + yPos[port];
		if (yPos[port] > 100)
		{
			yPos[port] = yPosBase;
		}
	}
	else
	{
	//	tftPrint((char *)result,xPos,14,0);
	}
	return foundAddr;

}

void StepperFollowsPitch(bool StepLenable, bool StepRenable)
{
	int ButtPos, oldButtPos=0;
	int16_t pos_motR=0, pos_motL=0;
	float AlphaBeta[2];
	char strT[32];
	static bool RunInit = true;
	if (RunInit)
	{
		tftFillScreen(tft_BLACK);
		tftSetColor(tft_RED, tft_WHITE);
		tftPrint("DHBW BALA Tilt (c)Fl\0",0,0,0);
		tftSetColor(tft_GREEN, tft_BLACK);
		StepperIHold(true);										//IHold switched on
		StepperResetPosition(&StepL);  		//resetPosition
		StepperResetPosition(&StepR);
		RunInit = false;
	}

	mpuGetPitch(&MPU1);
	AlphaBeta[1]= MPU1.pitch;


	if (fabs(AlphaBeta[1]) > 0.7)  // tilt angle more than  pi/4 = 45deg  -shut off Stepper control and reduce the IHold current and power consumption -> save the planet ;-)
	{
		StepperIHold(false);
		StepperSoftStop(&StepR);
		StepperSoftStop(&StepL);			//softStop

		//StepperResetPosition(&StepL);  		//resetPosition
		//StepperResetPosition(&StepR);
		//pos_motR = 0;
		//pos_motL = 0;
	}
	else
	{
		if (fabs(AlphaBeta[1]) < 0.05)
		{
			setRotaryColor(LED_GREEN);
		}
		else
		{
			setRotaryColor(LED_YELLOW);
			StepperIHold(true);
			pos_motL =(int16_t)(AlphaBeta[1]*rad2step);
			pos_motR =(int16_t)(AlphaBeta[1]*rad2step);
			if (StepRenable)
			{
				StepperSetPos(&StepR, pos_motR); //setPosition;
				StepRenable = false;
			}
			else
			{
				StepperSetPos(&StepL, pos_motL); //setPosition;
				StepRenable = true;
			}
		}
	}
	ButtPos = getRotaryPosition();
	if (getRotaryPushButton())
	{

		tftPrintInt(ButtPos,120,20,0);
		int PosR = (int)StepperGetPos(&StepR);
		int PosL = (int)StepperGetPos(&StepL);
		sprintf(strT, "%+5i  %+5i", PosL, PosR);
		tftPrintColor((char *)strT,20,60,tft_YELLOW);

	}

	if (ButtPos != oldButtPos)
	{
		/*kFilt = orgkFilt + ((float)ButtPos)/-500;
		if (kFilt < 0.001) {kFilt = 0.001;}
		if (kFilt > 1) {kFilt =1;}

		sprintf(strT, "kFilt %5.3f ", kFilt);
		tftPrint((char *)strT,10,20,0); */
		oldButtPos = ButtPos;
	}
//RunMode = 2;

}



