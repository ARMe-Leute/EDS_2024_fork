
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
#define SwVersion "DHBW Bala-V1.5a(c)Fl"
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
#include <i2cTOF.h>
#include <regler.h>
#include "route.h"

#include "i2cDevices.h"


/* uncomment the following line #define oszi
 * for display pitch value
*/
//#define Oszi



bool timerTrigger = false;


// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	DispTaskTimer = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    StepTaskTimer = 0UL;

#ifdef Oszi
	#define StepTaskTimeSet 20
#else
	#define StepTaskTimeSet 7UL			// the communication to stepper takes <1ms therefore total StepTaskTime = 7ms
#endif
#define DispTaskTimeSet 700			// Task for Position control and Display Status

/**
 * typedef for Task-Modes
 *
 */
typedef enum
{
	M_InitBat = 0,
	M_DispMpuData,  // 1
	M_DispTofData,
	M_CheckI2cSlaves,  	//prüfen welche I2C Slaves vorhanden sind
	M_StepFollowPitch,  // open loop control - stepper follows the pitch
	M_3DGinit,			//
	M_Bala,
} TaskModus;

/* --- StepTaskTime[] this are the Array of Task Times ----*
 * in the same order as the above TaskModus
 *
 */

uint32_t   StepTaskTime[] = {50UL, 50UL, 100UL, 50UL, StepTaskTimeSet, StepTaskTimeSet, StepTaskTimeSet};


/* ------- Private function prototypes -----------------------------------------------*/

void StepperFollowsPitch(bool StepLenable, bool StepRenable);		//open Loop demo
void dispMPUBat(MPU6050_t* MPU1, analogCh_t* pADChn);
void DispAlphaNumMPU(MPU6050_t* pMPU);
extern void visualisationTOF(TOFSensor_t* TOFSENS);

/*------------ reserve for RFID Reader -----------------*/
#define i2cAddr_RFID	0x50
bool enableRFID = false;


/*------------ MPU6050 Sensor -----------------*/

MPU6050_t MPU1;




/*-------- TOF (TimeOfFlight)-Sensor VL53X -------------*/

TOFSensor_t TOF1;
bool enableTOF1 = false;
#define i2cAddr_TOF1	0x29

// variables to store the distance -kann vermutlich weg
uint16_t TOF_DISTANCE_1 = 10;


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
	const uint8_t iOff = 0x0;  //switch off the PWM Regulator Value 0xFF only for AMIS
								// 0xFF only for AMIS IC
								// 0x0 reduced current to 59mA

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
							a_MODE =0,	a_Cour, a_posTol,a_phiZ,	a_GyAc,	a_HwLP,	a_LP,	a_piKP,	a_piKI,	a_piKD,	a_raRo,	a_maRo,	a_raTr
} argParam;



char ParamTitle[ParamCount][5] ={"MODE", "Cour", "poTo", 	"phiZ",		"GyAc",		"HwLP",	"LP  ",	"piKP",	"piKI",	"piKD", 		"raRo",		"maRo",		"raTr"};
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
	if (ParamValue[a_HwLP] <0 ) { ParamValue[a_HwLP] =0;}
	if (ParamValue[a_HwLP] >6 ) { ParamValue[a_HwLP] =6;}
	MPUa->LowPassFilt = tableLPFValue[(uint)ParamValue[a_HwLP]];
	mpuSetLpFilt(MPUa);
	if (ParamValue[a_Cour] <0 ) { ParamValue[a_Cour] =0;}
	if (ParamValue[a_Cour] > routeNumMax-1) { ParamValue[a_Cour] = routeNumMax-1;}
	routeNum = ParamValue[a_Cour];
	posTol = ParamValue[a_posTol];
}



void ParamEdit(TaskModus *Mode)
{
	int ButtPos;
	static int oldButtPos = 0;
	static int modif = 0;
	char strT[32];
	ButtPos = getRotaryPosition();
	if (getRotaryPushButton())
	{
		if (++modif >= ParamCount)		{	modif = 0;	}
		if ((modif > 0)&&(ParamValue[a_MODE]>0))
		{
			*Mode = (TaskModus)ParamValue[a_MODE];   // Mode switch
			ParamValue[a_MODE]=0;
		}
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



// *DevMask b0010 Left, b0001 right stepper; b0100 mpu;  b1000 tof/lidar
#define DevStepR 0b0001
#define DevStepL 0b0010
#define DevMPU1  0b0100
#define DevTOF1  0b1000


int CheckAndInitI2cSlaves(uint8_t* DevMask, Stepper_t* pStepL, Stepper_t* pStepR, MPU6050_t* pMPU1,TOFSensor_t* pTOF1)
{
	I2C_TypeDef *i2c = I2C1, *i2c2 = I2C2;
	static I2C_TypeDef *i2cMPU = I2C1;
	static I2C_TypeDef *i2cSTEP = I2C1;
	static I2C_TypeDef *i2cTOF = I2C1;
	uint8_t foundAddr;
	static uint8_t i2c_Addr = 1;
	static int CycleRun = -4;
	int MPU6050ret;

	if (CycleRun == -5)
	{
		foundAddr = i2cFindSlaveAddr(i2c, i2c_Addr); //dummy run
		foundAddr = i2cFindSlaveAddr(i2c2, i2c_Addr); //dummy run
		CycleRun++;
		return (CycleRun);
	}

	if ((( *DevMask & DevStepL) == 0)&& (CycleRun == -4))
	{
		i2c_Addr = i2cAddr_motL;
		foundAddr = i2cFindSlaveAddr(i2cSTEP, i2c_Addr);
		if (foundAddr == 0)
		{
			i2cSTEP = I2C2;
			foundAddr = i2cFindSlaveAddr(i2cSTEP, i2c_Addr);
		}
		if (foundAddr == i2c_Addr)
		{
			//StepLenable = true;
			tftPrint((char *)"<-Left  -OK-  \0",0,110,0);
			//StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, 							rotDir, acceleration, securePosition)
			StepperInit(pStepL, i2cSTEP, i2c_Addr,StepPaValue[0], StepPaValue[1], StepPaValue[2],StepPaValue[3],stepMode,(uint8_t)!stepRotDir,StepPaValue[4], 0);
			stepper.pwmFrequency.set(pStepL, 0);
			*DevMask |= DevStepL;
		}
		else
		{ pStepL->i2cAddress.value = 0; }			// if StepperLeft not exist set pointer to NULL
		CycleRun++;
		return (CycleRun);
	}
	if (( *DevMask & DevStepR) == 0)
	{
		foundAddr = i2cFindSlaveAddr(i2cSTEP, i2cAddr_motR);
		if (foundAddr != 0)
		{
			//  StepRenable = true;
			tftPrint((char *)"Right->\0",104,110,0);
			//StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, 							rotDir, acceleration, securePosition)
			StepperInit(pStepR, i2cSTEP, i2cAddr_motR,StepPaValue[0], StepPaValue[1], StepPaValue[2],StepPaValue[3],stepMode,(uint8_t)stepRotDir, StepPaValue[4], 0);
			stepper.pwmFrequency.set(pStepR, 0);
			*DevMask |= DevStepR;
			i2cSetClkSpd(i2cSTEP,  I2C_CLOCK_200); //speed up I2CBusclock max Stepper 400kHz<- doesn't worked
		}
		else
		{ pStepR->i2cAddress.value = 0; }			// if StepperRight not exist set pointer to NULL
	}

	// TOF check and INIT
	if ((( *DevMask & DevTOF1) == 0)&& (CycleRun == -3))
	{
		TOF1.i2c_tof = i2cTOF;
		TOF1.TOF_address_used = i2cAddr_TOF1;
	}
	if ((( *DevMask & DevTOF1) == 0) && (CycleRun <= -3))
	{
		if (TOF_init_address(&TOF1))
		{
			// Initialisieren des TOF-Sensors
			initTOFSensorData(&TOF1, i2cTOF, TOF_ADDR_VL53LOX, TOF_DEFAULT_MODE_D, TOF_DISTANCE_1);
			tftPrint((char *)"TOF found \0",0,80,0);
			*DevMask |= DevTOF1;
		}
		else
		{
			tftPrint((char *)"TOF not found \0",0,80,0);
		}
	}

	if ((( *DevMask & DevTOF1) != 0) && (CycleRun == -2))
	{

		bool InitResult = TOF_init_device(&TOF1);
		if (InitResult)
		{
			tftPrint((char *)"TOF init OK\0",0,80,0);
			//*DevMask |= DevTOF1;
		}

	}

	if ((( *DevMask & DevTOF1) != 0) && (CycleRun == -1))

	{
		// Konfigurieren und Aktivieren des Sensors
		configTOFSensor(&TOF1, TOF_DEFAULT_MODE_D, true);
		//TOF_set_ranging_profile(&TOF1);
		TOF_start_continuous(&TOF1);
	}


	// MPU6050 check and Init with 3 runs
	if (CycleRun == -3)
	{	// detected and first initrun
		foundAddr = i2cFindSlaveAddr(i2cMPU, i2cAddr_MPU6050);
		if (foundAddr == 0)
		{
			i2cMPU = I2C2;
			foundAddr = i2cFindSlaveAddr(i2cMPU, i2cAddr_MPU6050);
		}
		if (foundAddr != 0)
		{
			tftPrint((char *)"MPU6050 OK \0",0,95,0);
			*DevMask |= DevMPU1;
			//i2cSetClkSpd(i2cMPU,  I2C_CLOCK_1Mz); //speed up sensor Bus
			MPU6050ret = mpuInit(pMPU1, i2cMPU, i2cAddr_MPU6050, FSCALE_250, ACCEL_2g, LPBW_184, NO_RESTART);
			CycleRun = MPU6050ret;
		}
		else
		{	pMPU1->i2c_address = 0;	}
		return (CycleRun);
	}
	if (((*DevMask & DevMPU1) >0) && (CycleRun < 0))
	{
		MPU6050ret = mpuInit(pMPU1, i2cMPU, i2cAddr_MPU6050, FSCALE_250, ACCEL_2g, LPBW_184, NO_RESTART);
		CycleRun = MPU6050ret;
		return (CycleRun);
	}
	return (CycleRun);
}


int main(void)
{
	uint8_t DevPrMask = 0;			// Mask Presents Devices

	int I2cCheckResult;
	//float MPUfilt[3] = {0,0,0};

	bool StepLenable = false;
	bool StepRenable = false;
	bool resetStepL = true,
		 resetStepR = true;
	TaskModus TaskMode = M_InitBat;

	const float DivTimeTask= 0.01;   // StepTask div PosTask 7ms / 700ms

    uint8_t MotionVar = 0;


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
	//uint16_t tft_color;
	float AlphaBeta[2];
	char strT[32];
	//static uint8_t RunMode = 1;
	static bool RunInit = true;



       // Dies ist das Array, das die Adressen aller Timer-Variablen enthaelt.
       // Auch die Groesse des Arrays wird berechnet.

       uint32_t *timerList[] = { &StepTaskTimer, &ST7735_Timer , &DispTaskTimer /*, weitere Timer */ };
       size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);


    //BALOsetup();

    ledActivate();		// at BALO.c
    i2cActivate();		// at BALO.c
    LED_red_on;
    adcActivate();

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

    systickSetMillis(&StepTaskTimer, StepTaskTime[M_InitBat]);


    LED_red_off;

    tftPrintColor((char *)"I2C Scanner running \0",0,0,tft_MAGENTA);

    //tftPrint((char *)"Select I2C Connector \0",0,14,0);

    while (1)
    {
	   if (true == timerTrigger)
	   {
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
	   }

	   ParamEdit(&TaskMode);  // run routine if Push Buttom activated
	   if (isSystickExpired(StepTaskTimer))
	   {
		   systickSetTicktime(&StepTaskTimer, StepTaskTime[TaskMode]);
		   //LED_blue_off;
		   switch (TaskMode)
		   {
				case M_InitBat:  //BatterieMessungen starten und prüfen
				{

				   //setting at BALO.c BALOsetup() --> i2cActive
				   //i2cSetClkSpd(i2c,  I2C_CLOCK_400);  // for RFID Reader reduced to 200KHz AMIS 400kHz
				   //i2cSetClkSpd(i2c2,  I2C_CLOCK_1Mz);  // Sensor runs fast
				   BatStatus = getBatVolt(&adChn);
				   if (okBat == BatStatus)
				   {
						setRotaryColor(LED_GREEN);
					//   tft_color  = tft_GREEN;
				   }
				   else
				   {
						setRotaryColor(LED_RED);
					  // tft_color  = tft_YELLOW;
				   }
				   //sprintf(strT, "Battery: %3.1f V", adChn.BatVolt);
				   //tftPrintColor((char *)strT, 0 , 0, tft_color);
				   TaskMode  = M_CheckI2cSlaves;
				}
				break;
				case M_CheckI2cSlaves:  //I2C Scan
				{
				   setRotaryColor(LED_MAGENTA);
				   I2cCheckResult =  CheckAndInitI2cSlaves(&DevPrMask, &StepL,&StepR,&MPU1,&TOF1);

				   if ((I2cCheckResult == 0)&&(DevPrMask & DevMPU1) && (DevPrMask & DevStepL) && (DevPrMask & DevStepR))  //Motor and Sensor present
				   {
						StepRenable = true;
						StepLenable = true;
						TaskMode = M_Bala;  // Motor and Sensor present
						setRotaryColor(LED_GREEN);
						//StepTaskTime = StepTaskTimeSet;								// Tasktime for Stepper Balancing ca 8ms
						RunInit = true;
						break;
				   }
				   if ((I2cCheckResult == 0)&&(DevPrMask & DevTOF1))  //only TOF-Sensor present
				   {
						//StepTaskTime = 70;									// Tasktime for display 70ms
						TaskMode = M_DispTofData;
						setRotaryColor(LED_BLUE);
						break;
				   }
				   if ((I2cCheckResult == 0)&&(DevPrMask & DevMPU1))  //only MPU-Sensor present
				   {
						//StepTaskTime = 70;									// Tasktime for display 70ms
						TaskMode = M_DispMpuData;
						setRotaryColor(LED_GREEN);
						break;
				   }
				}
				break;
		   		case M_DispMpuData:  // read MPU Data
		   		{

		   			DispAlphaNumMPU(&MPU1);

				}
		   		break;
		 		case M_DispTofData:  // read TOF Data
				{
					setRotaryColor(LED_WHITE);
					if (TOF_read_distance_task(&TOF1))
					//if (TOF_start_up_task(&TOF1))
					{
						visualisationTOF(&TOF1);
					}
					setRotaryColor(LED_BLUE);
				}
				break;
		   		case M_Bala:  // Stepper Closed loop Control (old 8)
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
						MPU1.timebase = (float) (StepTaskTimeSet+1) * 10e-4;  			// CycleTime for calc from Gyro to angle  fitting statt 10-3 wird 10-4 gesetzt
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
									curMotR = (float)StepperGetPos(&StepR);					// 0.3 ms
								}
								else
								{
									curMotR = 0;
									StepperResetPosition(&StepR);
									resetStepR = false;
								}
								tarPosR = curMotR + incRot*rampRot;
								posMotR = (int16_t)(setPitch + tarPosR);
								StepperSetPos(&StepR, posMotR); 							//setPosition 0.4 ms
								StepRenable = false;

								gpioResetPin(LED_RED_ADR);									//RED LED ON
							}
							else
							{
								if (!resetStepL)
								{
									curMotL = (float)StepperGetPos(&StepL);					// 0.3ms
								}
								else
								{
									curMotL = 0;
									StepperResetPosition(&StepL);
									resetStepL = false;
								}
								tarPosL = curMotL - incRot*rampRot;
								posMotL = (int16_t)(setPitch + tarPosL);
								StepperSetPos(&StepL, posMotL); 							//setPosition;
								StepRenable = true;
							}
						}
					}
					//ParamEdit();  // run routine if Push Buttom activated
					gpioSetPin(LED_RED_ADR);
				}
				break;
		   		case M_StepFollowPitch:  // Stepper Position follow the tilt angle
				{
					StepperFollowsPitch(StepLenable, StepRenable);
				}
				break;
		   		default:
				{
					TaskMode = M_InitBat;
				}
		   }  //end switch (RunMode)
	    } // end if(isSystickExpired(StepTaskTimer))

/*--------------------------  Routine for Motion Control and Display -------------------*
 *
 *
 *
*/
		if (isSystickExpired(DispTaskTimer))
		{
			systickSetTicktime(&DispTaskTimer, DispTaskTimeSet);   // Reset Disp timer
		if (( DevPrMask & DevTOF1) != 0)
		{
			if (TOF_read_distance_task(&TOF1))
			//if (TOF_start_up_task(&TOF1))
			{
				visualisationTOF(&TOF1);
			}
		}
			if ((activeMove == true) && (TaskMode == M_Bala ))
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
					  pxPos = 4;   pyPos = 40;
					  //tft_color = tft_YELLOW;
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
					  pxPos = 4;   pyPos = 60;
					  //tft_color = tft_WHITE;

					 /**
					  * Debug print
					 tftPrintColor((char *)strT, pxPos, pyPos, tft_color);
					 **/
					}
					break;
					default:
					{
					  MotionVar = 0;
					}
				}		//end switch (MotionVar)
			}  //end if ((activeMove == true) && (RunMode == 8 ))

			//if ((activeMove == false) && (RunMode == 8 ))

			if ((TaskMode == M_DispMpuData)||
				(TaskMode == M_DispTofData)||
				(TaskMode == M_StepFollowPitch)||
				((TaskMode == M_Bala)&&(activeMove != true))
				)
			{
				 dispMPUBat(&MPU1,&adChn);
			}
			if ((MotionVar == 0) && (TaskMode == M_Bala ))
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


/*--------------                      ----------------
 *
 * */


void visualisationTOF(TOFSensor_t* TOFSENS)
{
#define POS_SCREEN_LINE_4 		0, 94, 0
	char buffer[3];
	static uint16_t olddistance_var = TOF_VL53L0X_OUT_OF_RANGE; // Statische Variable zur Speicherung des alten Werts
	uint16_t* olddistance = &olddistance_var; // Pointer auf die statische Variable

	// if value is not out of range
	if (TOFSENS->distanceFromTOF != TOF_VL53L0X_OUT_OF_RANGE)
	{
		// if it was out of range, restore unit visualization
		if (*olddistance == TOF_VL53L0X_OUT_OF_RANGE)
		{
			tftPrint("  cm        ", POS_SCREEN_LINE_4);
		}

		// visualize cm in 2 digits
		int16_t delta = (int16_t) fabs(TOFSENS->distanceFromTOF - *olddistance);
		if (delta > 10)
		{
			sprintf(buffer, "%02d", TOFSENS->distanceFromTOF/10);
			tftPrint(buffer, POS_SCREEN_LINE_4);
		}
	}
	// if value is out of range
	else
	{
		tftPrint("out of range", POS_SCREEN_LINE_4);
	}

	// store current distance to old value
	*olddistance = TOFSENS->distanceFromTOF;
}




void DispAlphaNumMPU(MPU6050_t* pMPU)
{
	char strX[8],strY[8],strZ[8];
//	sprintf(strT, "%+3.2f", mpuGetTemp(&MPU1));
//	tftPrint((char *)strT,40,40,0);

	//i2cLIS3DH_XYZ(i2c,(int16_t *) XYZraw);
	mpuGetAccel(pMPU);
	sprintf(strX, "%+6.3f", pMPU->accel[0]);
	tftPrint((char *)strX,20,50,0);
	sprintf(strY, "%+6.3f", pMPU->accel[1]);
	tftPrint((char *)strY,20,60,0);
	sprintf(strZ, "%+6.3f", pMPU->accel[2]);
	tftPrint((char *)strZ,20,70,0);

	//dispMPUBat(&MPU1,&adChn);
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


	if (fabs(AlphaBeta[1]) > 1)  // tilt angle more than  pi/3 = 60deg  -shut off Stepper control and reduce the IHold current and power consumption -> save the planet ;-)
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



