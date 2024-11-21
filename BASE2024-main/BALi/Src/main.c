/**
 * I2C Device Scan
 * ST7735 TFT Display
 * ===========================
 *
 * Ansteuerung eines TFT Display ueber SPI.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
//#include <stm32f4xx.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <ST7735.h>
#include <RotaryPushButton.h>
#include <Balancer.h>
#include <Sensor3DG.h>
#include <amis.h>

#include "i2cDevices.h"
#include "xyzScope.h"




bool timerTrigger = false;


// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	Timer1 = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    I2C_Timer = 0UL;
#define StepTaskTime 6


/* Private function prototypes -----------------------------------------------*/
void test_ascii_screen(void);
void test_graphics(void);

uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr);


/* ------ Def and Parameter for stepper motors ------ */

#define i2cAddr_motL 0x61
#define i2cAddr_motR 0x60

struct Stepper StepL, StepR;
const uint8_t iHold = 5;
const int16_t rad2step =  530;		// Ratio step-counts (200 Full-Steps div 1/16 Steps) per rotation at rad:  509.4 =  200* 16 / (2 PI) or 1600/PI

/**
 * void StepperIHold(bool OnSwitch)
 * @param  OnSwitch == true;  IHold active;
 *					== false; IHold reduced to minimum
 * @returns ---
 */
void StepperIHold(bool OnSwitch)
{
	static bool OldStatus = false;
	const uint8_t iOff = 0x00;
	if (OnSwitch != OldStatus)			// commands only active of OnSwitch Status changed
	{
		if (OnSwitch)
		{
			stepper.iHold.set(&StepL, iHold);
			stepper.iHold.set(&StepR, iHold);
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


#define ParamCount 6
struct Parameter
{
	char Title[5];
	float Value;
	float Min;
	float Max;
	float manInc;
} Param[ParamCount];


char ParamTitle[ParamCount][7] = {"PhiZ","HwLP","LP","KP","KD","Rot"};
float ParamValue[ParamCount] =  { 0.0, 3, 	0.14,  	0.4, 	0.1, 	1};
//								{ -0.05, 6, 	0.2,  	0.6, 	1.75, 	2};
float ParamScale[ParamCount] = 	 { 100,   1,	500, 	50, 	50, 	2};
//Param[0].Value = 0;



struct RegPameter
{
	float phi_0;			// Winkel der neutralen Nulllage
	float tp_3dg;			// Faktor des Tiefpass der Sensorwerte
	float tp_tar;			// Tiefpass ??
	float KP;				// Proportional Faktor [steps/phi]
	float KD;				// Differential Faktor
	float Rot;

} RegPa;


/**
 *
 */
int BalaPosRegler(int Pos, float phi)
{
	static float phi_old =0;
	float delta_phi = phi - RegPa.phi_0;
	int _iTargetPos = ((int)(rad2step)*(RegPa.KP* tan(delta_phi) + (RegPa.KD* (phi - phi_old))));
	phi_old = phi;
	return (Pos+_iTargetPos);
}

/**
 *
 */
void SetRegParameter(I2C_TypeDef *i2c)
{
	RegPa.phi_0 = ParamValue[0];
	RegPa.tp_3dg = ParamValue[2];
	RegPa.KP = ParamValue[3];
	RegPa.KD = ParamValue[4];
	if (ParamValue[1] <0 ) { ParamValue[1] =0;}
	if (ParamValue[1] >6 ) { ParamValue[1] =6;}
	i2cMPU6050LpFilt(i2c, (uint8_t) ParamValue[1]);
}



int main(void)
{
/**
 *  I2C Variables  */

	uint8_t        scanAddr = 0x7F;  //7Bit Adresse
	I2C_TypeDef   *i2c  = I2C1;
	I2C_TypeDef   *i2c2  = I2C2;

/**
*	MPU6050 parameter */

	int8_t MPU6050ret=-1;
	uint32_t   i2cTaskTime = 50UL;
	bool MPU6050enable = false;
	float MPUfilt[3] = {0,0,0};
	#define orgkFilt 0.02
	float kFilt = orgkFilt;


	bool StepLenable = false;
	bool StepRenable = false;



	int BalaPos = 0, BalaRot = 0;



	char strX[8],strY[8],strZ[8],strT[32];
	int8_t Temp;
	int16_t XYZraw[3],XYZMPU[3]; //XYZgMPU[3];

/**	Menue for the Filter
 *
 */

	int ButtPos, oldButtPos=0, modif=0;
	int16_t pos_motR=0, pos_motL=0;
	float XYZ[3], AlphaBeta[2];

	static uint8_t RunMode = 1;
	static bool RunInit = true;
	uint16_t timeTMode5;

	//int RunMode = 1;
   	//unsigned int r = 0;

       // Dies ist das Array, das die Adressen aller Timer-Variablen enthaelt.
       // Auch die Groesse des Arrays wird berechnet.
       uint32_t *timerList[] = { &I2C_Timer, &ST7735_Timer /*, weitere Timer */ };
       size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);


    BalaHWsetup();
    LED_red_on;

	//Inits needed for TFT Display
    // Initialisiert den Systick-Timer
	systickInit(SYSTICK_1MS);
	spiInit();
	tftInitR(INITR_REDTAB);

	//display setup
    tftSetRotation(LANDSCAPE_FLIP);
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
		   		   i2cSetClkSpd(i2c,  I2C_CLOCK_400);  // for RFID Reader reduced to 100
		   		   i2cSetClkSpd(i2c2,  I2C_CLOCK_400);
		   		   RunMode  = 1;
		   	   }
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
								//StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, rotDir, acceleration, securePosition)
							    StepperInit(&StepL, i2c, i2cAddr_motL, 	14, 	1,  	2, 		16, 		3, 			1, 		6,			 0);
							    stepper.pwmFrequency.set(&StepL, 1);

						   }
						   break;
					   	   case i2cAddr_motR:
						   {
							   StepRenable = true;
							   tftPrint((char *)"Right->\0",94,110,0);
								//StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, rotDir, acceleration, securePosition)
							   StepperInit(&StepR, i2c, i2cAddr_motR, 	14, 	1,  	2, 		16, 		3, 			0, 		6,			 0);
							   stepper.pwmFrequency.set(&StepR, 1);

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
							   tftPrint((char *)"TOF/LIADR\0",0,80,0);
						   }
						   break;
						   case i2cAddr_LIS3DH:
						   {

							   tftPrint((char *)"LIS3DH\0",95,95,0);
							   LED_blue_on;
						   }
						   break;
						   case i2cAddr_BMA020:
						   {

							   tftPrint((char *)"BMA020\0",90,95,0);
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
				   break;
				}
	// 3DG Sensor function
		   	 	case 4:  // 3DGInit Init
		   	 	{
		   	 		LED_green_on;
		   			if ((MPU6050enable) && (MPU6050ret <0))
					{
						MPU6050ret = i2cMPU6050_init(i2c,0);
					}
		   			else
		   			{ MPU6050ret = 0; }



					if  (MPU6050ret == 0)									// MPU6050 init-procedure finished
					{
						if ((StepRenable)&& (StepLenable))
						{

							RunMode = 8;
							RunInit = true;
						}
						else
						{
							i2cTaskTime = 70;									// Tasktime for display 70ms
							RunMode = 5;
							timeTMode5 = 100;							// count of cycles in Mode5
						}
					}
				}
				break;
		   		case 7:  // read LIS3DH Data
		   		{
		   			LED_blue_on;

		   			Temp = i2cLIS3DH_Temp(i2c);
		   			sprintf(strT, "%+3i", Temp);
		   			tftPrint((char *)strT,40,40,0);

		   			i2cLIS3DH_XYZ(i2c,(int16_t *) XYZraw);

  					XYZ[0] = (float) XYZraw[0]/0x3FFF;  //skalierung 1mg/digit at +-2g
		   			XYZ[1] = (float) XYZraw[1]/0x3FFF;
		   			XYZ[2] = (float) XYZraw[2]/0x3FFF;
		   			sprintf(strX, "%+6.3f", XYZ[0]);
		   			tftPrint((char *)strX,20,50,0);
		   			sprintf(strY, "%+6.3f", XYZ[1]);
		   			tftPrint((char *)strY,20,60,0);
		   			sprintf(strZ, "%+6.3f", XYZ[2]);
		   			tftPrint((char *)strZ,20,70,0);
					if ((timeTMode5--) > 0)
					{
						RunMode = 8;
						tftFillScreen(tft_BLACK);
						tftPrint("T:    LIS3DH (C)23Fl",0,0,0);
						i2cTaskTime = 100;
						LED_blue_off;

					}
				    break;
				}
		   		case 8:  // Stepper Closed loop Control
				{
					if (RunInit)
					{
						tftFillScreen(tft_BLACK);
						tftSetColor(tft_RED, tft_WHITE);
						tftPrint("DHBW BALANCER (c)Fl\0",0,0,0);
						tftSetColor(tft_GREEN, tft_BLACK);
						StepperIHold(true);										//IHold switched on
						StepperResetPosition(&StepL);  		//resetPosition
						StepperResetPosition(&StepR);
						SetRegParameter(i2c);
						i2cTaskTime = StepTaskTime;								// Tasktime for Stepper Balancing 7ms
						RunInit = false;
					}


					i2cMPU6050XYZ(i2c,(int16_t *) XYZMPU);
					getFiltertAccData(XYZMPU, MPUfilt, RegPa.tp_3dg);
					AlphaBeta[1] = atan(MPUfilt[1]/MPUfilt[0]);

					BalaPos = BalaPosRegler(0, AlphaBeta[1]);


					if (fabs(AlphaBeta[1]) > 0.7)  // tilt angle more than  pi/4 = 45deg  -shut off Stepper control and reduce the IHold current and power consumption -> save the planet ;-)
					{
						StepperIHold(false);
						StepperSoftStop(&StepR);
						StepperSoftStop(&StepL);			//softStop

					}
					else
					{
						if (fabs((AlphaBeta[1])-RegPa.phi_0) < 0.03)
						{
							setRotaryColor(LED_GREEN);
						}
						else
						{
							setRotaryColor(LED_YELLOW);
							StepperIHold(true);
						}

						BalaRot = (int)ParamValue[ParamCount-1];
						if (StepRenable)
						{
							pos_motR = StepperGetPos(&StepR) + BalaPos - BalaRot;
							StepperSetPos(&StepR, pos_motR); //setPosition;
							StepRenable = false;
						}
						else
						{
							pos_motL = StepperGetPos(&StepL) + BalaPos + BalaRot;
							StepperSetPos(&StepL, pos_motL); //setPosition;
							StepRenable = true;
						}

					}
					ButtPos = getRotaryPosition();
					if (getRotaryPushButton())
					{
						if (++modif >= ParamCount)		{	modif = 0;	}
						sprintf(strT, "%s :" , ParamTitle[modif]);
						tftPrintColor((char *)strT,10,60,tft_GREEN);
						sprintf(strT, "   %+5.3f   ", ParamValue[modif]);
						tftPrintColor((char *)strT,40,60,tft_GREEN);
						ButtPos = (int)ParamScale[modif]*ParamValue[modif];
						oldButtPos = ButtPos;
						setRotaryPosition(ButtPos);

					}

					if (ButtPos != oldButtPos)
					{
						ParamValue[modif] = ((float)ButtPos/ParamScale[modif]);
						sprintf(strT, "   %+5.3f   ", ParamValue[modif]);
						tftPrintColor((char *)strT,40,60,tft_YELLOW);
						oldButtPos = ButtPos;
						SetRegParameter(i2c);
					}

				}
				break;
		   		case 9:  // Stepper Position follow the tilt angle
				{
					if (RunInit)
					{
						tftFillScreen(tft_BLACK);
						tftSetColor(tft_RED, tft_WHITE);
						tftPrint("DHBW BALA Tilt (c)Fl\0",0,0,0);
						tftSetColor(tft_GREEN, tft_BLACK);
						StepperIHold(true);										//IHold switched on
						StepperResetPosition(&StepL);  		//resetPosition
						StepperResetPosition(&StepR);

						i2cTaskTime = StepTaskTime;								// Tasktime for Stepper Balancing 70ms
						RunInit = false;
					}


					if (MPU6050enable)
					{
						i2cMPU6050XYZ(i2c,(int16_t *) XYZMPU);
						getFiltertAccData(XYZMPU, MPUfilt, kFilt);
						AlphaBeta[1] = atan(MPUfilt[1]/MPUfilt[0]);
					}

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
						kFilt = orgkFilt + ((float)ButtPos)/-500;
						if (kFilt < 0.001) {kFilt = 0.001;}
						if (kFilt > 1) {kFilt =1;}

						sprintf(strT, "kFilt %5.3f ", kFilt);
						tftPrint((char *)strT,10,20,0);
						oldButtPos = ButtPos;
					}
				//RunMode = 2;

				}
				break;
		   		default:
				{
					RunMode = 0;
				}
		   }  //end switch (RunMode)
	   } // end if systickexp
    } //end while
    return 0;
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



/*
void balanceMotor(void)
{
	const int offset_phi = 0; 		// Absolutwert des Winkels für die Schwerpunktlage in degr*10
	int tp_fakttar = 50, tp_fakt3dg = 23;
	int kp = 180;//345; // 140;
	int kd = 999;	//1200 ;			// P Anteil kp/1000 , D-Anteil kd/1000
	int y_off = 100;
	//const float deg2rad = 0.0001745;				// Faktor PI/180°/100
	static int y_old, PotPos;
	static BYTE Ihold, OnMot;
	static long _sto_ltargmean;
	BYTE ret;
	long _ltargetpos;
	int _itargetpos, filt_target, x,y,z, PotPos_raw, phi, rw, Pos_OK, _txyz[4], gxyz[4];
	int rot_l, rot_r;
	static int pos_motL= 0, pos_motR= 0;
	int disp = 0;
	rot_l = -turnSteps;
	rot_r = turnSteps;


	//PotPos =  Conv_mV(ADCfilt[Chan_Uin])/(maxUpot_mV/250) - 125;
	y_off += PotPos;
	//kd += 3*PotPos;
	ret = read_axes(D3Sens_addr, _txyz);
	low_pass(_txyz, gxyz, tp_fakt3dg);
//----------------------------------------------------------------------------
// Regler auf Basis der Achs-Beschleunigungen
	y = gxyz[2];
	z = gxyz[3];
	sprintf(senden, "y%+04i,z%+04i", y,z);
	y += y_off;
	_ltargetpos = (((long)y* (long)kp + (long)(kd*(y - y_old))))/((long)z);
	y_old = y;

	_sto_ltargmean += (long)(_ltargetpos) - (filt_target = _sto_ltargmean/tp_fakttar);
	_itargetpos= -(int)_ltargetpos;  // Richtungsumkehr
//----------------------------------------------------------------------------
	sprintf(senden2, "tr%+05i,mt%+05i", _itargetpos, filt_target);
//	phi = (int)phi_yz(gxyz)-offset_phi+PotPos;
//	rw = (phi/10)*(phi_old/10);		// Werteüberlauf vermeiden
//	if (rw >= 0) // true kein Vorzeichenwechsel,d.h. kein Seitenwechsel


//		if ((phi < 150) && (phi > -150))
//		{
//			_itargetpos = (int) (kp/2* (float) tan(((float)phi)*deg2rad));		// Ruckelvermeidung durch geringere Verstärkung
//		}
//		else
//		{
//			_itargetpos = (int) (kp* (float) tan(((float)phi)*deg2rad));
//		}
//	_itargetpos += (int) (kd*(float) (phi - phi_old)*deg2rad);
	//_itargetpos += (int) (kd*(float) tan(((float)(phi - phi_old))*deg2rad));
//	phi_old = phi;



	Pos_OK = 0;
	if (z > 500)
	{
		Pos_OK = 1;
	} // <Pos nur wenn noch innerhalb von 40° gekippt ist
	else
	{
		softStop(motR_addr);
		softStop(motL_addr);
		sprintf(senden2, " 1___ %03i ---v ", y_off);
		resetPosition(motR_addr);
		pos_motR = 0;
		resetPosition(motL_addr);
		pos_motL = 0;
	}




//	if ((phi < 4500) && (phi > -4500)) {Pos_OK = 1;}		 // <Pos nur wenn noch innerhalb von 45° gekippt ist
//	if ((Pos_OK == 1) && (OnMot != 1))
//	{
//		setIhold(motL_addr,Ihold);
//		setIhold(motR_addr,Ihold);
//		OnMot = 1;
//	}
//	if (Pos_OK != 1)
//	{
//		Ihold = getIhold(motL_addr);
//		setIhold(motL_addr,0);
//		setIhold(motR_addr,0);
//		OnMot = 0;
//	}

	if (Pos_OK == 1)
	{
//			resetPosition(motL_addr);
//			setPosition(motL_addr, _itargetpos+rot_l);
//			resetPosition(motR_addr);
//			setPosition(motR_addr, _itargetpos+rot_r);


		if (_itargetpos <= rot_l )
		{

			// pos_motR = getActualPosition(motR_addr);		//
			setPosition(motR_addr, pos_motR+_itargetpos+rot_r);
			pos_motR += _itargetpos+rot_r;
		}
		else
		{
			//pos_motR = getActualPosition(motR_addr);		//
			setPosition(motR_addr, pos_motR+_itargetpos);
			pos_motR += _itargetpos;
		}
		if (rot_r <= _itargetpos)
		{
			//pos_motL = getActualPosition(motL_addr); //
			setPosition(motL_addr, pos_motL+_itargetpos+rot_l);
			pos_motL += _itargetpos+rot_l;
		}
		else
		{
			//pos_motL = getActualPosition(motL_addr); //
			setPosition(motL_addr, pos_motL+_itargetpos);
			pos_motL += _itargetpos;
		}
	}

}

*/



