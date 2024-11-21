/**
 * ST7735 TFT Display
 * ===========================
 *
 * Ansteuerung eines TFT Display ueber SPI.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <stm32f4xx.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <ST7735.h>

//#include "I2C.h" //DEL
//#include "xyzScope.h" //DEL
#include <Sensor_3DG.h>

#define mode_init 0
#define mode_run 1
#define mode_run_filtert 2
#define mode_fehler 3

bool timerTrigger = false;

uint8_t counterFürJakob = 0;




// Declaration  Timer1Sensor = Main Prog
// 				ST7725_Timer delay Counter
uint32_t Timer1Sensor = 0UL;
uint32_t ST7735_Sensor_Timer = 0UL;
uint32_t I2C_Timer = 0UL;
//#define I2CTaskTime 20

/* Private function prototypes -----------------------------------------------*/

uint8_t I2C_SCAN(uint8_t scanAddr);
void i2c_activate_pb89(I2C_TypeDef *i2c);
uint8_t *convDecByteToHex(uint8_t byte);


int main(void) {
	/*  I2C Variables  */
	uint8_t scanAddr = 0x7F;  //7Bit Adresse
	I2C_TypeDef *i2c = I2C1;
	uint32_t i2cTaskTime = 50UL;	//
	/*  End I2C Variables  */

	/* 	Sensor Variables		*/
	char sensorName[20];
	char strX[8], strY[8], strZ[8]; //, strAlpha[8], strBeta[8]
	char g_sum[8];
	//int8_t Temp;
	//int16_t gDataList[3];
	int16_t XYZraw[3];
	float XYZ[3] = {0, 0, 0} , XYZFiltert[3] = {0, 0, 0} ; // , AlphaBeta[2] = {0, 0}

	/* 	Sensor Variables Ende	*/


	static int mode = mode_init;

	//uint16_t timeTMode5; //Unused

	// Dies ist das Array, das die Adressen aller Timer-Variablen enthaelt.
	// Auch die Groesse des Arrays wird berechnet.
	uint32_t *timerList[] =
			{ &Timer1Sensor, &ST7735_Sensor_Timer, &I2C_Timer /*, weitere Timer */};
	size_t arraySize = sizeof(timerList) / sizeof(timerList[0]);

	// Initialisiert den Systick-Timer
	systickInit(SYSTICK_1MS);

	systickSetMillis(&Timer1Sensor, 100);
	systickSetMillis(&I2C_Timer, i2cTaskTime);

	// Display Setup
	tftSetup();
	//lcd7735_initR(INITR_REDTAB);
	tftSetRotation(LANDSCAPE);
	tftSetFont((uint8_t*) &SmallFont[0]);
	tftFillScreen(tft_BLACK);


	i2c_activate_pb89(i2c);

	tftPrint((char*) "I2C Scanner running \0", 0, 0, 0);

	while (1) {

		if (true == timerTrigger)
		{
			systickUpdateTimerList((uint32_t*) timerList, arraySize);
		}

		if (isSystickExpired(I2C_Timer))
		{
			systickSetTicktime(&I2C_Timer, i2cTaskTime);
			//LED_green_off;


			switch (mode)
			{
				case mode_init:
				{
					if(I2C_SCAN(scanAddr) !=0)
					{
						if(I2C_SCAN(scanAddr) == i2cAddr_Sensor[SENSOR_BMA020])
						{
							currentSensor = SENSOR_BMA020;
							tftPrint((char*) "3DG Sensor BMA020 erkannt! \0", 0, 10, 0);
							tftPrint((char*) "Addresse: 0x38 \0", 0, 20, 0);
						    // Leeren des Arrays
						    memset(sensorName, '\0', sizeof(sensorName));

						    // Neubefüllung des Arrays
						    strcpy(sensorName, "BMA020");
							enable3DGSensor = true;
						}
						else if (I2C_SCAN(scanAddr) == i2cAddr_Sensor[SENSOR_MPU6050])
						{
							currentSensor = SENSOR_MPU6050;
							tftPrint((char*) "MPU6050 erkannt! \0", 0, 10, 0);
							tftPrint((char*) "Addresse: 0x68 \0", 0, 20, 0);

						    // Leeren des Arrays
						    memset(sensorName, '\0', sizeof(sensorName));

						    // Neubefüllung des Arrays
						    strcpy(sensorName, "MPU6050");

							enable3DGSensor = true;
						}
						else if (I2C_SCAN(scanAddr) == i2cAddr_Sensor[SENSOR_LIS3DH])
						{
							currentSensor = SENSOR_LIS3DH;
							tftPrint((char*) "3DG Sensor LIS3DH erkannt! \0", 0, 10, 0);
							tftPrint((char*) "Addresse: 0x18 \0", 0, 20, 0);
						    // Leeren des Arrays
						    memset(sensorName, '\0', sizeof(sensorName));

						    // Neubefüllung des Arrays
						    strcpy(sensorName, "LIS3DH");
							enable3DGSensor = true;
						}
						else
						{
							tftPrint((char*) "Der Sensor muss im \0", 0, 40, 0);
							tftPrint((char*) "Code angepasst werden. \0", 0, 50, 0);
							//tftPrint((char*) "Entweder in main.c hinzufügen oder auch noch in Sensor_3DG.h definieren. \0", 0, 50, 0);
						}
					}
					if (scanAddr <= 0)
					{
						tftPrint((char*) "kein 3DG Sensor Erkannt \0", 0, 10, 0);
						scanAddr = 0x6F;
					}
					else
					{
						 scanAddr -=1;
					}
					if(enable3DGSensor == true)
					{
						mode = mode_run;

						// Sensor Initialisierung
						tftPrint((char*) "Sensor Init lauft... \0", 0, 30, 0);

						int8_t returnValue = sensor_init(i2c, 0);

						if (returnValue == 1)
						{
							mode = mode_fehler;
							tftFillScreen(tft_BLACK);
							break;
						}

						i2cTaskTime = 5;
						tftPrint((char*) "Sensor Init fertig. \0", 0, 30, 0);

						for(int i = 0; i < 100; i++)
					    {
					        // Leerer Schleifenkörper
					    }

						tftFillScreen(tft_BLACK);
						tftPrint(sensorName, 55, 0, 0);
						tftPrint((char*) "Xu:\0", 0, 20, 0);
						tftPrint((char*) "Yu:\0", 0, 30, 0);
						tftPrint((char*) "Zu:\0", 0, 40, 0);
						tftPrint((char*) "Gu:\0", 0, 50, 0);
//						tftPrint((char*) "A:\0", 0, 65, 0);
//						tftPrint((char*) "B:\0", 0, 75, 0);

						tftPrint((char*) "Xf:\0", 0, 70, 0);
						tftPrint((char*) "Yf:\0", 0, 80, 0);
						tftPrint((char*) "Zf:\0", 0, 90, 0);
						tftPrint((char*) "Gf:\0", 0, 100, 0);

					}
					break;
				}	//mode_init

				case mode_run:
				{
					tftPrint((char*) "mode_run \0", 0, 110, 0);
					getAccData(i2c,(int16_t *) XYZraw);

					XYZ[0] = (float) XYZraw[0]/(sensorScale[currentSensor]);  //skalierung 1mg/digit at +-2g
					XYZ[1] = (float) XYZraw[1]/((sensorScale[currentSensor]));
					XYZ[2] = (float) XYZraw[2]/((sensorScale[currentSensor]));

					double tempGSumu = sqrt(XYZ[0]*XYZ[0] + XYZ[1]*XYZ[1] + XYZ[2]*XYZ[2]);
					double tempGSumf = sqrt(XYZFiltert[0] * XYZFiltert[0] + XYZFiltert[1] * XYZFiltert[1] + XYZFiltert[2] * XYZFiltert[2]);

//					getAngleFromAcc(XYZ, AlphaBeta);
//
//					sprintf(strAlpha, "%+6.3f", AlphaBeta[0]);
//					tftPrint((char *)strAlpha,20,65,0);
//					sprintf(strBeta, "%+6.3f", AlphaBeta[1]/_pi * 180);
//					tftPrint((char *)strBeta,20,75,0);


					sprintf(strX, "%+6.3f", XYZ[0]);
					tftPrint((char *)strX,20,20,0);
					sprintf(strY, "%+6.3f", XYZ[1]);
					tftPrint((char *)strY,20,30,0);
					sprintf(strZ, "%+6.3f", XYZ[2]);
					tftPrint((char *)strZ,20,40,0);
					sprintf(g_sum, "%+6.3f", tempGSumu);
					tftPrint((char *)g_sum,20,50,0);



					getFiltertAccData(XYZ,XYZFiltert);

					sprintf(strX, "%+6.3f", XYZFiltert[0]);
					tftPrint((char*) strX, 20, 70, 0);
					sprintf(strY, "%+6.3f", XYZFiltert[1]);
					tftPrint((char*) strY, 20, 80, 0);
					sprintf(strZ, "%+6.3f", XYZFiltert[2]);
					tftPrint((char*) strZ, 20, 90, 0);
					sprintf(g_sum, "%+6.3f", tempGSumf);
					tftPrint((char *)g_sum,20,100,0);

//					Temp++;
//					if(Temp > 10)
//					{
//						Temp = 0;
//						mode = mode_run_filtert;
//					}

					break;


				}	//mode_run

				case mode_run_filtert:
				{
					tftPrint((char*) "mode_run_filtert \0", 0, 100, 0);
					getAccData(i2c, (int16_t*) XYZraw);

					XYZ[0] = (float) XYZraw[0] / ((sensorScale[currentSensor])); //skalierung 1mg/digit at +-2g
					XYZ[1] = (float) XYZraw[1] / ((sensorScale[currentSensor]));
					XYZ[2] = (float) XYZraw[2] / ((sensorScale[currentSensor]));



					sprintf(strX, "%+6.3f", XYZFiltert[0]);
					tftPrint((char*) strX, 20, 20, 0);
					sprintf(strY, "%+6.3f", XYZFiltert[1]);
					tftPrint((char*) strY, 20, 30, 0);
					sprintf(strZ, "%+6.3f", XYZFiltert[2]);
					tftPrint((char*) strZ, 20, 40, 0);
//					sprintf(g_sum, "%+6.3f", tempGSum);
//					tftPrint((char*) g_sum, 20, 50, 0);

					for(int i = 0; i < 100; i++)
				    {
				        // Leerer Schleifenkörper
				    }

					getFiltertAccData(XYZ,XYZFiltert);
//					double tempGSum = sqrt(XYZFiltert[0] * XYZFiltert[0] + XYZFiltert[1] * XYZFiltert[1] + XYZFiltert[2] * XYZFiltert[2]);
				break;
				}	//mode_run_filtert

				case mode_fehler:
				{
					if((counterFürJakob%2) == 0)
					{
						tftPrint((char*) "Fehler!" , 55, 00, 0);
					}

					else if((counterFürJakob%2) != 0)
					{
						tftPrint((char*) "           " , 55, 00, 0);
					}

					tftPrint((char*) "Das war so nicht geplant!" , 0, 20, 0);
					counterFürJakob++;
					if((counterFürJakob%10) == 0)
					{
						tftPrint((char*) "Da kannst Du lange warten. " , 0, 50, 0);
						tftPrint((char*) "Hier passiert nichts mehr." , 0, 80, 0);
					}

					break;
				} //mode_fehler

				default:
				{
					mode = mode_init;
				}
			}

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

uint8_t I2C_SCAN(uint8_t scanAddr) {
	I2C_TypeDef *i2c = I2C1;						// Suche auf dem I2C Bus 1
	uint8_t *outString2 = (uint8_t*) "Addr at: \0";	// Standart Text für Ergebnis darstellung
	uint8_t *result;

	uint8_t foundAddr = 0;
	static int xPos = 0;

	foundAddr = i2cFindSlaveAddr(i2c, scanAddr);
	if (xPos == 0) {
		tftPrint((char*) outString2, 0, 14, 0);
		xPos = 66;
	}
	result = convDecByteToHex(scanAddr);
	if (foundAddr != 0) {
		//outString = outString2;
		tftPrint((char*) result, xPos, 14, 0);
		xPos = (int) 20 + xPos;
		if (xPos > 140) {
			xPos = 66;
		}
	} else {
			tftPrint((char *)result,xPos,14,0);
	}
	return foundAddr;

}

void i2c_activate_pb89(I2C_TypeDef *i2c)
{

    GPIO_TypeDef  *portB = GPIOB;

    // GPIOB-Bustakt aktivieren wegen der Verwendung von PB8/PB9 (I2C).
    i2cSelectI2C(i2c);                           // I2C1: Bustakt aktivieren
    //i2cDisableDevice(i2c);
    gpioInitPort(portB);
    gpioSelectPinMode(portB, PIN8, ALTFUNC);
    gpioSelectAltFunc(portB, PIN8, AF4);         // PB8 : I2C1 SCL
    gpioSelectPinMode(portB, PIN9, ALTFUNC);
    gpioSelectAltFunc(portB, PIN9, AF4);         // PB9 : I2C1 SDA

    /**
     * Verwenden Sie auf keinen Fall die MCU-internen Pull-up-Widerstaende!
     * Widerstandswerte: jeweils 4k7 fuer SDA und SCL!
     */
    gpioSetOutputType(portB, PIN8, OPENDRAIN);   // Immer externe Pull-up-
    gpioSetOutputType(portB, PIN9, OPENDRAIN);   // Widerstaende verwenden!!!
    // Initialisierung des I2C-Controllers

    i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_50);

    i2cEnableDevice(i2c);                        // MCAL I2C1 activ
}

uint8_t *convDecByteToHex(uint8_t byte)
{
    static  uint8_t hex[2] = { 0 };

    uint8_t temp;

    temp = byte % 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[1] = temp;

    temp = byte / 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[0] = temp;

    return hex;
}
