/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief		   : This is the executable file for our implementation, which
 * 					 includes the functionality for an 3DG sensor and the TOF
 * 					 sensor in combination with an TFT display and the rotary
 * 					 push button
 * @date		   : April 11, 2024
 ******************************************************************************
 */

// CMSIS includes
#include <stm32f4xx.h>
#include <system_stm32f4xx.h>

// standard includes
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>

// MCAL includes
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>

// Balancer includes
#include <Balancer.h>
#include <RotaryPushButton.h>
#include <ST7735.h>
#include <Sensor3DG.h>
#include <SensorTof.h>

// includes of visualization and own header
#include "visualisation.h"
#include "main.h"


#define PIGGYBAG


// Time timer to check Button input, execute
// Menu change, calculate 3DG rotation data
#define timeTimerExec (50)

// Time timer for visualization
#define timeTimerVisu (100)

// Time timer for LED toggle
#define timeTimerLED (250)

// Time timer to get 3DG Data
#define timeTimer3DG (10)

// Variable needed for display
uint32_t ST7735_Timer = 0UL;

// flags, if sensors are initialized
bool inited3DG = false;
bool initedTOF = false;

// current page to be loaded and executed
SCREEN_PAGES_t page = SCREEN_PAGE3;		//SCREEN_MAIN

// variable to initialize TOF, just an internal flag,
// not necessary to use TOF library. There it is located here
ENABLE_TOF_SENSOR_t enableTOFSensor = ENABLE_TOF_FALSE;

// I2C bus used for each sensor.
I2C_TypeDef *i2c_3dg;
I2C_TypeDef *i2c_tof;

// TOF address in use
TOF_ADDR_t TOF_sensor_used = TOF_ADDR_NONE;

uint8_t scanAddr = I2C_MAXADRESS;			// scan I2C address
I2C_t i2cInitPort = I2C_1;					// which I2C is in use for initialization: I2C 1 or 2
uint8_t i2cInitAttempts = I2C_MAXATTEMPTS;	// stores the remaining init-attempts

EXIT_MENU_t exitMenu = EXIT_FALSE;			// variable to exit menu to a certain page

// variable for MCAL SysTick
bool timerTrigger = false;

/*
 * @function:	 main
 *
 * @brief: 		 executable function, timing control and trigger
 * 				 point for all functionality
 */

int main(void)
{

	// variables to store the distance
	uint16_t distance = 0;
	uint16_t olddistance = TOF_VL53L0X_OUT_OF_RANGE;


	// timer variables
	uint32_t TimerExec = 0UL;
	uint32_t TimerVisu = 0UL;
	uint32_t TimerLED = 0UL;
	uint32_t Timer3DG = 0UL;

	// Array with all timer variables and calculation of size
	uint32_t *timerList[] = { &TimerExec, &TimerVisu, &TimerLED, &Timer3DG};
	size_t arraySize = sizeof(timerList)/sizeof(timerList[0]);

	// init project
	initBala();

	visualisationClearBody();


	i2cScanAndInit(i2c);

	enableTOFSensor = true;
	TOF_sensor_used = 0x29;

	if(enableTOFSensor != ENABLE_TOF_FALSE && enableTOFSensor == (ENABLE_TOF_SENSOR_t)i2cInitPort)
	{
		// do TOF sensor initialization
		//visualisationSensorInit(SENSOR_INIT_RUNNING); // Hier wird das TFT weiß

		i2c_tof = i2c;

		bool result = TOF_init(i2c_tof, TOF_sensor_used);

		// check if init was successful
		if(result == true)
		{
			// show if init was successful
			//visualisationSensorInit(SENSOR_INIT_DONE);
			initedTOF = true;

			// give chance to read success-massage
			delayms(500);

			exitMenu = EXIT_FROMSUB1;
		}
	}

//--------------------LIBTESTS---------------------

	//uint16_t time = 20; // in der Realität gemessene 17ms

	//TOF_ReadSingleDistance(&distance);
	//visualisationClearBody();

	//TOF_ReadDistanceTimed(time, &distance);


if (!SetRangingProfile(DEFAULT_MODE_D))
{
	TOF_ReadSingleDistance(&distance);
}

TOF_ReadSingleDistance(&distance);
TOF_ReadSingleDistance(&distance);
TOF_ReadSingleDistance(&distance);
TOF_ReadSingleDistance(&distance);
TOF_ReadSingleDistance(&distance);


//------------------LIBTESTS ENDE-------------------



	// infinity loop to execute software
	while (1)
	{
		if (true == timerTrigger)
		{
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
		}


		// if timer execute is expired
		if (isSystickExpired(TimerExec))
		{
			visualisationTOF(distance, &olddistance);
			TOF_ReadSingleDistance(&distance);

			}


			systickSetTicktime(&TimerExec, timeTimerExec);
		}

	}

/*
 * @function:	 initBala
 *
 * @brief: 		 initialization the Bala-system, initialization hardware
 * 				 from Balancer library and start visualization
 */
void initBala(void)
{
	// initialization bala-library
	BalaHWsetup();

	// initialization rotary push button
	initRotaryPushButton();

	// initialization LED
	initRotaryPushButtonLED();

	// Configure of SysTick-Timers
	systickInit(SYSTICK_1MS);

	//initialization needed for TFT Display
	spiInit();
	tftInitR(INITR_REDTAB);
	// start visualization
	visualisationStart();

	//start page 1: i2c sensor connect
	//page = SCREEN_PAGE1;
}


/*
 * @function:	 initSubMenu
 *
 * @brief: 		 initialization for change into different menu pages
 * 				 with correct transition initialization
 *
 * @parameters:	 SCREEN_PAGES_t page:	 page to be initialized
 */
void initSubMenu(SCREEN_PAGES_t page)
{
	// switch case for menu pages
	switch(page)
	{
	case SCREEN_MAIN:
			break;
	case SCREEN_PAGE1:
		// disable "sensor enable" to be able to initialize all sensors
		enable3DGSensor = false;
		enableTOFSensor = false;

		// reset i2c scan
		scanAddr = I2C_MAXADRESS;
		i2cInitPort = I2C_1;
		i2cInitAttempts = I2C_MAXATTEMPTS;

		// reset sensor initialization
		initedTOF = false;
		inited3DG = false;
			break;
	case SCREEN_PAGE2:
		TOF_startContinuous(5);
			break;
	case SCREEN_PAGE3:
			break;
	case SCREEN_PAGE4:
			break;
	}
}

/*
 * @function:	 i2cScanAndInit
 *
 * @brief: 		 initialization for change into different menu pages
 * 				 with correct transition initialization
 *
 * @parameters:	 I2C_TypeDef   *i2c:	 i2c to be scanned
 */
void i2cScanAndInit(I2C_TypeDef   *i2c)
{
	if (I2C_SCAN(scanAddr, i2c) != 0)
	{
		// check if known sensor is found
		if (I2C_SCAN(scanAddr, i2c) == i2cAddr_Sensor[SENSOR_BMA020])
		{
			currentSensor = SENSOR_BMA020;
			visualisationSensorRecognized(VISUALISATION_BMA020);

			enable3DGSensor = true;
		}
		else if (I2C_SCAN(scanAddr, i2c)== i2cAddr_Sensor[SENSOR_MPU6050])
		{
			currentSensor = SENSOR_MPU6050;
			visualisationSensorRecognized(VISUALISATION_MPU6050);

			enable3DGSensor = true;
		}
		else if (I2C_SCAN(scanAddr, i2c) == i2cAddr_Sensor[SENSOR_LIS3DH])
		{
			currentSensor = SENSOR_LIS3DH;
			visualisationSensorRecognized(VISUALISATION_LIS3DH);

			enable3DGSensor = true;
		}
		else if(I2C_SCAN(scanAddr, i2c) == TOF_ADDR_VL53LOX)
		{
			TOF_sensor_used = TOF_ADDR_VL53LOX;
			visualisationSensorRecognized(VISUALISATION_VL53LOX);

			enableTOFSensor = true;
		}
		else
		{
			// show that an unknown sensor was found
			visualisationSensorRecognized(VISUALISATION_UNKNOWN);
		}

	}

	// check if all i2c addresses are searched
	if (scanAddr <= 0)
	{
		visualisationI2CScanDone(i2cInitAttempts);

		i2cInitAttempts -= 1;
		scanAddr = I2C_MAXADRESS;

		if(i2cInitAttempts < 1)
		{
			exitMenu = EXIT_FROMSUB1;
			i2cInitAttempts = I2C_MAXATTEMPTS;
		}
	}
	// otherwise decrement scan address and search next i2c address
	else
	{
		scanAddr -= 1;
	}

	// initialize 3DG sensor if one is found
	if (enable3DGSensor == true)
	{
		// Sensor initialization
		visualisationSensorInit(SENSOR_INIT_RUNNING);

		i2c_3dg = i2c;

		// do 3DG sensor initialization
		int8_t returnValue = sensor_init(i2c, 0);

		// check if initialization was successful
		if (returnValue == 1)
		{
			// if init failed
		}
		else
		{
			// show if init was successful
			visualisationSensorInit(SENSOR_INIT_DONE);
			inited3DG = true;

			// give chance to read success-massage
			delayms(500);

			exitMenu = EXIT_FROMSUB1;
		}
	}

	// initialize TOF sensor if one is found
	else if(enableTOFSensor != ENABLE_TOF_FALSE && enableTOFSensor == (ENABLE_TOF_SENSOR_t)i2cInitPort)
	{
		// do TOF sensor initialization
		visualisationSensorInit(SENSOR_INIT_RUNNING);

		i2c_tof = i2c;

		bool result = TOF_init(i2c_tof, TOF_sensor_used);

		// check if init was successful
		if(result == true)
		{
			// show if init was successful
			visualisationSensorInit(SENSOR_INIT_DONE);
			initedTOF = true;

			// give chance to read success-massage
			//delayms(500);

			exitMenu = EXIT_FROMSUB1;
		}
	}

	// if bala24 is defined, there are 2 i2c ports available, both must be initialized
	#ifdef BALA2024
	if(i2cInitPort == I2C_1 && exitMenu == EXIT_FROMSUB1)
	{
		exitMenu = EXIT_FALSE;
		visualisationI2C2();

		scanAddr = I2C_MAXADRESS;
		i2cInitPort = I2C_2;

	}
	#endif /* BALA2024 */
}

/*
 * @function:	 I2C_SCAN
 *
 * @brief: 		 function to execute i2c scan
 *
 * @parameters:	 uint8_t scanAddr:	 address to be checked
 * 				 I2C_TypeDef *i2c:	 i2c bus to be scanned
 *
 * @return:		 uint8_t: found address, 0 if not found
 */
uint8_t I2C_SCAN(uint8_t scanAddr, I2C_TypeDef *i2c) {
	uint8_t *result;

	uint8_t foundAddr = 0;

	foundAddr = i2cFindSlaveAddr(i2c, scanAddr);

	result = convDecByteToHex(scanAddr);

	tftPrint((char*) result, POS_SCREEN_LINE_2_R);

	// returns address, if not found it returns 0
	return foundAddr;

}

/*
 * @function:	 convDecByteToHex
 *
 * @brief: 		 convert decimal byte number to hex
 *
 * @parameters:	 uint8_t byte:	 	 byte input
 *
 * @return:		 uint8_t: hex output
 */
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

    // returns hex number
    return hex;
}

/*
 * @function:	 fastMean
 *
 * @brief: 		 calculate fast mean of array size input data
 *
 * @parameters:	 float Data[]:	 	 data to be used
 * 				 int arraySize:		 array size to be calculated
 *
 * @return:		 float: mean value
 */
float fastMean(float Data[], int arraySize)
{
    float sum = 0;
    float mean;

    for(int i = 0; i < arraySize; i++)
    {
        sum = sum + Data[i];
    }
    mean = sum / arraySize;
    return mean;
}
