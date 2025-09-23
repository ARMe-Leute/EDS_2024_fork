/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief		   : This is the executable file for our implementation, which
 * 					 includes the functionality for an 3DG sensor and the TOF
 * 					 sensor in combination with an TFT display and the rotary
 * 					 push button
 * @date		   : April 2024
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
//#include <mcalSPI.h>
#include <mcalI2C.h>
#include <mcalADC.h>
#include <mcalEXTI.h>
#include <mcalTimer.h>
#include <mcalRCC.h>

// Balancer includes
#include <BALO.h>
#include <RotaryPushButton.h>
#include <ST7735.h>
#include <i2cMPU.h>
#include <i2cTOF.h>

// includes of visualization and own header
#include "visualisation.h"
#include "main.h"

// Time timer to check Button input, execute
uint16_t timeTimerExec = 30;

// Time timer for visualization
#define timeTimerVisu (50)

// Time timer for LED toggle
#define timeTimerLED (250)

// Variable needed for display
uint32_t ST7735_Timer = 0UL;

// flags, if sensors are initialized
bool inited3DG = false;
// Mark inited3DG as false for compatibility with dependent functions

bool initedTOF = false;

// current page to be loaded and executed
SCREEN_PAGES_t page = SCREEN_MAIN;

// variable to initialize TOF, just an internal flag,
// not necessary to use TOF library. There it is located here
ENABLE_TOF_SENSOR_t enableTOFSensor = ENABLE_TOF_FALSE;

// I2C bus used for TOF sensor.
I2C_TypeDef *i2c_tof;

// TOF address in use
TOF_ADDR_t TOF_sensor_used = TOF_ADDR_NONE;

I2C_t i2cInitPort = I2C_1;					// which I2C is in use for initialization: I2C 1 or 2
uint8_t i2cInitAttempts = I2C_MAXATTEMPTS;	// stores the remaining init-attempts

EXIT_MENU_t exitMenu = EXIT_FALSE;			// variable to exit menu to a certain page

// variable for MCAL SysTick
bool timerTrigger = false;

//___________
	uint8_t interrupt_statustest = 0;	//Variable for the Register Content
//___________


/*
 * @function:	 main
 *
 * @brief: 		 executable function, timing control and trigger
 */
int main(void)
{

	enableTOFSensor = true;
	TOF_sensor_used = 0x29;

	// variables to store rotation and push button
	int32_t position = 0;
	int32_t oldPosition = 0;
	bool buttonPushed = false;

	// variables to store the distance
	uint16_t TOF_DISTANCE_1 = 10;
	//uint16_t olddistance = TOF_VL53L0X_OUT_OF_RANGE;


	// timer variables
	uint32_t TimerExec = 0UL;
	uint32_t TimerVisu = 0UL;
	uint32_t TimerLED = 0UL;

	// Array with all timer variables and calculation of size
	uint32_t *timerList[] = { &TimerExec, &TimerVisu, &TimerLED};
	size_t arraySize = sizeof(timerList)/sizeof(timerList[0]);

	// init project
	initBala();

	// TOF-Instanz deklarieren
	TOFSensor_t TOF_Sensor_1;

	// Initialisieren des TOF-Sensors
	initializeTOFSensor(&TOF_Sensor_1, I2C1, TOF_ADDR_VL53LOX, TOF_DEFAULT_MODE_D, TOF_DISTANCE_1);

	// Konfigurieren und Aktivieren des Sensors
	configureTOFSensor(&TOF_Sensor_1, TOF_DEFAULT_MODE_D, true);
	//TOF_set_ranging_profile(&TOF_Sensor_1);


	while (1)
	{
		if (true == timerTrigger)
		{
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
		}

		// if timer execute is expired
		if (isSystickExpired(TimerExec))
		{
			position = getRotaryPosition();
			buttonPushed = getRotaryPushButton();

			// switch case for different screen pages
			switch(page)
			{
			case SCREEN_MAIN:
				if(buttonPushed)
					{
					page = (uint16_t)position % 4 + 1;

					// check if it is tried to go to the page of an not initialized sensor
					if(page == 2 && initedTOF == false)
					{
						page = 0;

						visualisationShowError(SCREEN_PAGE1);
					}

					if(page == 3 && initedTOF == false)
					{
						page = 0;

						visualisationShowError(SCREEN_PAGE1);
					}


					// change menu page
					visualisationMenu(page, initedTOF, inited3DG, &TOF_Sensor_1);
					initSubMenu(page, &TOF_Sensor_1);
					}

				if(oldPosition != position)
				{
					oldPosition = position;

					visualisationMenuGridFocus((uint16_t)position % 4, tft_WHITE, tft_YELLOW);
				}
				break;

			// init I2C page
			case SCREEN_PAGE1:
				if(buttonPushed)
				{
					exitMenu = EXIT_FROMSUB1;
				}
				switch(i2cInitPort)
				{
				case I2C_1:
					i2cScanAndInit(&TOF_Sensor_1);
					break;
				case I2C_2:
					//i2cScanAndInit(i2c2);
					break;
				}

				break;

			// TOF page
			case SCREEN_PAGE2:
				if(buttonPushed)
				{
					TOF_stop_continuous(&TOF_Sensor_1);
					exitMenu = EXIT_FROMSUB2;
				}
				TOF_read_continuous_distance(&TOF_Sensor_1);
				break;

			// Ranging Mode Page page
			case SCREEN_PAGE3:
				position = getRotaryPosition();
				uint16_t MODE = (uint16_t)position % 4 + 1;
				visualisationRangingProfileTOF(MODE);

				if(buttonPushed)
				{
					configureTOFSensor(&TOF_Sensor_1, MODE, true);
					//TOF_set_ranging_profile(&TOF_Sensor_1);

					switch(MODE){		//change timerexecution time to recommended +3ms to make shure works
					case 1:
						timeTimerExec = 30;
						break;
					case 2:
						timeTimerExec = 23;
						break;
					case 3:
						timeTimerExec = 203;
						break;
					case 4:
						timeTimerExec = 36;
						break;
					default:
						timeTimerExec = 30;
						break;
					}
					TimerExec = 0UL;
					TimerVisu = 0UL;
					TimerLED = 0UL;

					exitMenu = EXIT_FROMSUB3;
				}
				break;

			case SCREEN_PAGE4:
				if(buttonPushed)
				{
					exitMenu = EXIT_FROMSUB4;
				}
				break;
			}

			// exit sub page to main menu, focus right menu item
			if(exitMenu != EXIT_FALSE)
			{
				page = SCREEN_MAIN;
				setRotaryPosition(exitMenu);
				visualisationMenu(page, initedTOF, inited3DG, &TOF_Sensor_1);

				exitMenu = EXIT_FALSE;
			}

			systickSetTicktime(&TimerExec, timeTimerExec);
		}

		// if timer visualization is expired
		if (isSystickExpired(TimerVisu))
		{
			// switch case for dynamic display update
			switch(page)
			{
			case SCREEN_MAIN:
				break;
			case SCREEN_PAGE1:
				break;
			case SCREEN_PAGE2:

/*
 *
 * Hier ist der Testabschnitt des 6. Semesters, um testen zu können muss der Controller ganz normal gestartet werden
 * und anschließend zur Distanz Messung navigiert werden dieser bleibt in der While Schleife hängen und führt dauerhaft
 * die neue Funktione TOF_read_distance_task aus (s.u.).
 *
*/
/*
				TOF_start_up_task(&TOF_Sensor_1);
				while(1)
					{
						setRotaryColor(LED_GREEN);
						TOF_read_distance_task(&TOF_Sensor_1);
						setRotaryColor(LED_RED);
						delayms(200);
						visualisationTOF(&TOF_Sensor_1);

					}
*/
				visualisationTOF(&TOF_Sensor_1);

				break;
			case SCREEN_PAGE3:
				//visualisationRangingProfileTOF(mode);
				break;
			case SCREEN_PAGE4:
				break;
			}

			systickSetTicktime(&TimerVisu, timeTimerVisu);
		}

		// if timer LED is expired
		if (isSystickExpired(TimerLED))
		{
			// toggle LED to show current speed of system
			gpioTogglePin(LED_BLUE_ADR);

			systickSetTicktime(&TimerLED, timeTimerLED);
		}

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
	//BalaHWsetup();

	 ledActivate();		// at BALO.c
	 i2cActivate();		// at BALO.c
	 LED_red_on;
	 //adcActivate();



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
void initSubMenu(SCREEN_PAGES_t page, TOFSensor_t* TOFSENS)
{
	// switch case for menu pages
	switch(page)
	{
	case SCREEN_MAIN:
			break;
	case SCREEN_PAGE1:
		//enable3DGSensor = false;
		// disable "sensor enable" to be able to initialize all sensors

		enableTOFSensor = false;

		// reset i2c scan
		//scanAddr = I2C_MAXADRESS;
		i2cInitPort = I2C_1;
		i2cInitAttempts = I2C_MAXATTEMPTS;

		// reset sensor initialization
		initedTOF = false;
		break;
	case SCREEN_PAGE2:
		TOF_start_continuous(TOFSENS);
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
void i2cScanAndInit(TOFSensor_t *TOFSENS) //
{
	static uint8_t scanAddr = (uint8_t)I2C_MAXADRESS;			// scan I2C address
	uint8_t scanResult;
	bool InitResult;
	I2C_TypeDef *i2c ;
	i2c = TOFSENS->i2c_tof;
	scanResult =I2C_SCAN(i2c, scanAddr);
	if (scanResult != 0)
	{
		// check if known sensor is found
/*
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

		else		*/
		if(scanResult == TOF_ADDR_VL53LOX)
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

	// initialize TOF sensor if one is found
	if(enableTOFSensor != ENABLE_TOF_FALSE && enableTOFSensor == (ENABLE_TOF_SENSOR_t)i2cInitPort)
	{
		// do TOF sensor initialization
		visualisationSensorInit(SENSOR_INIT_RUNNING);

		i2c_tof = i2c;

		InitResult = TOF_init(TOFSENS);		//Funktion in welcher der MCAL Fehler auftritt

		//ToDo Fehler in der MCAL beheben

		// check if init was successful
		if(InitResult)
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
