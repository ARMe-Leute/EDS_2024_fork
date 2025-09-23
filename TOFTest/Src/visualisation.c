/**
 ******************************************************************************
 * @file           : visualisation.c
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief          : In addition to the executable, this file is for all
 * 					 visualization parts and the communication with the
 * 					 TFT-display.
 * @date		   : December, 2024
 ******************************************************************************
 */

// standard includes
#include <stdint.h>
#include <stdio.h>
#include <math.h>

//#include <mcalI2C.h>
//#include <mcalADC.h>


// includes from balancer library
#include <RotaryPushButton.h>
#include <ST7735.h>
#include <i2cTOF.h>
#include <i2cMPU.h>

// include own header
#include "visualisation.h"

// variable to print rotation for 3DG
char printStrRotX[8], printStrRotY[8];

// current screen orientation
ORIENTATION_SCREEN_t screen_orientation;

// store position 3DG for visualization
int16_t pos_3DG_x = 0;
int16_t pos_3DG_y = 0;

// visualization 3DG invert: -1 to be inverted, 1 to use it as is is
int8_t invert_x = 1;
int8_t invert_y = 1;


/*
 * @function:	 visualisationStart
 *
 * @brief: 		 start visualization and start menu
 */
void visualisationStart(void)
{
	//display setup
	tftSetRotation(LANDSCAPE_FLIP);
	tftSetFont((uint8_t *)&SmallFont[0]);
	tftFillScreen(tft_BLACK);

	// show header on screen
	visualisationHeader();

	// welcome massage
	tftPrint("----------------",0,10,0);
	tftPrint("TimeOfFlight Sensor",0,30,0);
	tftPrint("Philipp & Andreas",0,50,0);
	tftPrint("----------------",0,70,0);


	// wait, that it can be read
	//delayms(20000);

	// start menu scan i2c
	//visualisationMenu(SCREEN_PAGE3, false, false);
}

/*
 * @function:	 visualisationHeader
 *
 * @brief: 		 show screen header
 */
void visualisationHeader(void)
{
	tftPrint("ARM-TMT22GR2",0,0,0);
}

/*
 * @function:	 visualisationMenuGrid
 *
 * @brief: 		 visualize different screen grids
 *
 * @parameters:	 MENU_GRID_t grid:	 which grid should be visualized
 * 				 uint16_t color:	 color for grid
 */
void visualisationMenuGrid(MENU_GRID_t grid, uint16_t color)
{
	//reset rectangle, print it black, if we don't want to draw black
	if(grid != GRID4)
	{
		visualisationMenuGrid(GRID4, tft_BLACK);
	}

	// switch different grids to be visualized
	switch(grid)
	{
		// just a frame
		case GRID1:
			tftDrawRect(0, 14, tftGetWidth() - 1, tftGetHeight() - 1, color);
			break;
		// two frame grid vertical
		case GRID2V:
			tftDrawRect(0, 14, tftGetWidth()/2 -1, tftGetHeight() - 1, color);
			tftDrawRect(tftGetWidth()/2 -1, 14, tftGetWidth() - 1, tftGetHeight() - 1, color);
			break;
		// two frame grid horizontal
		case GRID2H:
			tftDrawRect(0, 14, tftGetWidth() - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14, color);
			tftDrawRect(0, tftGetHeight() - tftGetWidth()/2 + 7 + 14, tftGetWidth() - 1, tftGetHeight() - 1, color);
			break;
		// four frame grid
		case GRID4:
			tftDrawRect(0, 14, tftGetWidth() - tftGetWidth()/2 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14, color);
			tftDrawRect(tftGetWidth()/2 - 1, 14, tftGetWidth() - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14, color);
			tftDrawRect(0, tftGetHeight() - tftGetWidth()/2 + 7 + 14, tftGetWidth() - tftGetWidth()/2 - 1, tftGetHeight() - 1, color);
			tftDrawRect(tftGetWidth()/2 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14, tftGetWidth() - 1, tftGetHeight() - 1, color);
			break;
	}
}

/*
 * @function:	 visualisationMenuGridFocus
 *
 * @brief: 		 focus in 4 frame grid one frame in menu
 *
 * @parameters:	 int32_t position:	 	position to focus
 * 				 uint16_t colorDefault:	default color to reset focus
 * 				 uint16_t colorFocus:	focus color
 */
void visualisationMenuGridFocus(int32_t position, uint16_t colorDefault, uint16_t colorFocus)
{
	//reset rectangle from focus
	visualisationMenuGrid(GRID4, colorDefault);
	tftDrawRect(0 + 1, 14 + 1, tftGetWidth() - tftGetWidth()/2 - 1 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 - 1, tft_BLACK);
	tftDrawRect(tftGetWidth()/2 - 1 + 1, 14 + 1, tftGetWidth() - 1 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 - 1, tft_BLACK);
	tftDrawRect(0 + 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 + 1, tftGetWidth() - tftGetWidth()/2 - 1 - 1, tftGetHeight() - 1 - 1, tft_BLACK);
	tftDrawRect(tftGetWidth()/2 - 1 + 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 + 1, tftGetWidth() - 1 - 1, tftGetHeight() - 1 - 1, tft_BLACK);

	// which frame should be focused
	switch(position)
	{
	case 0:
		tftDrawRect(0 + 1, 14 + 1, tftGetWidth() - tftGetWidth()/2 - 1 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 - 1, colorFocus);
		tftDrawRect(0, 14, tftGetWidth() - tftGetWidth()/2 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14, colorFocus);
		break;
	case 1:
		tftDrawRect(tftGetWidth()/2 - 1 + 1, 14 + 1, tftGetWidth() - 1 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 - 1, colorFocus);
		tftDrawRect(tftGetWidth()/2 - 1, 14, tftGetWidth() - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14, colorFocus);
		break;
	case 2:
		tftDrawRect(tftGetWidth()/2 - 1 + 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 + 1, tftGetWidth() - 1 - 1, tftGetHeight() - 1 - 1, colorFocus);
		tftDrawRect(tftGetWidth()/2 - 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14, tftGetWidth() - 1, tftGetHeight() - 1, colorFocus);
		break;
	case 3:
		tftDrawRect(0 + 1, tftGetHeight() - tftGetWidth()/2 + 7 + 14 + 1, tftGetWidth() - tftGetWidth()/2 - 1 - 1, tftGetHeight() - 1 - 1, colorFocus);
		tftDrawRect(0, tftGetHeight() - tftGetWidth()/2 + 7 + 14, tftGetWidth() - tftGetWidth()/2 - 1, tftGetHeight() - 1, colorFocus);
		break;
	default:
		break;
	}
}

/*
 * @function:	 visualisationMenu
 *
 * @brief: 		 print static content for menu pages: main screen and 4 submenu screens
 *
 * @parameters:	 SCREEN_PAGES_t page:	menu page to be loaded
 * 				 bool initedTOF:		flag if TOF is initialized
 * 				 bool inited3DG:		flag if 3DG is initialized
 */
void visualisationMenu(SCREEN_PAGES_t page, bool initedTOF, bool inited3DG, TOFSensor_t* TOFSENS)
{
	// clear content
	visualisationClearBody();

	// load right screen
	switch(page)
	{
	case SCREEN_MAIN:
		visualisationMenuGridFocus((uint16_t)getRotaryPosition() % 4, tft_WHITE, tft_YELLOW);

		// 1. Submenu
		tftPrint("Sensor-",13,27,0);
		tftPrint("init",23,39,0);

		// 2. Submenu
		tftPrint("TOF", 28 + tftGetWidth()/2, 27, 0);

		// show if sensor is not connected
		if(initedTOF == false)
		{
			tftPrint("(discon.)", 3 + tftGetWidth()/2, 39, 0);
		}

		// 3. Submenu
		tftPrint("Mode", 28 + tftGetWidth()/2, 20 + tftGetHeight()/2, 0);



		// 4. Submenu
		tftPrint("INFO", 24, 20 + tftGetHeight()/2, 0);
		break;
	case SCREEN_PAGE1:
		visualisationMenuGrid(GRID1, tft_WHITE);
		tftPrint("Sensor-init: i2c",POS_SCREEN_LINE_1);
		tftPrint("Address:",POS_SCREEN_LINE_2);
		break;
	case SCREEN_PAGE2:
		visualisationMenuGrid(GRID1, tft_WHITE);
		tftPrint("Read distance with", POS_SCREEN_LINE_1);
		tftPrint("TOF Sensor:", POS_SCREEN_LINE_2);
		tftPrint("     mm      ", POS_SCREEN_LINE_4);

		tftPrint("- PRESS TO GO BACK-", POS_SCREEN_LINE_10);
		break;
	case SCREEN_PAGE3:
		visualisationMenuGrid(GRID1, tft_WHITE);

		tftPrint("Current Mode", POS_SCREEN_LINE_3);
		switch(TOFSENS->Ranging_Profiles_t) {
		    case TOF_DEFAULT_MODE_D:
		        tftPrint("DEFAULT        ", POS_SCREEN_LINE_4);
		        break;

		    case TOF_HIGH_SPEED_MODE_S:
		        tftPrint("HIGH SPEED     ", POS_SCREEN_LINE_4);
		        break;

		    case TOF_HIGH_ACCURACY_MODE_A:
		        tftPrint("HIGH ACCURACY  ", POS_SCREEN_LINE_4);
		        break;

		    case TOF_LONG_RANGE_MODE_R:
		        tftPrint("LONG RANGE     ", POS_SCREEN_LINE_4);
		        break;

		    default:
		        tftPrint("RANGINGERROR   ", POS_SCREEN_LINE_4);
		        break;
		}

		tftPrint("- PRESS TO GO BACK-", POS_SCREEN_LINE_10);
		break;
	case SCREEN_PAGE4:
		visualisationMenuGrid(GRID1, tft_WHITE);
		tftPrint("INFO:", POS_SCREEN_LINE_1);
		tftPrint("NAVIGATION:", POS_SCREEN_LINE_2);
		tftPrint("in the manual!     ", POS_SCREEN_LINE_9);
		tftPrint("Further information", POS_SCREEN_LINE_8);
		tftPrint("       through menu", POS_SCREEN_LINE_6);
		tftPrint(" - rotate: navigate", POS_SCREEN_LINE_5);
		tftPrint("         - go back ", POS_SCREEN_LINE_4);
		tftPrint(" - push: - enter   ", POS_SCREEN_LINE_3);
		tftPrint("- PRESS TO GO BACK-", POS_SCREEN_LINE_10);
		break;
	}
}

/*
 * @function:	 visualisationClearBody
 *
 * @brief: 		 clear content field to black
 */
void visualisationClearBody(void)
{
	tftFillRect(0, 14, tftGetWidth(), tftGetHeight() - 14, tft_BLACK);
}

/*
 * @function:	 visualisationShowError
 *
 * @brief: 		 show error screen for not initialized sensor
 *
 * @parameters:	 SCREEN_PAGES_t page:	which sensor is not initialized
 */
void visualisationShowError(SCREEN_PAGES_t page)
{
	tftFillRect(14, 14, tftGetWidth() - 28, tftGetHeight() - 28, tft_YELLOW);
	tftDrawRect(14, 14, tftGetWidth() - 14, tftGetHeight() - 14, tft_RED);

	tftPrint("ERROR", 60, 25, 0);
	tftPrint("not connected", 27, 55, 0);

	// which sensor
	switch(page)
	{
		case SCREEN_PAGE1:
			tftPrint("TOF-sensor", 40, 40, 0);
			break;
		case SCREEN_PAGE2:
			tftPrint("Ranging-Mode", 40, 40, 0);
			break;
		default:
			break;
	}

	// give time to read
	delayms(2500);
}

/*
 * @function:	 visualisationTOF
 *
 * @brief: 		 visualization of dynamic content for TOF sensor
 * 				 show current distance;
 *
 * @parameters:	 uint16_t distance:		current distance
 * 				 uint16_t *olddistance: old distance
 */
void visualisationTOF(TOFSensor_t* TOFSENS)
{
    static uint16_t olddistance_var = TOF_VL53L0X_OUT_OF_RANGE; // Statische Variable zur Speicherung des alten Werts
    uint16_t* olddistance = &olddistance_var; // Pointer auf die statische Variable

    // if value is not out of range
    if (TOFSENS->distanceFromTOF != TOF_VL53L0X_OUT_OF_RANGE)
    {
        // if it was out of range, restore unit visualization
        if (*olddistance == TOF_VL53L0X_OUT_OF_RANGE)
        {
            tftPrint("     mm      ", POS_SCREEN_LINE_4);
        }

        // visualize mm in 4 digits
        char buffer[100];
        sprintf(buffer, "%04d", TOFSENS->distanceFromTOF);
        tftPrint(buffer, POS_SCREEN_LINE_4);
    }
    // if value is out of range
    else
    {
        tftPrint("out of range", POS_SCREEN_LINE_4);
    }

    // store current distance to old value
    *olddistance = TOFSENS->distanceFromTOF;
}


void visualisationRangingProfileTOF(MODE_PAGES_t* MODE){
	switch(*MODE)
	{
		case DEFAULTMODE:
			tftPrint("DEFAULT MODE      ", POS_SCREEN_LINE_6);
			break;
		case HIGHSPEEDMODE:
			tftPrint("HIGH SPEED MODE   ", POS_SCREEN_LINE_6);
			break;
		case HIGHACCURACYMODE:
			tftPrint("HIGH ACCURACY MODE", POS_SCREEN_LINE_6);
			break;
		case LONGRANGEMODE:
			tftPrint("LONG RANGE MODE   ", POS_SCREEN_LINE_6);
			break;
		default:
			break;
	}


}



/*
 * @function:	 visualisation3DG
 *
 * @brief: 		 visualization of dynamic content for 3DG sensor
 * 				 visualize current distance;
 *
 * @parameters:	 float rotX:		current rotation in x
 * 				 float rotY:		current rotation in y
 * 				 float *oldrotX:	old rotation in x
 * 				 float *oldrotY:	old rotation in y
 */
/*
void visualisation3DG(float rotX, float rotY, float *oldrotX, float *oldrotY)
{
	// only change if value changed
	if(rotX != *oldrotX)
	{
		// print value
		sprintf(printStrRotX, "%+6.1f", rotX/_pi*180);
		tftPrint((char*) printStrRotX, POS_SCREEN_LINE_4_R);

		// clear old visualization
		tftDrawPixel(tftGetWidth()/2 - 4, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_BLACK);
		tftDrawPixel(tftGetWidth()/2 - 3, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_BLACK);
		tftDrawPixel(tftGetWidth()/2 - 1, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_BLACK);
		tftDrawPixel(tftGetWidth()/2 - 0, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_BLACK);

		// restore cross in middle
		tftDrawFastHLine(72, tftGetHeight()/2 + 8, 10, tft_WHITE);

		// visualization inverted if screen flipped
		if(screen_orientation == ORIENTATION_1)
		{
			invert_x = -1;
		}
		else
		{
			invert_x = 1;
		}

		// calc position for visualization
		pos_3DG_x = (int)(((rotX/_pi*180*invert_x)/90)*28 + 28);

		// draw new visualization
		tftDrawPixel(tftGetWidth()/2 - 4, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_RED);
		tftDrawPixel(tftGetWidth()/2 - 3, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_RED);
		tftDrawPixel(tftGetWidth()/2 - 1, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_RED);
		tftDrawPixel(tftGetWidth()/2 - 0, ((tftGetHeight()/2 + 8) - 28) + pos_3DG_x, tft_RED);

		// update old rotation value
		*oldrotX = rotX;
	}

	// only change if value changed
	if(rotY != *oldrotY)
	{
		// print value
		sprintf(printStrRotY, "%+6.1f", rotY/_pi*180);
		tftPrint((char*) printStrRotY, POS_SCREEN_LINE_7_R);

		// clear old visualization
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 6, tft_BLACK);
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 7, tft_BLACK);
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 9, tft_BLACK);
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 10, tft_BLACK);

		// restore cross in middle
		tftDrawFastVLine(tftGetWidth()/2 - 2, (tftGetHeight()/2 + 8) - 5, 10, tft_WHITE);

		// visualization inverted if screen flipped
		if(screen_orientation == ORIENTATION_2)
		{
			invert_y = -1;
		}
		else
		{
			invert_y = 1;
		}

		// calc position for visualization
		pos_3DG_y = (int)(((rotY/_pi*180*invert_y)/90)*151 + 75);

		// draw new visualization
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 6, tft_RED);
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 7, tft_RED);
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 9, tft_RED);
		tftDrawPixel(4 + pos_3DG_y, tftGetHeight()/2 + 10, tft_RED);

		// update old rotation value
		*oldrotY = rotY;
	}
}
*/


/*
 * @function:	 visualisationSensorRecognized
 *
 * @brief: 		 show recognition of sensor
 *
 * @parameters:	 SENSOR_t sensor:	which sensor is recognized
 */
void visualisationSensorRecognized(SENSOR_t sensor)
{
	switch(sensor)
	{
	case VISUALISATION_BMA020:
		tftPrint("BMA020 recognized!", POS_SCREEN_LINE_2);
		tftPrint("Address: 0x38", POS_SCREEN_LINE_3);
		break;
	case VISUALISATION_MPU6050:
		tftPrint("MPU6050 recognized!", POS_SCREEN_LINE_2);
		tftPrint("Address: 0x68", POS_SCREEN_LINE_3);
		break;
	case VISUALISATION_LIS3DH:
		tftPrint("LIS3DH recognized!", POS_SCREEN_LINE_2);
		tftPrint("Address: 0x18", POS_SCREEN_LINE_3);
		break;
	case VISUALISATION_VL53LOX:
		tftPrint("TOF recognized!", POS_SCREEN_LINE_2);
		tftPrint("Address: 0x29", POS_SCREEN_LINE_3);
		break;
	case VISUALISATION_UNKNOWN:
		tftPrint("Unknown sensor!", POS_SCREEN_LINE_3);
		delayms(1000);
		tftPrint("               ", POS_SCREEN_LINE_3);
		break;
	}
}

/*
 * @function:	 visualisationI2CScanDone
 *
 * @brief: 		 show i2c scan through all addresses is done
 *
 * @parameters:	 uint8_t i2cInitAttempts:	attempts left
 */
void visualisationI2CScanDone(uint8_t i2cInitAttempts)
{
	tftPrint("No Sensor found   ", POS_SCREEN_LINE_2);

	// if there are attempts left, trying again, otherwise leave
	if(i2cInitAttempts > 1)
	{
		tftPrint("trying again!     ", POS_SCREEN_LINE_3);
	}
	else
	{
		tftPrint("max attempts!     ", POS_SCREEN_LINE_3);
	}

	// give chance to read
	delayms(3000);


	// restore view
	tftPrint("Address:        ",POS_SCREEN_LINE_2);
	tftPrint("                ",POS_SCREEN_LINE_3);
}

/*
 * @function:	 visualisationSensorInit
 *
 * @brief: 		 visualize sensor init step: running or done
 *
 * @parameters:	 VIS_SENSOR_INIT_t step:	running or done
 */
void visualisationSensorInit(VIS_SENSOR_INIT_t step)
{
	switch(step)
	{
	case SENSOR_INIT_RUNNING:
		tftPrint("Sensor Init running", POS_SCREEN_LINE_3);
		break;
	case SENSOR_INIT_DONE:
		tftPrint("Sensor Init done.  ", POS_SCREEN_LINE_3);
		break;
	}
}

/*
 * @function:	 visualisationFlip
 *
 * @brief: 		 flip screen
 *
 * @parameters:	 SCREEN_PAGES_t page:				menu page to load
 * 				 bool initedTOF:					flag if TOF is inited
 * 				 bool inited3DG:					flag if 3DG is inited
 * 				 ORIENTATION_SCREEN_t orientation:	destination orientation
 */
void visualisationFlip(SCREEN_PAGES_t page, bool initedTOF, bool inited3DG, ORIENTATION_SCREEN_t orientation,  TOFSensor_t* TOFSENS)
{
	// store orientation
	screen_orientation = orientation;

	// flip orientation
	switch(orientation)
	{
	case ORIENTATION_1:
		tftSetRotation(LANDSCAPE_FLIP);
		break;
	case ORIENTATION_2:
		tftSetRotation(LANDSCAPE);
		break;
	}

	// reload screen in right orientation
	tftFillRect(0, 0, tftGetWidth(), 14, tft_BLACK);

	visualisationMenu(page, initedTOF, inited3DG, TOFSENS);

	visualisationHeader();
}

// for BALA2024
#ifdef BALA2024
/*
 * @function:	 visualisationI2C2
 *
 * @brief: 		 visualization of Sensor init for i2c 2
 */
void visualisationI2C2(void)
{
	tftPrint("Sensor-init: i2c2 ",POS_SCREEN_LINE_1);
	tftPrint("Address:          ",POS_SCREEN_LINE_2);
	tftPrint("                  ",POS_SCREEN_LINE_3);

}
#endif /* BALA2024 */
