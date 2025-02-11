/**
 ******************************************************************************
 * @file           : visualisation.h
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief          : In addition to the executable, this file is for all
 * 					 visualization parts and the communication with the
 * 					 TFT-display.
 * @date		   : December, 2024
 ******************************************************************************
 */

#ifndef VISUALISATION_H
#define VISUALISATION_H

// define text position to fill screen
#define POS_SCREEN_LINE_1 		4, 17, 0
#define POS_SCREEN_LINE_2 		4, 30, 0
#define POS_SCREEN_LINE_2_R 	tftGetWidth()/2, 30, 0
#define POS_SCREEN_LINE_3 		4, 40, 0
#define POS_SCREEN_LINE_4 		4, 50, 0
#define POS_SCREEN_LINE_4_R 	tftGetWidth()/2, 50, 0
#define POS_SCREEN_LINE_5 		4, 60, 0
#define POS_SCREEN_LINE_5_R 	tftGetWidth()/2, 60, 0
#define POS_SCREEN_LINE_6 		4, 70, 0
#define POS_SCREEN_LINE_6_R 	tftGetWidth()/2, 70, 0
#define POS_SCREEN_LINE_7 		4, 80, 0
#define POS_SCREEN_LINE_7_R 	tftGetWidth()/2, 80, 0
#define POS_SCREEN_LINE_8 		4, 90, 0
#define POS_SCREEN_LINE_9 		4, 100, 0
#define POS_SCREEN_LINE_10 		4, 110, 0

// define flip threshold
#define FLIP_THRESHHOLD_MAX 0.785398	//in rad: 45°
#define FLIP_THRESHHOLD_MIN -0.785398	//in rad: -45°

// enum for the grid in screen
typedef enum
{
	GRID1 	= 0,  // One rectangle
	GRID2H  = 1,  // two rectangles horizontal
	GRID2V  = 2,  // two rectangles vertical
	GRID4  	= 3,  // four rectangles
} MENU_GRID_t;

// screen enum to be visualized
typedef enum
{
	SCREEN_MAIN 	= 0,  // Main screen
	SCREEN_PAGE1	= 1,  // page 1
	SCREEN_PAGE2	= 2,  // page 2
	SCREEN_PAGE3	= 3,  // page 3
	SCREEN_PAGE4	= 4,  // page 4
}SCREEN_PAGES_t;


// screen enum to be visualized
typedef enum
{
	DEFAULTMODE 		= 1,  // Main screen
	HIGHSPEEDMODE		= 2,  // page 1
	HIGHACCURACYMODE	= 3,  // page 2
	LONGRANGEMODE		= 4,  // page 3

}MODE_PAGES_t;



// enum for sensor init the right sensor
typedef enum
{
	VISUALISATION_BMA020 	= 0,
	VISUALISATION_MPU6050	= 1,
	VISUALISATION_LIS3DH	= 2,
	VISUALISATION_VL53LOX	= 3,
	VISUALISATION_UNKNOWN	= 4
}SENSOR_t;

// enum sensor init running or done
typedef enum
{
	SENSOR_INIT_RUNNING = 0,
	SENSOR_INIT_DONE = 1
}VIS_SENSOR_INIT_t;

// enum for screen orientation
typedef enum
{
	ORIENTATION_1 = 0,
	ORIENTATION_2 = 1,
} ORIENTATION_SCREEN_t;

/*
 * @function:	 visualisationStart
 *
 * @brief: 		 start visualization and start menu
 */
void visualisationStart(void);

/*
 * @function:	 visualisationHeader
 *
 * @brief: 		 show screen header
 */
void visualisationHeader(void);

/*
 * @function:	 visualisationMenuGrid
 *
 * @brief: 		 visualize different screen grids
 *
 * @parameters:	 MENU_GRID_t grid:	 which grid should be visualized
 * 				 uint16_t color:	 color for grid
 */
void visualisationMenuGrid(MENU_GRID_t grid, uint16_t color);

/*
 * @function:	 visualisationMenuGridFocus
 *
 * @brief: 		 focus in 4 frame grid one frame in menu
 *
 * @parameters:	 int32_t position:	 	position to focus
 * 				 uint16_t colorDefault:	default color to reset focus
 * 				 uint16_t colorFocus:	focus color
 */
void visualisationMenuGridFocus(int32_t position, uint16_t colorDefault, uint16_t colorFocus);

/*
 * @function:	 visualisationMenu
 *
 * @brief: 		 print static content for menu pages: main screen and 4 submenu screens
 *
 * @parameters:	 SCREEN_PAGES_t page:	menu page to be loaded
 * 				 bool initedTOF:		flag if TOF is initialized
 * 				 bool inited3DG:		flag if 3DG is initialized
 */
void visualisationMenu(SCREEN_PAGES_t page, bool initedTOF, bool inited3DG, TOFSensor_t* TOFSENS);

/*
 * @function:	 visualisationClearBody
 *
 * @brief: 		 clear content field to black
 */
void visualisationClearBody(void);

/*
 * @function:	 visualisationShowError
 *
 * @brief: 		 show error screen for not initialized sensor
 *
 * @parameters:	 SCREEN_PAGES_t page:	which sensor is not initialized
 */
void visualisationShowError(SCREEN_PAGES_t page);

/*
 * @function:	 visualisationTOF
 *
 * @brief: 		 visualization of dynamic content for TOF sensor
 * 				 show current distance;
 *
 * @parameters:	 uint16_t distance:		current distance
 * 				 uint16_t *olddistance: old distance
 */
void visualisationTOF(TOFSensor_t* TOFSENS);



void visualisationRangingProfileTOF(MODE_PAGES_t* MODE);




/*
 * @function:	 visualisation3DG
 *
 * @brief: 		 visualization of dynamic content for 3DG sensor
 * 				 visualize current rotation;
 *
 * @parameters:	 float rotX:		current rotation in x
 * 				 float rotY:		current rotation in y
 * 				 float *oldrotX:	old rotation in x
 * 				 float *oldrotY:	old rotation in y
 */
void visualisation3DG(float rotX, float rotY, float *oldrotX, float *oldrotY);

/*
 * @function:	 visualisationSensorRecognized
 *
 * @brief: 		 show recognition of sensor
 *
 * @parameters:	 SENSOR_t sensor:	which sensor is recognized
 */
void visualisationSensorRecognized(SENSOR_t sensor);

/*
 * @function:	 visualisationI2CScanDone
 *
 * @brief: 		 show i2c scan through all addresses is done
 *
 * @parameters:	 uint8_t i2cInitAttempts:	attempts left
 */
void visualisationI2CScanDone(uint8_t i2cInitAttempts);

/*
 * @function:	 visualisationSensorInit
 *
 * @brief: 		 visualize sensor init step: running or done
 *
 * @parameters:	 VIS_SENSOR_INIT_t step:	running or done
 */
void visualisationSensorInit(VIS_SENSOR_INIT_t step);

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
void visualisationFlip(SCREEN_PAGES_t page, bool initedTOF, bool inited3DG, ORIENTATION_SCREEN_t orientation,  TOFSensor_t* TOFSENS);

// for BALA2024
#ifdef BALA2024
/*
 * @function:	 visualisationI2C2
 *
 * @brief: 		 visualization of Sensor init for i2c 2
 */
void visualisationI2C2(void);
#endif /* BALA2024 */

#endif /* VISUALISATION_H */
