/**
 ******************************************************************************
 * @file           : main.h
 * @author         : Linus Blanke & Christoph Lederbogen
 * @brief          : This is the executable file for our implementation, which
 * 					 includes the functionality for an 3DG sensor and the TOF
 * 					 sensor in combination with an TFT display and the rotary
 * 					 push button
 * @date		   : April 11, 2024
 ******************************************************************************
 */

#include <SensorTOF.h>
#ifndef MAIN_H
#define MAIN_H

// defines for the system
#define I2C_MAXADRESS 	0x7F
#define I2C_MAXATTEMPTS 2

// i2c connections, which can be used
I2C_TypeDef   *i2c  = I2C1;
#ifdef BALA2024
I2C_TypeDef   *i2c2  = I2C2;
#endif // BALA2024

// Enum for exit menu
typedef enum
{
	EXIT_FALSE 		= -1,  	// do not exit
	EXIT_FROMSUB1  	= 0,	// exit from submenu 1
	EXIT_FROMSUB2  	= 1,	// exit from submenu 2
	EXIT_FROMSUB3  	= 2,	// exit from submenu 3
	EXIT_FROMSUB4 	= 3,	// exit from submenu 4
} EXIT_MENU_t;

// Enum for I2C
typedef enum
{
	I2C_1 	= 1,
	I2C_2  	= 2,
} I2C_t;

// Enum for TOF connected with used I2C 1 or 2
typedef enum
{
	ENABLE_TOF_FALSE = 0,
	ENABLE_TOF_I2C_1 = 1,
	ENABLE_TOF_I2C_2 = 2
} ENABLE_TOF_SENSOR_t;

// extern variable if TOF sensor is enabled
extern ENABLE_TOF_SENSOR_t enableTOFSensor;

/*
 * @function:	 initBala
 *
 * @brief: 		 initialization the Bala-system, initialization hardware
 * 				 from Balancer library and start visualization
 */
void initBala(void);

/*
 * @function:	 initSubMenu
 *
 * @brief: 		 initialization for change into different menu pages
 * 				 with correct transition initialization
 *
 * @parameters:	 SCREEN_PAGES_t page:	 page to be initialized
 */
void initSubMenu(SCREEN_PAGES_t page, TOFSensor_t* TOFSENS);

/*
 * @function:	 i2cScanAndInit
 *
 * @brief: 		 initialization for change into different menu pages
 * 				 with correct transition initialization
 *
 * @parameters:	 I2C_TypeDef   *i2c:	 i2c to be scanned
 */
void i2cScanAndInit(TOFSensor_t* TOFSENS, I2C_TypeDef *i2c);
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
uint8_t I2C_SCAN(uint8_t scanAddr, I2C_TypeDef *i2c);

/*
 * @function:	 convDecByteToHex
 *
 * @brief: 		 convert decimal byte number to hex
 *
 * @parameters:	 uint8_t byte:	 	 byte input
 *
 * @return:		 uint8_t: hex output
 */
uint8_t *convDecByteToHex(uint8_t byte);

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
float fastMean(float Data[], int arraySize);

#endif /* MAIN_H */
