/**
 ******************************************************************************
 * @file           : SensorTOF.h
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief          : This is the library file to communicate with an TOF sensor.
 * 					 Currently adapted to VL53LOX.
 * 					 It is possible to get the distance in single mode and
 * 					 continuous mode. The configuration is implemented with the
 * 					 right register controls.
 * @date		   : April 16, 2024
 ******************************************************************************
 */

/**
 ******************************************************************************
 * This code and the register addresses are based on the official api which can be found here:
 * https://www.st.com/en/embedded-software/stsw-img005.html#get-software
 ******************************************************************************
 */

#ifndef SENSORTOF_H
#define SENSORTOF_H

// include standard libraries
#include <stdbool.h>

// defines with registers for communication according to api
#define TOF_REG_IDENTIFICATION_MODEL_ID (0xC0)	//get Dive id
#define TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define TOF_REG_MSRC_CONFIG_CONTROL (0x60)
#define TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define TOF_REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define TOF_REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define TOF_REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define TOF_REG_RESULT_INTERRUPT_STATUS (0x13)
#define TOF_REG_SYSRANGE_START (0x00)
#define TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define TOF_REG_RESULT_RANGE_STATUS (0x14)
#define TOF_REG_SLAVE_DEVICE_ADDRESS (0x8A)
#define TOF_SYSTEM_INTERMEASUREMENT_PERIOD (0x04)
#define TOF_OSC_CALIBRATE_VAL (0xF8)

#define TOF_RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define TOF_RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define TOF_RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define TOF_RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define TOF_VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define TOF_VL53L0X_DEFAULT_ADDRESS (0x29)//sollte das nicht 0x27 sein ????

// define out of range
#define TOF_VL53L0X_OUT_OF_RANGE (8190)

// enum with implemented Sensors and addresses (currently only VL53LOX)
typedef enum
{
	TOF_ADDR_NONE		= -1,
	TOF_ADDR_VL53LOX	= 0x29
} TOF_ADDR_t;

// enum for calibration phase
typedef enum
{
    TOF_CALIBRATION_TYPE_VHV 	= 0,
	TOF_CALIBRATION_TYPE_PHASE	= 1
} TOF_calibration_type_t;



typedef enum {
    VcselPeriodPreRange = 0,
    VcselPeriodFinalRange
} vcselPeriodType;


//-------------------- ADDITIONAL SETUP ---------------------
//enum for RangingProfiles
typedef enum
{
	DEFAULT_MODE_D  		= 1,
	HIGH_SPEED_MODE_S		= 2,
	HIGH_ACCURACY_MODE_A  	= 3,
	LONG_RANGE_MODE_R		= 4

} Ranging_Profiles_t;



typedef enum
{
	MODE1  		= 1,
	MODE2		= 2,
	MODE3  	= 3

} vcselPeriodType_t;


typedef struct
{
    uint8_t pre_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint32_t msrc_dss_tcc_us;
    uint16_t pre_range_mclks;
    uint32_t pre_range_us;
    uint8_t final_range_vcsel_period_pclks;
    uint16_t final_range_mclks;
    uint32_t final_range_us;
} SequenceStepTimeouts;

typedef struct {
    uint8_t tcc;          // TCC step enabled (1 if enabled, 0 if disabled)
    uint8_t dss;          // DSS step enabled (1 if enabled, 0 if disabled)
    uint8_t msrc;         // MSRC step enabled (1 if enabled, 0 if disabled)
    uint8_t pre_range;    // Pre-range step enabled (1 if enabled, 0 if disabled)
    uint8_t final_range;  // Final range step enabled (1 if enabled, 0 if disabled)
} SequenceStepEnables;



//---------------------EXTERNAL FUNCTIONS---------------------

/*
 * @function:	 TOF_init
 *
 * @brief: 		 init TOF sensor
 *
 * @parameters:	 I2C_TypeDef *i2c:	i2c used
 * 				 TOF_ADDR_t addr:	TOF address used
 *
 * @returns:	 bool: true if successful
 */
bool TOF_init(I2C_TypeDef *i2c, TOF_ADDR_t addr);

/*
 * @function:	 TOF_startContinuous
 *
 * @brief: 		 Start continuous ranging measurements
 * 				 get measurement with function TOF_ReadContinuousDistance()
 *
 * @parameters:	 uint32_t period_ms: 	period of measurement in ms
 *
 * @returns:	 bool: true if successful
 */
bool TOF_startContinuous(uint32_t period_ms);

/*
 * @function:	 TOF_stopContinuous
 *
 * @brief: 		 stops continuous measurment
 *
 * @returns:	 bool: true if successful
 */
bool TOF_stopContinuous();

/*
 * @function:	 TOF_ReadContinuousDistance
 *
 * @brief: 		 get distance in continuous mode
 *
 * @parameters:	 uint16_t *range:	variable with measurement
 *
 * @returns:	 bool: true if successful
 */
bool TOF_ReadContinuousDistance(uint16_t *range);

/*
 * @function:	 TOF_ReadSingleDistance
 *
 * @brief: 		 get distance in single mode
 *
 * @parameters:	 uint16_t *range:	variable with measurement
 *
 * @returns:	 bool: true if successful
 */
bool TOF_ReadSingleDistance(uint16_t *range);



//--------------- ADDITIONAL EXTERNAL FUNCTIONS---------------

/*
 * @function:	 TOF_SetAddress
 *
 * @brief: 		 get distance in single mode with preset time delay
 *
 * @parameters:	 uint16_t *range :	variable with measurement
 * 				 uint16_t time :	variable with time preset
 *
 * @returns:	 bool: true if successful
 */
bool TOF_SetAddress( uint8_t newAddr);





/*
 * @function:	 TOF_ReadDistanceTimed
 *
 * @brief: 		 get distance in single mode with preset time delay
 *
 * @parameters:	 uint16_t *range :	variable with measurement
 * 				 uint16_t time :	variable with time preset
 *
 * @returns:	 bool: true if successful
 */
bool TOF_ReadDistanceTimed( uint16_t time, uint16_t *range);


/*
 * @function:	 SetRangingProfile
 *
 * @brief: 		 sets the RangingProfile
 *
 * @parameters:	 uint16_t Rangingprofile :	variable with Rangingprofile
 *				 Default mode (D); High speed (S); High accuracy (A); Long range (R)
 *
 * @returns:	 bool: true if successful
 */
bool SetRangingProfile(uint16_t Rangingprofile);



//--------------- NEUE ---------------


/*
 * @function:	 TOF_SetTimingBudget
 *
 * @brief: 		 get distance in single mode with preset time delay
 *
 * @parameters:	 uint16_t *range :	variable with measurement
 * 				 uint16_t time :	variable with time preset
 *
 * @returns:	 bool: true if successful
 */
bool TOF_SetTimingBudget();

/*
 * @function:	 TOF_SetVcselPulsePeriod
 *
 * @brief: 		 get distance in single mode with preset time delay
 *
 * @parameters:	 uint16_t *range :	variable with measurement
 * 				 uint16_t time :	variable with time preset
 *
 * @returns:	 bool: true if successful
 */
bool TOF_SetVcselPulsePeriod();


/*
* @function:	 encodeTimeOut
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
uint16_t encodeTimeOut(uint16_t final_range_timeout_mclks);

/*
* @function:	 decodeTimeOut
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
uint16_t decodeTimeout(uint16_t reg_val);

/*
 * @function:	 TOF_getSignalRateLimit
 *
 * @brief: 		 get distance in single mode with preset time delay
 *
 * @parameters:	 uint16_t *range :	variable with measurement
 * 				 uint16_t time :	variable with time preset
 *
 * @returns:	 bool: true if successful
 */
bool TOF_getSignalRateLimit(float *signalRateLimit);

/*
 * @function:	 TOF_SetSignalRateLimit
 *
 * @brief: 		 get distance in single mode with preset time delay
 *
 * @parameters:	 uint16_t *range :	variable with measurement
 * 				 uint16_t time :	variable with time preset
 *
 * @returns:	 bool: true if successful
 */
bool TOF_SetSignalRateLimit(float *signalRateLimit);

/*
* @function:	 getSequenceStepEnables
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
bool getSequenceStepEnables(SequenceStepEnables *enables);

/*
* @function:	 getSequenceStepTimeouts
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
bool getSequenceStepTimeouts(SequenceStepEnables *enables, SequenceStepTimeouts *timeouts);

/*
* @function:	 timeoutMclksToMicroseconds
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

/*
* @function:	 getVcselPulsePeriod
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
uint8_t getVcselPulsePeriod(vcselPeriodType type);

/*
* @function:	 setMeasurementTimingBudget
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
bool setMeasurementTimingBudget(uint32_t budget_us);

/*
* @function:	 timeoutMicrosecondsToMclks
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

/*
* @function:	 calcMacroPeriod
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t *range :	variable with measurement
* 				 uint16_t time :	variable with time preset
*
* @returns:	 bool: true if successful
*/
uint32_t calcMacroPeriod(uint8_t vcsel_period_pclks);










#endif /* SENSORTOF_H */
