/**
 ******************************************************************************
 * @file           : VL53L0X_TOF_Lib.h
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief A library for interfacing with a TOF (Time-of-Flight) sensor.
 *
 * This implementation is specifically tailored for the VL53L0X sensor.
 * It provides functionality to measure distances in both single-shot and
 * continuous modes. The sensor is configured using appropriate register
 * settings to ensure proper operation.

 * @date		   : December 21, 2024
 ******************************************************************************
 */

/**
 ******************************************************************************
 * This code and the register addresses are based on the official api which can be found here:
 * https://www.st.com/en/embedded-software/stsw-img005.html#get-software
 ******************************************************************************
 */


#ifndef VL53L0X_TOF			//Begin Include-Guard
#define VL53L0X_TOF			//define SENSORTOF_H

#ifdef VL53L0X_TOF
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

#define TOF_RANGE_SEQUENCE_STEP_TCC (0x10) 					/* Target CentreCheck */
#define TOF_RANGE_SEQUENCE_STEP_MSRC (0x04)					/* Minimum Signal Rate Check */
#define TOF_RANGE_SEQUENCE_STEP_DSS (0x28) 					/* Dynamic SPAD selection */
#define TOF_RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define TOF_VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define TOF_VL53L0X_DEFAULT_ADDRESS (0x29)					/*Default TOF Address*/


#define TOF_VL53L0X_OUT_OF_RANGE (8190)						/* define out of range */

//------ Adressen für Ranging Profiles

#define TOF_REG_TimingBudget 0x71  // Adresse für das Timing-Budget
#define TOF_REG_SignalRateLimit 0x44  //"FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT"
#define TOF_REG_PreRange 0x51			//"PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI"




#define MSRC_CONFIG_TIMEOUT_MACROP         0x46
#define PRE_RANGE_CONFIG_VCSEL_PERIOD      0x50
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD    0x70
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71

#define TOF_REG_SYSTEM_SEQUENCE_CONFIG 0x01

#define TOF_REG_SIGNAL_RATE_LIMIT 0x44 // Beispieladresse für das Signalratenlimit
#define TOF_I2C_ADDRESS 0x29 // Beispieladresse des Sensors, bitte anpassen


// Beispielhafte Registeradressen für VCSEL und Timeout-Werte
#define TOF_REG_PreRange 0x51
#define TOF_REG_FinalRange 0x61
#define TOF_REG_VCSEL_PERIOD 0x70
#define TOF_REG_PHASE_CAL_LIM 0x0F
#define TOF_REG_TIMEOUT_MACROP_HI 0x0A
#define TOF_REG_VALID_PHASE_HIGH 0x22
#define TOF_REG_VALID_PHASE_LOW 0x23
#define TOF_REG_MSRC_TIMEOUT_MACROP 0x12
#define TOF_REG_TIMEOUT_MACROP 0x10











#else
/**
Placeholder for other adapted Sensor Register Adresses


 */
#endif



















// enum with implemented Sensors and addresses (currently only VL53LOX)
typedef enum
{
	TOF_ADDR_NONE				= -1,
	TOF_ADDR_VL53LOX			= 0x29
} TOF_ADDR_t;

// enum for calibration phase
typedef enum
{
    TOF_CALIBRATION_TYPE_VHV 	= 0,
	TOF_CALIBRATION_TYPE_PHASE	= 1
} TOF_calibration_type_t;


//enum for RangingProfiles
typedef enum
{
	DEFAULT_MODE_D  			= 1,
	HIGH_SPEED_MODE_S			= 2,
	HIGH_ACCURACY_MODE_A  		= 3,
	LONG_RANGE_MODE_R			= 4

} Ranging_Profiles_t;


//enum for error handling
typedef enum
{
	I2C_ERROR 					= 1,
	COMMUNICATION_ERROR 		= 2

} Errorhandler_t ;


//---------------------EXTERNAL FUNCTIONS---------------------

/*
 * @function:	 TOF_init
 *
 * @brief: 		 This function executes an initialization for the TOF
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









bool TOF_set_timing_budget(uint16_t timing_budget_us);

bool setSignalRateLimit(float limit_Mcps);

bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

bool encodeVcselPeriod(uint8_t period_pclks, uint8_t* encoded_period);

bool getSequenceStepEnables(SequenceStepEnables *enables);

bool getSequenceStepTimeouts(SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts);

bool timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks, uint32_t* mclks);

bool calcMacroPeriod(uint8_t vcsel_period_pclks, uint32_t* macro_period_ns);

bool encodeTimeout(uint32_t timeout_mclks, uint16_t* encoded_timeout);







#endif /* SENSORTOF_H */		//End Include-Guard
