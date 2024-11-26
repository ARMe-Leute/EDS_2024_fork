/**
 ******************************************************************************
 * @file           : SensorTOF.h
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief          : This is the library file to communicate with an TOF sensor.
 * 					 Currently adapted to VL53LOX.
 * 					 It is possible to get the distance in single mode and
 * 					 continuous mode. The configuration is implemented with the
 * 					 right register controls.
 * @date		   : December 2025
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
#define TOF_VL53L0X_DEFAULT_ADDRESS (0x29)

// define out of range
#define TOF_VL53L0X_OUT_OF_RANGE (8190)

//defines for SetRangingProfile
#define MSRC_CONFIG_TIMEOUT_MACROP 0x46
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define SYSTEM_SEQUENCE_CONFIG 0x01
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH 0x57
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW 0x56
#define PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x48
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW 0x47
#define MSRC_CONFIG_TIMEOUT_MACROP 0x46
#define GLOBAL_CONFIG_VCSEL_WIDTH 0x32
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define SYSTEM_SEQUENCE_CONFIG 0x01
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70


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


//-------------------- ADDITIONAL SETUP ---------------------
//enum for RangingProfiles
typedef enum
{
	DEFAULT_MODE_D  		= 1,
	HIGH_SPEED_MODE_S		= 2,
	HIGH_ACCURACY_MODE_A  	= 3,
	LONG_RANGE_MODE_R		= 4

} Ranging_Profiles_t;


//enum for vcselPeriodType
typedef enum {
    VcselPeriodPreRange = 0,
    VcselPeriodFinalRange =1
} vcselPeriodType;


//struct for SequenceStepTimeouts
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


//struct for SequenceStepEnables
typedef struct {
    uint8_t tcc;          // TCC step enabled (1 if enabled, 0 if disabled)
    uint8_t dss;          // DSS step enabled (1 if enabled, 0 if disabled)
    uint8_t msrc;         // MSRC step enabled (1 if enabled, 0 if disabled)
    uint8_t pre_range;    // Pre-range step enabled (1 if enabled, 0 if disabled)
    uint8_t final_range;  // Final range step enabled (1 if enabled, 0 if disabled)
} SequenceStepEnables;






//--------------------- SENSOR FUNCTIONS ------------------------

// Strukturdefinition für den TOF-Sensor
typedef struct TOFSensor TOFSensor_t;

struct TOFSensor {
    uint16_t TOF_address_used;        // Die Adresse des Sensors (z.B. 0x29)
    uint16_t i2cAddress;           // Die I2C-Adresse des Sensors
    uint16_t measurementMode;      // Der Modus des Sensors für die Reichweitenmessung
    uint16_t distanceFromTOF;      // Die aktuelle Distanzmessung (distanz vom TOF)
    uint16_t maxRange;             // Der maximal messbare Bereich
    bool enableTOFSensor;          // Aktivierung des Sensors (true/false)

    void (*initialize)(TOFSensor_t*, uint16_t, uint16_t, uint16_t, uint16_t);  // Initialisieren des TOF-Sensors
    void (*configure)(TOFSensor_t*, uint16_t, bool);      // Konfigurieren des TOF-Sensors
    uint16_t (*getMeasurement)(TOFSensor_t*);                                 // Abrufen des aktuellen Messwerts
};

// Funktionsprototypen
extern void initializeTOFSensor(TOFSensor_t* sensor, uint16_t TOF_address_used, uint16_t i2cAddress, uint16_t measurementMode, uint16_t maxRange);
extern void configureTOFSensor(TOFSensor_t* sensor, uint16_t measurementMode, bool enable);




//---------------------INTERNAL FUNCTIONS---------------------

/*
 * @function:	 TOF_configure_interrupt
 *
 * @brief: 		 configure interrupt
 *
 * @returns:	 bool: true if successful
 */
bool TOF_configure_interrupt(void);


/*
 * @function:	 TOF_init_address
 *
 * @brief: 		 init address
 *
 * @returns:	 bool: true if successful
 */
bool TOF_init_address();

/*
 * @function:	 TOF_data_init
 *
 * @brief: 		 data init
 *
 * @returns:	 bool: true if successful
 */
bool TOF_data_init();


/*
 * @function:	 TOF_get_spad_info_from_nvm
 *
 * @brief: 		 get spad info from nvm
 *
 * @parameters:	 uint8_t * count:			count variable
 * 				 bool * type_is_aperture:	flag type is aperture
 *
 * @returns:	 bool: true if successful
 */
bool TOF_get_spad_info_from_nvm(uint8_t * count, bool * type_is_aperture);


/*
 * @function:	 TOF_set_spads_from_nvmvoid
 *
 * @brief: 		 set spads from nvm
 *
 * @returns:	 bool: true if successful
 */
bool TOF_set_spads_from_nvm(void);


/*
 * @function:	 TOF_load_default_tuning_settings
 *
 * @brief: 		 Load tuning settings (same as default tuning settings provided by ST api code
 *
 * @returns:	 bool: true if successful
 */
bool TOF_load_default_tuning_settings(void);


/*
 * @function:	 TOF_set_sequence_steps_enabled
 *
 * @brief: 		 Enable (or disable) specific steps in the sequence
 *
 * @returns:	 bool: true if successful
 */
bool TOF_set_sequence_steps_enabled(uint8_t sequence_step);


/*
 * @function:	 TOF_perform_single_ref_calibration
 *
 * @brief: 		 perform calibration
 *
 * @parameters:	 STOF_calibration_type_t calib_type:	calibration type
 *
 * @returns:	 bool: true if successful
 */
bool TOF_perform_single_ref_calibration(TOF_calibration_type_t calib_type);


/*
 * @function:	 TOF_perform_ref_calibration
 *
 * @brief: 		 perform reference calibration
 *
 * @returns:	 bool: true if successful
 */
bool TOF_perform_ref_calibration();


/*
 * @function:	 TOF_init_device
 *
 * @brief: 		 do different device init
 *
 * @returns:	 bool: true if successful
 */
bool TOF_init_device();


/*
 * @function:	 TOF_getMeasurement
 *
 * @brief: 		 get the measured distance
 *
 * @parameters:	 uint16_t *range:		measured range
 *
 * @returns:	 bool: true if successful
 */
bool TOF_getMeasurement(uint16_t *range);


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
bool TOF_ReadSingleDistance(TOFSensor_t* TOFSensor, uint16_t *range);


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
 * @function:	 setVcselPulsePeriod
 *
 * @brief: 		 sets the pulse period
 *
 * @parameters:	 	vcselPeriodType type : typedef
 * 					uint8_t period_pclks : defines limits based on the requested period
 *
 * @returns:	 bool: true if successful
 */
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);


/*
* @function:	 encodeTimeOut
*
* @brief: 		 sets the final range timeout
*
* @parameters:	 uint16_t final_range_timeout_mclks : defines encode timeout for final range
*
* @returns:	 uint16_t : ms_byte / ls_byte
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
 * @function:	 TOF_SetSignalRateLimit
 *
 * @brief: 		 sets the signal rate limit
 *
 * @parameters:	 float signalRateLimit : signal rate limit (e.g. based on the ranging profile)
 *
 * @returns:	 bool: true if successful
 */
bool setSignalRateLimit(float signalRateLimit);


/*
* @function:	 getSequenceStepEnables
*
* @brief: 		checks if the I2C read was succesful
*
* @parameters:	 SequenceStepEnables
*
* @returns:	 bool: true if successful
*/
bool getSequenceStepEnables(SequenceStepEnables *enables);


/*
* @function:	 getSequenceStepTimeouts
*
* @brief: 		 gets the sequence step timeouts
*
* @parameters:	 SequenceStepEnables
* 				 SequenceStepTimeouts
*
* @returns:	 bool: true if successful
*/
bool getSequenceStepTimeouts(SequenceStepEnables *enables, SequenceStepTimeouts *timeouts);


/*
* @function:	 timeoutMclksToMicroseconds
*
* @brief: 		 get distance in single mode with preset time delay
*
* @parameters:	 uint16_t timeout_period_mclks : timeout period in microseconds
* 				 uint8_t vcsel_period_pclks : Vertical cavity surface emitting laser period
*
* @returns:	 uint32_t : returns timeout_period_mclks * macro_period_ns
*/
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);


/*
* @function:	 getVcselPulsePeriod
*
* @brief: 		 reads the current pulse period type
*
* @parameters:	 enum : vcselPeriodType
*
* @returns:	 uint8_t : vcsel_period
*/
uint8_t getVcselPulsePeriod(vcselPeriodType type);


/*
* @function:	 setMeasurementTimingBudget
*
* @brief: 		 calculates timing budget
*
* @parameters:	 uint32_t budget_us : current timing budget
*
* @returns:	 bool: true if successful
*/
bool setMeasurementTimingBudget(uint32_t budget_us);


/*
* @function:	 timeoutMicrosecondsToMclks
*
* @brief: 		 calculates the macro period in microseconds
*
* @parameters:	 uint32_t timeout_period_us : timeout period in mycroseconds
* 				 uint8_t vcsel_period_pclks : Vertical cavity surface emitting laser period
*
* @returns:	 uint32_t : return value
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
