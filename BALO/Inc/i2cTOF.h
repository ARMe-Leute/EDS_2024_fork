/**
 ******************************************************************************
 * @file        : i2cTOF.h
 * @authors     : Linus Blanke & Christoph Lederbogen & Andreas Ladner & Philipp Röhlke
 * @authors 	: tflaemig68
 * @brief       : This library file handles communication with a Time-of-Flight (TOF) sensor.
 *                Currently adapted to the VL53LOX sensor.
 *                The library supports both single-shot and continuous measurement modes.
 *                Configuration is managed via the appropriate register settings for the sensor.
 * @date        : April 2025
 * @date	    : July 2025 updates by tflaemig68 marked with //!< TF_July25
 ******************************************************************************


 ******************************************************************************
 * This code and the register addresses are based on the official API, which can be found here:
 * https://www.st.com/en/embedded-software/stsw-img005.html#get-software
 ******************************************************************************


 ******************************************************************************
 * Sensor Configuration:
 * - Supports the VL53LOX sensor via I2C communication.
 * - Defines various operating modes such as high accuracy, long range, and high speed.
 * - Allows for configuration of measurement profiles and VCSEL periods.
 * - Provides functions to initialize, configure, and read measurements from the sensor.
 *
 * Macros:
 * - Calculation of macro periods and VCSEL periods.
 * - Handling of I2C communication for sensor register access.
 * 
 * Structures:
 * - The library includes structures for handling TOF sensor settings and configuration,
 *   such as address, I2C instance, ranging profiles, and measured range.
 * 
 * Functionality:
 * - Initialize TOF sensor with specified settings (address, I2C, mode, range).
 * - Configure the sensor for different measurement modes (e.g., single or continuous).
 * - Retrieve distance measurements from the sensor.
 * 
 * Supported modes:
 * - Default mode, high accuracy mode, high-speed mode, and long-range mode.
 * - Customizable for future sensors as per the interface and register mappings.
 ******************************************************************************

 ******************************************************************************
 * Datasheets and Resources
 *
 * 1. VL53L0X Datasheet:
 *    World's Smallest Time-of-Flight Ranging and Gesture Detection Sensor -
 *    Application Programming Interface (PDF)
 *    Link: https://www.st.com/resource/en/user_manual/um2039-world-smallest-timeofflight-ranging-and-gesture-detection-sensor-application-programming-interface-stmicroelectronics.pdf
 *
 * 2. STMicroelectronics Time-of-Flight Sensor Overview:
 *    Product Page
 *    Link: https://estore.st.com/en/vl53l0cxv0dh-1-cpn.html
 *
 * 3. API Documentation:
 *    VL53L0X Datasheet (PDF)
 *    Link: https://www.st.com/resource/en/datasheet/vl53l0x.pdf
 *
 * 4. Register Definitions:
 *    github: VL53L0X Register Map
 *    Link: https://github.com/GrimbiXcode/VL53L0X-Register-Map?tab=readme-ov-file#register-map-of-vl53l0x
 *
 *
 ******************************************************************************
 */

#ifndef I2CTOF_H
#define I2CTOF_H

// include standard libraries
#include <stdbool.h>

// Register defines for communication with the TOF sensor (according to the API)
#define TOF_REG_SYSRANGE_START                               (0x00)  // Trigger start of range measurement
#define TOF_REG_SYSTEM_SEQUENCE_CONFIG                       (0x01)  // Sequence configuration register
#define TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD               (0x04)  // Inter-measurement period for the system
#define TOF_REG_SYSTEM_RANGE_CONFIG							 (0x09)	 // System range config register
#define TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                 (0x0A)  // System interrupt config GPIO register
#define TOF_REG_SYSTEM_INTERRUPT_CLEAR                       (0x0B)  // System interrupt clear register
#define TOF_REG_SYSTEM_THRESH_HIGH							 (0x0C)	 // high tresh register
#define TOF_REG_INTERNAL_CONFIG_0x0D						 (0x0D)	 // Not officially documented
#define TOF_REG_SYSTEM_THRESH_LOW							 (0x0E)	 // low tresh register
#define TOF_REG_INTERNAL_CONFIG_0x10						 (0x10)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x11						 (0x11)	 // Not officially documented
#define TOF_REG_RESULT_INTERRUPT_STATUS                      (0x13)  // Interrupt status for results
#define TOF_REG_RESULT_RANGE_STATUS                          (0x14)  // Range status result register
#define TOF_REG_INTERNAL_CONFIG_0x20						 (0x20)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x22						 (0x22)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x23						 (0x23)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x24						 (0x24)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x25						 (0x25)	 // Not officially documented
#define TOF_REG_PRE_RANGE_CONFIG_MIN_SNR					 (0x27)	 // Minimum pre range config reg
#define TOF_REG_ALGO_PHASECAL								 (0x30)	 // Algo phasecal register
#define TOF_REG_INTERNAL_CONFIG_0x31						 (0x31)	 // Not officially documented
#define TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH                    (0x32)  // VCSEL width configuration for global settings
#define TOF_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT		 (0x33)	 // Initial phase select register
#define TOF_REG_INTERNAL_CONFIG_0x34						 (0x34)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x35						 (0x35)	 // Not officially documented
#define TOF_REG_SYSTEM_HISTOGRAM_BIN						 (0x40)	 // System histogram register 1
#define TOF_REG_INTERNAL_CONFIG_0x42						 (0x42)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x43						 (0x43)	 // Not officially documented
#define TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  (0x44)  // Final range config for count rate return limit
#define TOF_REG_INTERNAL_CONFIG_0x45						 (0x45)	 // Not officially documented
#define TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP                   (0x46)  // MSRC timeout macrop register
#define TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW           (0x47)  // Final range valid phase low register
#define TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH          (0x48)  // Final range valid phase high register
#define TOF_REG_INTERNAL_CONFIG_0x49						 (0x49)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4A						 (0x4A)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4B						 (0x4B)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4C						 (0x4C)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4D						 (0x4D)	 // Not officially documented
#define TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD          (0x4E)  // Dynamic SPAD requested reference count register
#define TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET             (0x4F)  // Dynamic SPAD reference enable start offset register
#define TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD                (0x50)  // Pre-range VCSEL period register
#define TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI           (0x51)  // High byte for pre-range timeout
#define TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO			 (0x52)	 // Low byte for pre-range timeout
#define TOF_REG_INTERNAL_CONFIG_0x54						 (0x54)	 // Not officially documented
#define TOF_REG_HISTOGRAM_CONFIG_READOUT_CTRL				 (0x55)	 // Readout CTRL register
#define TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW             (0x56)  // Pre-range valid phase low register
#define TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH            (0x57)  // Pre-range valid phase high register
#define TOF_REG_MSRC_CONFIG_CONTROL                          (0x60)  // MSRC (Minimum Signal Rate Check) control register
#define TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI			 (0x61)	 // High sigma tresh register
#define TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO			 (0x62)	 // Low sigma tresh register
#define TOF_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT			 (0x64)	 // Minimum count rate register
#define TOF_REG_INTERNAL_CONFIG_0x65						 (0x65)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x66						 (0x66)	 // Not officially documented
#define TOF_REG_FINAL_RANGE_CONFIG_MIN_SNR					 (0x67)	 // Final-range minimum SNR register
#define TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD              (0x70)  // Final-range VCSEL period register
#define TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI         (0x71)  // High byte for final range timeout
#define TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO         (0x72)  // LOW byte for final range timeout
#define TOF_REG_INTERNAL_CONFIG_0x75						 (0x75)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x76						 (0x76)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x77						 (0x77)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x78						 (0x78)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x7A						 (0x7A)	 // Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x7B						 (0x7B)	 // Not officially documented
#define TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE             (0x80)  // Power management register
#define TOF_REG_SYSTEM_HISTOGRAM_NON						 (0x81)	 // System Histogram register 2
#define TOF_REG_INTERNAL_CONFIG_0x83						 (0x83)	 // Not officially documented
#define TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH                      (0x84)  // GPIO HV MUX active high configuration register
#define TOF_REG_I2C_MODE                                     (0x88)  // NOT officially documented
#define TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV             (0x89)  // VHV (Voltage High Voltage) configuration register
#define TOF_REG_SLAVE_DEVICE_ADDRESS                         (0x8A)  // Slave device address register
#define TOF_REG_INTERNAL_CONFIG_0x8E						 (0x8E)	 // Not officially documented
#define TOF_REG_INTERNAL_TUNING_1                            (0x91)  // Internal tuning register 1
#define TOF_REG_INTERNAL_CONFIG_0x94						 (0x94)	 // Not officially documented
#define TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0             (0xB0)  // Global SPAD enables reference register
#define TOF_REG_GLOBAL_CONFIG_REF_EN_START_SELECT            (0xB6)  // Global config for reference enable start selection
#define TOF_REG_IDENTIFICATION_MODEL_ID                      (0xC0)  // Get Device ID (Model ID)
#define TOF_REG_OSC_CALIBRATE_VAL                            (0xF8)  // Oscillator calibration value register
#define TOF_REG_INTERNAL_TUNING_2                            (0xFF)  // Internal tuning register 2


//Configuration Constants
// Range Out-of-Range Value
#define TOF_VL53L0X_OUT_OF_RANGE                             (8190)  // Value indicating out-of-range result

// Range Sequence Step Registers (Step identifiers for different ranges)
#define TOF_RANGE_SEQUENCE_STEP_TCC                          (0x10)  // Target Center Check step (TCC)
#define TOF_RANGE_SEQUENCE_STEP_MSRC                         (0x04)  // Minimum Signal Rate Check (MSRC)
#define TOF_RANGE_SEQUENCE_STEP_DSS                          (0x28)  // Dynamic SPAD selection (DSS)
#define TOF_RANGE_SEQUENCE_STEP_PRE_RANGE                    (0x40)  // Pre-range step
#define TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE                  (0x80)  // Final range step

// Identification Registers
#define TOF_VL53L0X_EXPECTED_DEVICE_ID                       (0xEE)  // Expected device ID for VL53L0X
#define TOF_VL53L0X_DEFAULT_ADDRESS                          (0x29)  // Default I2C address for VL53L0X sensor

// Helper macro for encoding VCSEL period
#define encodeVcselPeriod(period_pclks)                      (((period_pclks) >> 1) - 1)  // Encodes VCSEL period in PCLKs


// Enum defining implemented TOF Sensors and their addresses (currently only VL53LOX)
typedef enum
{
	TOF_ADDR_NONE       = -1,  // No address
	TOF_ADDR_VL53LOX    = 0x29, // VL53LOX sensor address
	TOF_REGULAR_ADDRESS = 0x54	// Address noted in datasheet
} TOF_ADDR_t;

// Enum for calibration phases
typedef enum
{
	TOF_CALIBRATION_TYPE_VHV   = 0,  // VHV calibration phase
	TOF_CALIBRATION_TYPE_PHASE = 1   // Phase calibration phase
} TOF_calibration_type_t;

// Enum defining Ranging Profiles for the TOF sensor
typedef enum
{
	TOF_DEFAULT_MODE_D         = 1, // Default mode
	TOF_HIGH_SPEED_MODE_S      = 2, // High speed mode
	TOF_HIGH_ACCURACY_MODE_A   = 3, // High accuracy mode
	TOF_LONG_RANGE_MODE_R      = 4, // Long range mode
	TOF_RANGINGPROFILE_ERROR   = 5  // Error state
} Ranging_Profiles_t;

// Enum for VCSEL Period Types (Pre-range and Final-range)
typedef enum
{
	VcselPeriodPreRange  = 0, // Pre-range VCSEL period
	VcselPeriodFinalRange = 1  // Final-range VCSEL period
} vcselPeriodType;

// Enum for Default tuning setting
typedef enum
{
	TOF_DEFAULT_0x00	= 0x00,
	TOF_DEFAULT_0x01	= 0x01,
	TOF_DEFAULT_0x03	= 0x03,
	TOF_DEFAULT_0x04	= 0x04,
	TOF_DEFAULT_0x05	= 0x05,
	TOF_DEFAULT_0x06	= 0x06,
	TOF_DEFAULT_0x08	= 0x08,
	TOF_DEFAULT_0x09	= 0x09,
	TOF_DEFAULT_0x0A	= 0x0A,
	TOF_DEFAULT_0x14	= 0x14,
	TOF_DEFAULT_0x1A	= 0x1A,
	TOF_DEFAULT_0x20	= 0x20,
	TOF_DEFAULT_0x21	= 0x21,
	TOF_DEFAULT_0x25	= 0x25,
	TOF_DEFAULT_0x26	= 0x26,
	TOF_DEFAULT_0x28	= 0x28,
	TOF_DEFAULT_0x2C	= 0x2C,
	TOF_DEFAULT_0x30	= 0x30,
	TOF_DEFAULT_0x32	= 0x32,
	TOF_DEFAULT_0x34	= 0x34,
	TOF_DEFAULT_0x40	= 0x40,
	TOF_DEFAULT_0x44	= 0x44,
	TOF_DEFAULT_0x83	= 0x83,
	TOF_DEFAULT_0x96	= 0x96,
	TOF_DEFAULT_0xA0	= 0xA0,
	TOF_DEFAULT_0xF8	= 0xF8,
	TOF_DEFAULT_0xFE	= 0xFE,
	TOF_DEFAULT_0xFF	= 0xFF

} defaulttuningsetting;


// Struct for SequenceStepTimeouts, containing timing values for different steps in the sensor's sequence
typedef struct
{
	uint8_t pre_range_vcsel_period_pclks;  // Pre-range VCSEL period in PCLKs
	uint16_t msrc_dss_tcc_mclks;           // MSRC/DSS/TCC time in MCLKs
	uint32_t msrc_dss_tcc_us;              // MSRC/DSS/TCC time in microseconds
	uint16_t pre_range_mclks;              // Pre-range time in MCLKs
	uint32_t pre_range_us;                 // Pre-range time in microseconds
	uint8_t final_range_vcsel_period_pclks; // Final-range VCSEL period in PCLKs
	uint16_t final_range_mclks;            // Final-range time in MCLKs
	uint32_t final_range_us;               // Final-range time in microseconds
} SequenceStepTimeouts;

// Struct for SequenceStepEnables, containing flags for enabling/disabling different sequence steps
typedef struct
{
	uint8_t tcc;         // TCC step enabled (1 for enabled, 0 for disabled)
	uint8_t dss;         // DSS step enabled (1 for enabled, 0 for disabled)
	uint8_t msrc;        // MSRC step enabled (1 for enabled, 0 for disabled)
	uint8_t pre_range;   // Pre-range step enabled (1 for enabled, 0 for disabled)
	uint8_t final_range; // Final range step enabled (1 for enabled, 0 for disabled)
} SequenceStepEnables;


// --------------------- SENSOR STRUCTURE AND FUNCTIONS ------------------------


// Structure definition for the TOF sensor
typedef struct TOFSensor TOFSensor_t;

struct TOFSensor {
	I2C_TypeDef *i2c_tof;              // Pointer to the I2C peripheral (e.g., I2C1, I2C2)
	uint16_t TOF_address_used;         // The sensor's I2C address (e.g., 0x29 for VL53LOX)
	uint16_t Ranging_Profiles_t;       // The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
	uint16_t distanceFromTOF;          // The current distance measurement (in mm)
	uint32_t measuredRange;            // RAW Data of measured distance
	bool enableTOFSensor;              // Flag indicating if the sensor is enabled (true/false)
	uint32_t Ranging_Profile_time;	   // Time for the execution for readcontinuos in dependence of RangingProfile
	uint8_t TOF_readyFlag;			   // Flag indicating if the sensor data is ready to read
	uint8_t TOF_measuringage; 		   // Age of the measured distance

	// Function pointers for initializing, configuring, and retrieving measurements from the TOF sensor
	void (*initialize)(TOFSensor_t*, I2C_TypeDef*, uint16_t, uint16_t, uint16_t);  // Function for initializing the sensor
	void (*configure)(TOFSensor_t*, uint16_t, bool);                               // Function for configuring the sensor
	uint16_t (*getMeasurement)(TOFSensor_t*);                                      // Function for retrieving the current measurement
};

// Function prototypes for initializing and configuring the TOF sensor
extern void initTOFSensorData(TOFSensor_t* sensor, I2C_TypeDef *i2c_tof, uint16_t TOF_address_used, uint16_t Ranging_Profiles_t, uint16_t measuredRange);
extern void configTOFSensor(TOFSensor_t* sensor, uint16_t Ranging_Profiles_t, bool enable);


//--------------------- INTERNAL FUNCTIONS ---------------------

/**
 * @function:    TOF_configure_interrupt
 *
 * @brief:       Configures the interrupt settings for the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function initializes and configures the interrupt mechanism for the TOF sensor.
 *               It sets the interrupt to trigger on a new sample being ready and configures the
 *               GPIO interrupt pin as active low, which is compatible with most breakout boards.
 *               The function communicates with the TOF sensor over I2C to apply the required settings.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the configuration is successful, otherwise false.
 */
extern bool TOF_configure_interrupt(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_verify_device
 *
 * @brief:       Verifies the connection to the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function performs initialization steps to prepare communication with the TOF sensor.
 *               It checks the sensor's I2C address and attempts to read the device identification register
 *               to confirm the correct TOF sensor is connected. If the device ID matches the expected value,
 *               the function confirms the presence of the correct sensor.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the correct TOF sensor is connected and communication is successful, otherwise false.
 */
extern bool TOF_init_address(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_data_init
 *
 * @brief:       Initializes data communication and operating modes for the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function sets up the TOF sensor by configuring its power supply mode and I2C communication
 *               parameters. It enables 2.8V mode for the sensor's power pads and configures the sensor to use
 *               standard I2C mode. The function also initializes internal sensor variables required for
 *               proper communication and operation. If any I2C operation fails, the initialization process
 *               terminates and the function returns false.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the sensor is successfully initialized, otherwise false.
 */
extern bool TOF_data_init(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_get_spad_info_from_nvm
 *
 * @brief:       Retrieves SPAD (Single Photon Avalanche Diode) configuration information from the TOF sensor's NVM.
 *
 * @details:     This function reads the TOF sensor's non-volatile memory (NVM) to retrieve information about
 *               the SPAD array, including the count of active SPADs and whether they are of the aperture type.
 *               The function uses a sequence of I2C register reads and writes to configure the sensor, access
 *               the NVM, and extract the required information. Upon successful execution, the SPAD count and
 *               type information are stored in the provided output parameters.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 *				uint8_t * count 							where the SPAD count will be stored.
 * 				bool * type_is_aperture						indicating whether the SPADs are of the aperture type.
 *
 * @returns:    bool: true if the SPAD information is successfully retrieved, otherwise false.
 */
extern bool TOF_get_spad_info_from_nvm(TOFSensor_t* TOFSENS, uint8_t * count, bool * type_is_aperture);


/**
 * @function:    TOF_set_spads_from_nvm
 *
 * @brief:       Configures the SPAD (Single Photon Avalanche Diode) settings for the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function reads SPAD configuration data from the sensor's non-volatile memory (NVM) and
 *               sets up the SPAD array for operation. It retrieves the SPAD count and type (aperture or non-aperture)
 *               using the TOF_get_spad_info_from_nvm function. Based on this data, the function adjusts the SPAD
 *               enable map stored in the sensor's memory to activate the appropriate SPADs. This ensures the sensor
 *               operates optimally according to its hardware configuration.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the SPAD configuration is successful, otherwise false.
 */
extern bool TOF_set_spads_from_nvm(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_load_default_tuning_settings
 *
 * @brief:       Configures the Time-of-Flight (TOF) sensor with default tuning settings.
 *
 * @details:     This function writes a predefined set of configuration values to the TOF sensor registers.
 *               These settings optimize the sensor's operation by adjusting internal parameters such as
 *               timing, gain, and measurement sensitivity. This default tuning is typically required to
 *               initialize the sensor for accurate and reliable distance measurements.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the tuning configuration was successful, otherwise false.
 */
extern bool TOF_load_default_tuning_settings(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_set_sequence_steps_enabled
 *
 * @brief:       Configures the sequence steps enabled in the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function sets the SYSTEM_SEQUENCE_CONFIG register in the TOF sensor to enable
 *               specific sequence steps for measurement. The sequence steps define the sensor's internal
 *               operational flow, such as initialization, pre-range, final range, etc. The sequence_step
 *               parameter is a bitmask that determines which steps are enabled.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 *  			 uint8_t sequence_step						Bitmask specifying the sequence steps to enable. Each bit corresponds to 
 *  														specific step in the sensor's operational sequence.
 *
 * @returns:     bool: true if the sequence configuration was successfully written to the sensor, otherwise false.
 */
extern bool TOF_set_sequence_steps_enabled(TOFSensor_t* TOFSENS, uint8_t sequence_step);


/**
 * @function:    TOF_perform_single_ref_calibration
 *
 * @brief:       Performs a single reference calibration on the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function configures and executes a single calibration sequence, either for 
 *               Very High Voltage (VHV) settings or for phase calibration, based on the provided 
 *               calibration type. The calibration ensures the sensor is accurately configured for
 *               reliable distance measurements.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *                  
 * 				 TOF_calibration_type_t calib_type 			The type of calibration to perform. It can be one of the following:
 *                  - TOF_CALIBRATION_TYPE_VHV  			Calibrates Very High Voltage (VHV) settings.
 *                  - TOF_CALIBRATION_TYPE_PHASE 			Calibrates phase measurements.
 *
 * @returns:     bool: true if the calibration was successfully performed, otherwise false.             
 */              
extern bool TOF_perform_single_ref_calibration(TOFSensor_t* TOFSENS, TOF_calibration_type_t calib_type);


/**
 * @function:    TOF_perform_ref_calibration
 *
 * @brief:       Performs a complete reference calibration on the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function runs two sequential calibration steps for the sensor:
 *               - Very High Voltage (VHV) calibration
 *               - Phase calibration
 *               After the calibration is complete, the function restores the sensor's default
 *               measurement sequence settings to ensure proper operation.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the reference calibration was successfully completed, otherwise false.
 *
 * @note:        - This function depends on the helper function TOF_perform_single_ref_calibration
 *                 to perform individual calibration steps.
 *               - After calibration, the sequence steps for measurement are re-enabled using
 *                 TOF_set_sequence_steps_enabled.
 *               - If any step in the calibration or configuration fails, the function returns false.
 */
extern bool TOF_perform_ref_calibration(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_init_device
 *
 * @brief:       Initializes the Time-of-Flight (TOF) sensor and prepares it for operation.
 *
 * @details:     This function performs all necessary steps to initialize the TOF sensor, including:
 *               - Data initialization
 *               - Loading SPAD (Single Photon Avalanche Diode) settings from Non-Volatile Memory (NVM)
 *               - Loading default tuning settings
 *               - Configuring the interrupt settings
 *               - Setting the measurement sequence steps
 *               - Performing a reference calibration
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the initialization is successful, otherwise false.
 * 
 *
 * @note:        - This function depends on several helper functions, each responsible for a specific initialization
 *                 task. If any step fails, the initialization halts and the function returns false.
 *               - Ensure the I2C communication setup is properly initialized before calling this function.
 */
extern bool TOF_init_device(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_getMeasurement
 *
 * @brief:       Retrieves a distance measurement from the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function reads the measured range data from the sensor and processes it.
 *               It handles interrupt checking, data retrieval, and out-of-range conditions.
 *
 * @param[out]:  uint16_t *range							variable where the measured range will be stored.
 *               - The value represents the measured distance in millimeters (mm).
 *               - If the obstacle is out of range, the value is set to TOF_VL53L0X_OUT_OF_RANGE.
 *
 * @returns:     bool: true if the measurement was successfully retrieved, false otherwise.
 *
 * @note:        - Ensure the sensor is initialized using TOF_init_device before calling this function.
 *               - The function assumes the interrupt status is handled internally by the sensor.
 *
 * @limitations:
 *               - If the range is out of the sensor's measurable distance, the value returned is defined
 *                 as TOF_VL53L0X_OUT_OF_RANGE.
 */
extern bool TOF_getMeasurement(TOFSensor_t* TOFSENS, uint16_t *range);


//--------------------- EXTERNAL FUNCTIONS ---------------------


/**
 * @function:    TOF_init
 *
 * @brief:       Initializes the Time-of-Flight (TOF) sensor for operation.
 *
 * @details:     This function performs the necessary initialization steps to prepare the TOF sensor for use,
 *               including setting the I2C address, verifying connectivity, and initializing the device with
 *               default configurations and calibration.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the initialization was successful, otherwise false.
 */
extern bool TOF_init(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_start_continuous
 *
 * @brief:       Initiates continuous measurement mode for the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function configures the TOF sensor to operate in continuous measurement mode, 
 *               either timed or back-to-back mode, based on the provided period. It adjusts settings 
 *               for the selected mode and ensures accurate timing by using the oscillator calibration value.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *                  
 * 				uint32_t period_ms							Measurement interval in milliseconds.
 *
 * @returns:     bool: true if continuous mode starts successfully, otherwise false.
 */
extern bool TOF_start_continuous(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_stop_continuous
 *
 * @brief:       Stops the continuous measurement mode of the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function halts the ongoing continuous measurement mode and restores the sensor
 *               to a single-shot measurement mode. It resets relevant configuration registers and
 *               ensures that the sensor is ready for other operations.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if continuous mode stops successfully, false otherwise.
 */
extern bool TOF_stop_continuous(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_read_continuous_distance
 *
 * @brief:       Reads the distance measurement from the Time-of-Flight (TOF) sensor while in continuous mode.
 *
 * @details:     This function retrieves the latest distance measurement from the sensor and updates the 
 *               distanceFromTOF field in the provided TOFSensor_t structure. It assumes that the sensor 
 *               is operating in continuous measurement mode.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the distance is successfully read and updated, otherwise false.
 */
extern bool TOF_read_continuous_distance(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_read_single_distance
 *
 * @brief:       Initiates a single-shot distance measurement on the TOF sensor and retrieves the result.
 *
 * @details:     This function configures the TOF sensor for a single measurement, waits for the measurement 
 *               to complete, and retrieves the measured distance. The distance is stored in the 
 *               distanceFromTOF field of the TOFSensor_t structure.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the measurement was successfully initiated, completed, and retrieved, false otherwise.

 */
extern bool TOF_read_single_distance(TOFSensor_t* TOFSensor);



/**
 * @function:    TOF_start_up_task
 *
 * @brief:       Initializes and starts the "Briefkasten" measuring method of the TOF sensor.
 *
 * @details:     This function configures the TOF sensor by sending a series of commands to set up the measuring management,
 *               internal tuning, and ranging start registers.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: Returns true if the TOF sensor was successfully initialized and started, otherwise false.
 */
extern bool TOF_start_up_task(TOFSensor_t* TOFSENS);



/**
 * @function:    TOF_read_distance_task
 *
 * @brief:       Reads the current distance of the TOF sensor by checking permanently for new measuring results ("Briefkasten" measuring method).
 *
 * @details:     The function initiates communication with the TOF sensor and checks for a measurement result.
 *               If the data ready flag is set, the new measurement result is processed, and a new measurement is started.
 *               If no new data is available, the measurement age counter is incremented.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     uint16_t: returns current stored distance value.
 */
extern bool TOF_read_distance_task(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_set_address
 *
 * @brief:       Sets a new I2C address for the TOF sensor.
 *
 * @details:     This function updates the I2C address of the TOF sensor by writing the new address 
 *               to the appropriate register on the sensor. The new address is also updated in the 
 *               TOFSensor_t structure for future communication.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the address was successfully updated, otherwise false.
 */
extern bool TOF_set_address(TOFSensor_t* TOFSENS, uint8_t newAddr);


/**
 * @function:    TOF_read_distance_timed
 *
 * @brief:       Reads a single distance measurement from the TOF sensor with a specified delay.
 *
 * @details:     This function triggers a single measurement on the TOF sensor after waiting for the 
 *               specified delay. It handles the necessary register configuration for the measurement, 
 *               reads the distance result, and stores it in the provided `range` pointer.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *                  
 *               uint16_t time    							The delay in milliseconds before initiating the measurement.
 * @param[out]:  uint16_t *range							Pointer to store the measured distance in millimeters.
 *
 * @returns:     bool: true if the distance was successfully read, false otherwise.
 *
 * @warning: 	NOT IMPLEMENTED YET
 *
 *
 */
extern bool TOF_read_distance_timed(TOFSensor_t* TOFSENS, uint16_t time, uint16_t *range);


/**
 * @function:    TOF_set_ranging_profile
 *
 * @brief:       Configures the Time-of-Flight (TOF) sensor to operate with a specified ranging profile.
 *
 * @details:     This function sets different operational profiles for the TOF sensor, such as high speed,
 *               high accuracy, long range, or default mode. The profiles adjust parameters like measurement
 *               timing budgets and signal rate limits to optimize the sensor for specific applications.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @returns:     bool: true if the ranging profile was successfully set, otherwise false.
 */
extern bool TOF_set_ranging_profile(TOFSensor_t* TOFSENS);


/**
 * @function:    TOF_set_vcsel_pulse_period
 *
 * @brief:       Configures the VCSEL (Vertical-Cavity Surface-Emitting Laser) pulse period for the TOF sensor.
 *
 * @details:     This function adjusts the VCSEL pulse period used for either the pre-range or final-range measurement phase.
 *               It updates related phase settings, timeouts, and applies the necessary calibration adjustments.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * 				vcselPeriodType type						Specifies whether to configure the pre-range (VcselPeriodPreRange) or final-range (VcselPeriodFinalRange) phase.
 * 				 uint8_t period_pclks						The new VCSEL pulse period in PCLKs. Acceptable values vary for pre-range and final-range types:
 *                   - Pre-Range: 12, 14, 16, 18.
 *                   - Final-Range: 8, 10, 12, 14.
 *
 * @returns:     bool: true if the configuration was successful, otherwise false.
 *
 * @notes:
 *               - Changing VCSEL pulse periods may impact the sensor's measurement range and accuracy.
 *               - Always reapply the measurement timing budget after changing VCSEL configurations to maintain consistency.
 */
extern bool TOF_set_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type, uint8_t period_pclks);


/**
 * @function:    TOF_set_signal_rate_limit
 *
 * @brief:       Sets the signal rate limit for the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function configures the maximum signal rate that the sensor will measure, expressed in megacounts per second (MCPS).
 *               It ensures the signal rate limit is within the valid range (0 to 511.99 MCPS) and applies the appropriate conversion
 *               to send the value to the sensor.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * 				float signalRateLimit						The desired signal rate limit in MCPS (megacounts per second). This should be a floating-point value between 0 and 511.99.
 *
 * @returns:     bool: true if the signal rate limit was successfully set, false otherwise.
 *
 * @limitations:
 *               - The signalRateLimit must be within the range of 0 to 511.99. If it is outside this range, the function returns false.
 *               - The I2C transaction must succeed for the function to return true.
 *
 * @notes:
 *               - The signal rate limit impacts the accuracy and the maximum measurable distance of the sensor.
 *               - If the rate limit is set too low, it may affect the sensor's ability to detect distant objects.
 *               - If the rate limit is too high, the sensor may be overwhelmed with noise, leading to inaccurate measurements.
 */
extern bool TOF_set_signal_rate_limit(TOFSensor_t* TOFSENS, float signalRateLimit);


/**
 * @function:    TOF_get_sequence_step_enables
 *
 * @brief:       Retrieves the sequence step enables from the TOF sensor.
 *
 * @details:     This function reads the current sequence configuration of the sensor from the SYSTEM_SEQUENCE_CONFIG register and
 *               extracts the bit values to determine which sequence steps (TCC, DSS, MSRC, PRE_RANGE, and FINAL_RANGE) are enabled.
 *               These steps define the order and conditions for various measurement phases in the sensor's operation.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * @param[out]:  SequenceStepEnables *enables 				Pointer to a SequenceStepEnables structure where the extracted enable values will be stored.
 *
 * @returns:     bool: true if the sequence step enables were successfully retrieved, false otherwise.
 *
 * @notes:
 *               - The SYSTEM_SEQUENCE_CONFIG register stores the configuration of the sequence steps used by the sensor.
 *               - Each bit in the byte represents an individual sequence step: TCC, DSS, MSRC, PRE_RANGE, FINAL_RANGE.
 *               - The SequenceStepEnables structure is populated with the status of each sequence step based on these bits.
 */
extern bool TOF_get_sequence_step_enables(TOFSensor_t* TOFSENS, SequenceStepEnables *enables);


/**
 * @function:    TOF_get_sequence_step_timeouts
 *
 * @brief:       Retrieves the timeout values for the sequence steps in the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function calculates the timeout values for different steps in the ranging sequence (MSRC, pre-range, and final-range) based on the current sensor configuration. The timeouts are adjusted according to the VCSEL (Vertical Cavity Surface Emitting Laser) pulse periods for the pre-range and final-range modes. The timeouts are expressed in microseconds.
 *
 * @param[in]:   TOFSENS - Pointer to the `TOFSensor_t` structure containing the sensor's configuration and state.
 * 				 enables - Pointer to a `SequenceStepEnables` structure containing the enables for each step in the sequence.
 * @param[out]:  timeouts - Pointer to a `SequenceStepTimeouts` structure that will hold the calculated timeout values for each sequence step.
 *
 * @returns:     bool: true if the timeouts were successfully retrieved and calculated, false if there was an error in the process.
 *
 * @notes:
 *               - The calculated timeouts are in microseconds and are stored in the `timeouts` structure.
 *               - The VCSEL period for both pre-range and final-range steps is used to calculate the corresponding timeouts.
 */
extern bool TOF_get_sequence_step_timeouts(TOFSensor_t* TOFSENS, SequenceStepEnables *enables, SequenceStepTimeouts *timeouts);


/**
 * @function:    TOF_get_vcsel_pulse_period
 *
 * @brief:       Retrieves the VCSEL (Vertical Cavity Surface Emitting Laser) pulse period from the TOF sensor.
 *
 * @details:     This function reads the VCSEL pulse period for either the pre-range or final-range mode of the sensor. The VCSEL period is stored in specific registers, and the function decodes the value to return the period in a readable format.
 *               The VCSEL pulse period determines the time duration of the laser pulses emitted by the sensor, and it can vary based on the measurement mode.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * 				vcselPeriodType type 						Specifies whether to get the VCSEL pulse period for pre-range (`VcselPeriodPreRange`) or final-range (`VcselPeriodFinalRange`).
 *
 * @returns:     uint8_t: The decoded VCSEL pulse period value. Returns 255 if there was an error reading or decoding the value.
 *
 * @notes:
 *               - This function reads the sensor register corresponding to the VCSEL period for either the pre-range or final-range mode.
 *               - The raw register values are decoded before returning.
 *               - A return value of 255 indicates an error during the process (either reading the register or decoding the value).
 */
extern uint8_t TOF_get_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type);


/**
 * @function:    TOF_set_measurement_timing_budget
 *
 * @brief:       Configures the measurement timing budget for the Time-of-Flight (TOF) sensor.
 *
 * @details:     The measurement timing budget defines the total time allowed for a single measurement,
 *               which includes the time consumed by each sequence step (e.g., pre-range, final range).
 *               This function calculates the required time for enabled steps and adjusts the final range
 *               timeout to meet the specified budget.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 *               uint32_t budget_us							Desired timing budget in microseconds (µs).
 *
 * @returns:     bool: true if the timing budget was successfully set, otherwise false.
 */
extern bool TOF_set_measurement_timing_budget(TOFSensor_t* TOFSENS, uint32_t budget_us);


/**
 * @function:    TOF_get_vcsel_pulse_period
 *
 * @brief:       Retrieves the VCSEL (Vertical Cavity Surface Emitting Laser) pulse period from the TOF sensor.
 *
 * @details:     This function reads the VCSEL pulse period for either the pre-range or final-range mode of the sensor. The VCSEL period is stored in specific registers, and the function decodes the value to return the period in a readable format.
 *               The VCSEL pulse period determines the time duration of the laser pulses emitted by the sensor, and it can vary based on the measurement mode.
 *
 * @param[in]:   TOFSENS
 * 					- TOF_address_used          			The sensor's I2C address (e.g., 0x29 for VL53LOX)
 *					- Ranging_Profiles_t        			The sensor's ranging mode (e.g., HIGH_SPEED_MODE_S)
 *					- distanceFromTOF           			The current distance measurement (in mm)
 *					- measuredRange             			RAW Data of measured distance
 *					- enableTOFSensor           			Flag indicating if the sensor is enabled (true/false)
 *					- Ranging_Profile_time 	  				Time for the execution for readcontinuos in dependence of RangingProfile
 *					- TOF_readyFlag 			  			Flag indicating if the sensor data is ready to read
 *					- TOF_measuringage  		  			Age of the measured distance
 *
 * 				vcselPeriodType type 						Specifies whether to get the VCSEL pulse period for pre-range (`VcselPeriodPreRange`) or final-range (`VcselPeriodFinalRange`).
 *
 * @returns:     uint8_t: The decoded VCSEL pulse period value. Returns 255 if there was an error reading or decoding the value.
 *
 * @notes:
 *               - This function reads the sensor register corresponding to the VCSEL period for either the pre-range or final-range mode.
 *               - The raw register values are decoded before returning.
 *               - A return value of 255 indicates an error during the process (either reading the register or decoding the value).
 */
extern uint8_t TOF_get_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type);


//--------------------- ADDITIONAL NON TOF FUNCTIONS ---------------------


/**
 * @function:    encode_timeOut
 *
 * @brief:       Encodes the timeout value into a specific format used by the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function takes a timeout value in macro clocks (MCLKs) and encodes it into a register-friendly format. The encoding process involves extracting the most significant byte (MSB) and the least significant byte (LSB) from the timeout value, as per the sensor's internal requirements.
 *               The result is returned as a 16-bit encoded value that can be directly written to the sensor's timeout registers.
 *
 * @param[in]:   final_range_timeout_mclks 					The timeout value in macro clocks (MCLKs) for the final range step.
 *
 * @returns:     uint16_t: A 16-bit encoded timeout value that can be written to the sensor's timeout registers. The result is composed of:
 *                         - The most significant byte (MSB) that indicates the number of shifts used to fit the timeout value.
 *                         - The least significant byte (LSB) that holds the adjusted timeout value.

 * @limitations:
 *               - The function assumes that the input timeout value (final_range_timeout_mclks) is a valid unsigned 16-bit value.
 *               - If the input timeout is zero, the function returns 0.

 *
 * @notes:
 *               - This function is crucial for converting a timeout value in macro clocks (MCLKs) into the format that can be used by the TOF sensor.
 *               - The encoded value is a 16-bit value where the higher byte represents the number of shifts, and the lower byte holds the adjusted value.
 *               - The input timeout value should be greater than zero, as the function is designed to handle positive timeouts.
 */
extern uint16_t encode_timeOut(uint16_t final_range_timeout_mclks);


/**
 * @function:    decodeTimeout
 *
 * @brief:       Decodes the timeout value from the register format used by the Time-of-Flight (TOF) sensor.
 *
 * @details:     This function decodes a 16-bit register value, which encodes a timeout in the form of:
 *               - The most significant byte (MSB) represents how many times the least significant byte (LSB) needs to be shifted left.
 *               - The least significant byte (LSB) holds the timeout value.
 *               The decoded value is the actual timeout in macro clocks (MCLKs), which can be used for further processing.
 *
 * @param[in]:   reg_val - A 16-bit register value that represents an encoded timeout.
 *                 - The MSB (upper 8 bits) indicates the number of shifts.
 *                 - The LSB (lower 8 bits) holds the adjusted timeout value.
 *
 * @returns:     uint16_t: The decoded timeout value in macro clocks (MCLKs). This is the actual timeout value
 *                         that was encoded in the register.
 *
 * @limitations:
 *               - The function assumes that the input register value (reg_val) is a valid 16-bit encoded timeout.
 *
 * @notes:
 *               - This function is used to decode the timeout value from the format used by the TOF sensor's registers.
 *               - The formula used to calculate the timeout ensures that the timeout is a value greater than or equal to 1.
 *               - The decoded timeout can be used in further sensor configurations or calculations.
 */
extern uint16_t decode_timeout(uint16_t reg_val);


/**
 * @function:    timeoutMclksToMicroseconds
 *
 * @brief:       Converts a timeout period from macro clocks (MCLKs) to microseconds.
 *
 * @details:     This function takes a timeout value expressed in macro clocks (MCLKs) and converts it into microseconds.
 *               The calculation is based on the VCSEL period, which affects the macro period in nanoseconds.
 *               The macro period is calculated using the `calcMacroPeriod` function, which determines the time duration
 *               of each macro clock in nanoseconds. The timeout period in MCLKs is then multiplied by the macro period
 *               and converted to microseconds.
 *
 * @param[in]:   timeout_period_mclks - The timeout period in macro clocks (MCLKs).
 *               This is the raw value obtained from the sensor's registers.
 *				 vcsel_period_pclks  - The VCSEL period in photon clocks (PCLKs).
 *               This is the period in photon clocks used for the light emission cycle.
 *
 * @returns:     uint32_t: The timeout period converted to microseconds (µs).
 *               This value is calculated by multiplying the timeout period by the macro period and converting to microseconds.
 *
 *
 * @notes:
 *               - This function is used to convert the sensor's timeout period (expressed in MCLKs) into a more human-readable form (microseconds).
 *               - The calculation ensures the proper timing conversion based on the VCSEL period, which varies depending on the sensor configuration.
 */
extern uint32_t timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);


/**
 * @function:    timeout_microseconds_to_mclks
 *
 * @brief:       Converts a timeout period from microseconds to macro clocks (MCLKs).
 *
 * @details:     This function takes a timeout value in microseconds and converts it into macro clocks (MCLKs), which is the raw timing unit used by the sensor's hardware.
 *               The conversion is based on the VCSEL period, which affects the macro period in nanoseconds. The macro period is calculated using the `calcMacroPeriod` function,
 *               which determines the time duration of each macro clock in nanoseconds. The timeout period in microseconds is then converted into MCLKs.
 *
 * @param[in]:   timeout_period_us - The timeout period in microseconds (µs).
 *               This value is usually obtained from the sensor configuration or measurement timing budget.
 *				 vcsel_period_pclks  - The VCSEL period in photon clocks (PCLKs).
 *               This is the period used for the light emission cycle, which affects the macro clock timing.
 *
 * @returns:     uint32_t: The timeout period converted to macro clocks (MCLKs).
 *               This value represents the timeout period in macro clock cycles based on the VCSEL period and microseconds input.
 *
 * @notes:
 *               - This function is used to convert a timeout period from microseconds into the raw timing unit (MCLKs), which is suitable for sensor register configurations.
 *               - The conversion takes into account the VCSEL period (in PCLKs) to properly adjust for different sensor configurations.
 */
extern uint32_t timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

#endif /* SENSORTOF_H */
