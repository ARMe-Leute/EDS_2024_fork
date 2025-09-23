/**
 ******************************************************************************
 * @file           : i2cTOF.c
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief          : This library file handles communication with a Time-of-Flight (TOF) sensor.
 *                    Currently adapted to the VL53LOX sensor.
 *                    The library supports both single-shot and continuous measurement modes.
 *                    Configuration is managed via the appropriate register settings for the sensor.
 * @date           : April 2025
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
 ******************************************************************************
 */

// standard includes
#include <stdbool.h>

// includes from mcal and cmsis
#include <stm32f4xx.h>
#include <mcalI2C.h>

// inits from Balancer library
#include <i2cTOF.h>
#include <ST7735.h>
#include <BALO.h>

// Variables for I2C and TOF sensor address
TOF_ADDR_t TOF_address_used = TOF_ADDR_NONE; // TOF sensor address, initially set to NONE (-1)
I2C_TypeDef *TOF_i2c;  // Pointer to the I2C peripheral (e.g., I2C1, I2C2)

// TOF stop variable for register store value (used to store certain register values temporarily)
static uint8_t TOF_stop_variable = 0;

// Flag for continuous mode (Indicates if the sensor is in continuous measurement mode)
bool TOF_continuous_mode = false;

// Math macros for sensor configuration and period decoding

// Calculate macro period based on VCSEL period (pulses per clock)
#define calcMacroPeriod(vcsel_period_pclks) (((uint32_t)(2304) * (vcsel_period_pclks) * 1655 + 500) / 1000)

// Decode VCSEL period based on register value
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)


//--------------------- SENSOR FUNCTIONS ------------------------

// Function to initialize the TOF sensor
// Initializes the sensor with given address, I2C interface, ranging profile, and measurement range.
void initializeTOFSensor(TOFSensor_t* sensor, I2C_TypeDef *i2c_tof, uint16_t TOF_address_used, uint16_t Ranging_Profiles_t, uint16_t measuredRange) {
	sensor->i2c_tof = i2c_tof;                    // Set the I2C interface (e.g., I2C1, I2C2)
	sensor->TOF_address_used = TOF_address_used;  // Set the TOF sensor address
    sensor->Ranging_Profiles_t = Ranging_Profiles_t;  // Set the ranging profile (measurement mode)
    sensor->measuredRange = measuredRange;        // Set the maximum measurable range
    sensor->distanceFromTOF = 0;                  // Initialize the measured distance to zero
    sensor->enableTOFSensor = false;              // Default is to keep the sensor disabled
}

// Function to configure the TOF sensor
// Configures the sensor's ranging profile and whether the sensor should be enabled or disabled.
void configureTOFSensor(TOFSensor_t* sensor, uint16_t Ranging_Profiles_t, bool enable) {
    sensor->Ranging_Profiles_t = Ranging_Profiles_t;  // Set the new ranging profile
    sensor->enableTOFSensor = enable;                  // Enable or disable the TOF sensor
    if(sensor->enableTOFSensor == true)
    {
    	TOF_set_ranging_profile(sensor);
    }
}


//---------------------INTERNAL FUNCTIONS---------------------

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
 *
 * @returns:     bool: true if the configuration is successful, otherwise false.
 */
bool TOF_configure_interrupt(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	/* Interrupt on new sample ready */
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    if (i2c_return != I2C_OK)
    {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
    uint8_t gpio_hv_mux_active_high[1];
    i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high, 1);
    if (i2c_return != I2C_OK)
	{
		return false;
	}

    gpio_hv_mux_active_high[0] &= ~0x10;
    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH,  gpio_hv_mux_active_high[0]);
    if (i2c_return != I2C_OK)
	{
		return false;
	}

    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
	if (i2c_return != I2C_OK)
	{
		return false;
	}

    return true;
}


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
bool TOF_init_address(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	//activate GPIO if not already done
	//gpioActivate();

	//set i2c clock speed
	//i2cSetClkSpd(TOF_i2c, I2C_CLOCK_400);

	//check correct device by reading the ID
	uint8_t device_id[1];
	I2C_RETURN_CODE_t i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_IDENTIFICATION_MODEL_ID, device_id, 1);

	if (i2c_return != I2C_OK)
	{
		// returns false, if i2cBurstRegRead was not successful
		return false;
	}

	//returns true, if correct TOF sensor is connected, otherwise false
	return (device_id[0] == TOF_VL53L0X_EXPECTED_DEVICE_ID);
}


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
bool TOF_data_init(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success = false;

	/* Set 2v8 mode */
	uint8_t vhv_config_scl_sda = 0;
	success = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda);
	if (success != I2C_OK)
	{
		return false;
	}

	vhv_config_scl_sda |= 0x01;

	success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda);
	if (success != I2C_OK)
	{
		return false;
	}

	/* Set I2C standard mode */
	success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_I2C_MODE, 0x00);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	success &= i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, &TOF_stop_variable);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);

	if (success != I2C_OK)
	{
		return false;
	}

	return true;
}


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
bool TOF_get_spad_info_from_nvm(TOFSensor_t* TOFSENS, uint8_t * count, bool * type_is_aperture)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t tmp;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);

	uint8_t data = 0;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x06);
	i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, &data);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, data | 0x04);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x07);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, 0x01);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x94, 0x6b);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, 0x00);

	data = 0;
	I2C_RETURN_CODE_t i2c_return;

	do
		{
			i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_INTERNAL_CONFIG_0x83, &data, 1);
		} while (i2c_return == I2C_OK && data == 0x00);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, 0x01);
	i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, 0x92, &tmp);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	data = 0;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_HISTOGRAM_NON, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x06);
	i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, &data);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, data  & ~0x04);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);

	return true;
}


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
bool TOF_set_spads_from_nvm(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t spad_count;
	bool spad_type_is_aperture;
	if (!TOF_get_spad_info_from_nvm(TOFSENS, &spad_count, &spad_type_is_aperture))
	{
		return false;
	}

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
		if (i < first_spad_to_enable || spads_enabled == spad_count)
		{
		// This bit is lower than the first one that should be enabled, or
		// (reference_spad_count) bits have already been enabled, so zero this bit
		ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
		{
		spads_enabled++;
		}
	}

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[0]);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 1, ref_spad_map[1]);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 2, ref_spad_map[2]);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 3, ref_spad_map[3]);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 4, ref_spad_map[4]);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 5, ref_spad_map[5]);

	return true;
}


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
bool TOF_load_default_tuning_settings(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_RANGE_CONFIG, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x10, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x11, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x24, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x25, TOF_DEFAULT_0xFF);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x75, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, TOF_DEFAULT_0x2C);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_ALGO_PHASECAL, TOF_DEFAULT_0x20);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_ALGO_PHASECAL, TOF_DEFAULT_0x09);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x54, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x31, TOF_DEFAULT_0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, TOF_DEFAULT_0x03);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_HISTOGRAM_BIN, TOF_DEFAULT_0x83);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, TOF_DEFAULT_0x25);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_CONTROL, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_MIN_SNR, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, TOF_DEFAULT_0x06);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, TOF_DEFAULT_0x96);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, TOF_DEFAULT_0x08);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, TOF_DEFAULT_0x30);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x65, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x66, TOF_DEFAULT_0xA0);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x22, TOF_DEFAULT_0x32);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, TOF_DEFAULT_0x14);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x49, TOF_DEFAULT_0xFF);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4A, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x7A, TOF_DEFAULT_0xA0);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x7B, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x78, TOF_DEFAULT_0x21);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x23, TOF_DEFAULT_0x34);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x42, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, TOF_DEFAULT_0xFF);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x45, TOF_DEFAULT_0x26);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, TOF_DEFAULT_0x05);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_HISTOGRAM_BIN, TOF_DEFAULT_0x40);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_THRESH_LOW, TOF_DEFAULT_0x06);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x20, TOF_DEFAULT_0x1A);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x43, TOF_DEFAULT_0x40);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x34, TOF_DEFAULT_0x03);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x35, TOF_DEFAULT_0x44);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x31, TOF_DEFAULT_0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4B, TOF_DEFAULT_0x09);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4C, TOF_DEFAULT_0x05);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4D, TOF_DEFAULT_0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x45, TOF_DEFAULT_0x20);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, TOF_DEFAULT_0x08);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, TOF_DEFAULT_0x28);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_SNR, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, TOF_DEFAULT_0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, TOF_DEFAULT_0xFE);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x76, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x77, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x0D, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, TOF_DEFAULT_0xF8);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x8E, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, TOF_DEFAULT_0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, TOF_DEFAULT_0x00);

    if (success != I2C_OK)
	{
		return false;
	}

	return true;
}


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
bool TOF_set_sequence_steps_enabled(TOFSensor_t* TOFSENS, uint8_t sequence_step)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	bool result = false;

	I2C_RETURN_CODE_t success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);

	if(success == I2C_OK)
	{
		result = true;
	}

	return result;
}


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
bool TOF_perform_single_ref_calibration(TOFSensor_t* TOFSENS, TOF_calibration_type_t calib_type)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success;

    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type)
    {
    case TOF_CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case TOF_CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }

    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);
    if (success != I2C_OK)
    {
        return false;
    }
//ToDo Abfrage ob die jeweiligen Initialisierungen richtig gesetzt wurden.
    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, sysrange_start);
    if (success != I2C_OK)
    {
        return false;
    }

    /* Wait for interrupt */



    uint8_t interrupt_status = 0;
    do {		//Funktion in welcher der MCAL Fehler auftritt //ToDo i2cReadByteFromSlaveReg durch i2cBurstRegRead ersetzen

        //success = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
        success = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status, 1);
    } while ((interrupt_status & 0x07) == 0);

    //} while (success == I2C_OK && ((interrupt_status & 0x07) == 0));
    if (success != I2C_OK)
    {
        return false;
    }

    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (success != I2C_OK)
    {
        return false;
    }

    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
    if (success != I2C_OK)
    {
        return false;
    }

    return true;
}


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
bool TOF_perform_ref_calibration(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if (!TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_VHV)) {
		return false;
	}

	if (!TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_PHASE)) {		//Funktion in welcher der MCAL Fehler auftritt
		return false;
	}


	/* Restore sequence steps enabled */

	if (!TOF_set_sequence_steps_enabled(TOFSENS, TOF_RANGE_SEQUENCE_STEP_DSS + TOF_RANGE_SEQUENCE_STEP_PRE_RANGE + TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}

	return true;
}


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
bool TOF_init_device(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if (!TOF_data_init(TOFSENS))
	{
		return false;
	}

	if (!TOF_set_spads_from_nvm(TOFSENS)) {
		return false;
	}

	if (!TOF_load_default_tuning_settings(TOFSENS))
	{
		return false;
	}

	if (!TOF_configure_interrupt(TOFSENS))
	{
		return false;
	}

	if (!TOF_set_sequence_steps_enabled(TOFSENS, TOF_RANGE_SEQUENCE_STEP_DSS + TOF_RANGE_SEQUENCE_STEP_PRE_RANGE + TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}

    if (!TOF_perform_ref_calibration(TOFSENS))			//Funktion in welcher der MCAL Fehler auftritt
    {
        return false;
    }

	return true;
}


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
bool TOF_getMeasurement(TOFSensor_t* TOFSENS, uint16_t *range)
{
	I2C_RETURN_CODE_t i2c_return;
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;
	uint8_t interrupt_status[1];
	do
	{
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_RESULT_INTERRUPT_STATUS, interrupt_status, 1);
	} while (i2c_return == I2C_OK && ((interrupt_status[0] & 0x07) == 0));
	if (i2c_return != I2C_OK)
	{
		return false;
	}

	uint8_t readBuffer[2];
	i2c_return = i2cBurstRegRead(
			TOF_i2c, TOF_address_used,
			TOF_REG_RESULT_RANGE_STATUS + 10,
			readBuffer, 2);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}
	*range = (readBuffer[0] << 8) + readBuffer[1];

	TOFSENS->measuredRange = (uint32_t)readBuffer;
	i2c_return = i2cSendByteToSlaveReg(
			TOF_i2c, TOF_address_used,
			TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}

	/* 8190 or 8191 may be returned when obstacle is out of range. */
	if (*range == 8190 || *range == 8191)
	{
		*range = TOF_VL53L0X_OUT_OF_RANGE;
	}

	return true;
}


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
bool TOF_init(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	// Init i2c address and check connectivity
	if (!TOF_init_address(TOFSENS))
	{
		return false;
	}

	//device initialization
	if (!TOF_init_device(TOFSENS))		//Funktion in welcher der MCAL Fehler auftritt
	{
		return false;
	}

	// return true, if everything was fine
	return true;
}


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
bool TOF_start_continuous(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint32_t period_ms = TOFSENS->Ranging_Profile_time;
	I2C_RETURN_CODE_t i2c_return;

	TOF_address_used = TOFSENS->TOF_address_used;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);

	if (period_ms != 0)
	{
		// continuous timed mode

		uint8_t readBuffer[2];

		i2c_return = i2cBurstRegRead(
				TOF_i2c, TOF_address_used,
				TOF_REG_OSC_CALIBRATE_VAL,
				readBuffer, 2);
		if (i2c_return != I2C_OK)
		{
			// returns false, if i2c communication was not successful
			return false;
		}

		uint16_t osc_calibrate_val = (readBuffer[0]<<8) + readBuffer[1];

		if (osc_calibrate_val != 0)
		{
			period_ms *= osc_calibrate_val;
		}

		uint8_t bytes[4];

		bytes[0] = (period_ms >> 24) & 0xFF;
		bytes[1] = (period_ms >> 16) & 0xFF;
		bytes[2] = (period_ms >> 8) & 0xFF;
		bytes[3] = period_ms & 0xFF;

		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD, bytes[0]);
		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD + 1, bytes[1]);
		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD + 2, bytes[2]);
		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD + 3, bytes[3]);

		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	}
	else
	{
		// continuous back-to-back mode
		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}

	TOF_continuous_mode = true;

	return true;
}


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
bool TOF_stop_continuous(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if(!TOF_continuous_mode)
	{
		return false;
	}

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);

	TOF_continuous_mode = false;

	return true;
}


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
bool TOF_read_continuous_distance(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if(!TOF_continuous_mode)
	{
		return false;
	}

	if(!TOF_getMeasurement(TOFSENS, &TOFSENS->distanceFromTOF))
	{
		return false;
	}

	return true;
}


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
bool TOF_read_single_distance(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
	if (i2c_return != I2C_OK) {
		return false;
	}


	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}

	uint8_t sysrange_start[1];
	do
	{
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, sysrange_start, 1);
	} while (i2c_return == I2C_OK && (sysrange_start[0] & 0x01));
	if (i2c_return != I2C_OK)
	{
		return false;
	}
	TOF_getMeasurement(TOFSENS, &TOFSENS->distanceFromTOF);


	return true;
}


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
bool TOF_start_up_task(TOFSensor_t* TOFSENS)
{
	I2C_RETURN_CODE_t i2c_return;
	TOF_address_used = TOFSENS->TOF_address_used;	//TOF Adress from Struct TOF
	TOF_i2c = TOFSENS->i2c_tof;		//I2C Adress from Struct TOF

	//start new measurement without reading
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	if (i2c_return != I2C_OK)
	{
		return false;
	}
	return true;
}


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
bool TOF_read_distance_task(TOFSensor_t* TOFSENS)
{
	I2C_RETURN_CODE_t i2c_return;
	TOF_address_used = TOFSENS->TOF_address_used;	//TOF Adress from Struct TOF
	TOF_i2c = TOFSENS->i2c_tof;		//I2C Adress from Struct TOF
	uint8_t interrupt_status[1];	//Variable for the Register Content
	uint16_t taskdistance;			//Variable to Store the read distance

	//check the ReadyData Flag
	i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_RESULT_INTERRUPT_STATUS, interrupt_status, 1);
	if (i2c_return != I2C_OK)
	{
		return false;
	}

	//readydata Flag high: read measurement result
	if(i2c_return == I2C_OK && ((interrupt_status[0] & 0x07) != 0))
	{
		uint8_t readBuffer[2];
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_RESULT_RANGE_STATUS + 10, readBuffer, 2);
		if (i2c_return != I2C_OK)
		{
			return false;
		}

		taskdistance = (readBuffer[0] << 8) + readBuffer[1];

		TOFSENS->measuredRange = (uint32_t)readBuffer;

		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (i2c_return != I2C_OK)
		{
			return false;
		}

		if (taskdistance == 8190 || taskdistance == 8191)
		{
			taskdistance = TOF_VL53L0X_OUT_OF_RANGE;
		}


		TOFSENS->distanceFromTOF = taskdistance;


		//Successfull Measurement start new one
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
		if (i2c_return != I2C_OK)
		{
			return false;
		}

		TOFSENS->TOF_measuringage = 0; 		//reset measuring age
	}


	else		//readydata Flag LOW !
	{
		TOFSENS->TOF_measuringage ++;

	}
	return true;

}


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
bool TOF_set_address(TOFSensor_t* TOFSENS, uint8_t new_Addr)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

    I2C_RETURN_CODE_t i2c_return;

    uint8_t newaddr = new_Addr;

    // Send the new address to the device
    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x8A, newaddr & 0x7F);

    if (i2c_return != I2C_OK) {
        return false; // Return false if the operation fails
    }
    TOFSENS->TOF_address_used = newaddr;
    return true;
}


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
bool TOF_read_distance_timed(TOFSensor_t* TOFSENS, uint16_t time, uint16_t *range)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	delayms(time);

	I2C_RETURN_CODE_t i2c_return;

	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
	if (i2c_return != I2C_OK) {
		return false;
	}


	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}

	uint8_t sysrange_start[1];
	do
	{
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_SYSRANGE_START,	sysrange_start, 1);
	}

	while (i2c_return == I2C_OK && (sysrange_start[0] & 0x01));
		if (i2c_return != I2C_OK)
		{
			return false;
		}

	TOF_getMeasurement(TOFSENS, range);

	return true;
}


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
bool TOF_set_ranging_profile(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	bool value = false;
	bool prevalue = false;
    switch (TOFSENS->Ranging_Profiles_t) {
    case TOF_DEFAULT_MODE_D:
    	if(TOF_set_measurement_timing_budget(TOFSENS, 30000) == true)
    	{
    		TOFSENS->Ranging_Profiles_t = TOF_DEFAULT_MODE_D;
    		TOFSENS->Ranging_Profile_time = 30;
    		value = true;
    		break;
    	}
    	else
    	{
        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
        	value = TOF_RANGINGPROFILE_ERROR;
    		return false;
    		break;
    	}

    case TOF_HIGH_SPEED_MODE_S:
        if(TOF_set_measurement_timing_budget(TOFSENS, 20000) == true)
        {
        	TOFSENS->Ranging_Profiles_t = TOF_HIGH_SPEED_MODE_S;
        	TOFSENS->Ranging_Profile_time = 20;
        	value = true;
        	break;
        }
        else
        {
        	value = TOF_RANGINGPROFILE_ERROR;
        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;

        	break;
        }

    case TOF_HIGH_ACCURACY_MODE_A:
        if(TOF_set_measurement_timing_budget(TOFSENS, 200) == true)
        {
        	TOFSENS->Ranging_Profiles_t = TOF_HIGH_ACCURACY_MODE_A;
        	TOFSENS->Ranging_Profile_time = 200000;
        	value = true;
        	break;
        }
        else
        {
        	value = TOF_RANGINGPROFILE_ERROR;
        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
        	break;
        }

    case TOF_LONG_RANGE_MODE_R:
    	if(TOF_set_measurement_timing_budget(TOFSENS, 33) == true)
    	        {
    	        	value = true;
    	        	break;
    	        }
    	        else
    	        {
    	        	value = TOF_RANGINGPROFILE_ERROR;
    	        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    	        	break;
    	        }

        if(TOF_set_signal_rate_limit(TOFSENS, 0.1) == true)
    	{
    		prevalue = true;
    		value = true;

    	}
    	else
    	{
    		prevalue = false;
        	value = TOF_RANGINGPROFILE_ERROR;
    		TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    		break;
    	}

    	if(TOF_set_vcsel_pulse_period(TOFSENS, VcselPeriodPreRange, 18) == true && prevalue == true)
    	{
    		prevalue = true;
    	}
    	else
    	{

        	value = TOF_RANGINGPROFILE_ERROR;
    		TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    		break;
    	}

    	if(TOF_set_vcsel_pulse_period(TOFSENS, VcselPeriodFinalRange, 14) == true && prevalue == true)
    	{
        	TOFSENS->Ranging_Profiles_t = TOF_LONG_RANGE_MODE_R;
        	TOFSENS->Ranging_Profile_time = 33000;

    		break;
    	}

    	else
    	{
    		prevalue = false;
        	value = TOF_RANGINGPROFILE_ERROR;
    		TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    		break;
    	}

    default:
        // Handle an invalid profile case
        return value;
    }

    return value;
}


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
bool TOF_set_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type, uint8_t period_pclks)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
	I2C_RETURN_CODE_t i2c_return;

	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	// Get the current sequence step enables and timeouts from the sensor
	TOF_get_sequence_step_enables(TOFSENS, &enables);
	TOF_get_sequence_step_timeouts(TOFSENS, &enables, &timeouts);

	// Apply specific settings for the requested VCSEL period
	if (type == VcselPeriodPreRange)
	{
		// Set phase check limits based on the requested period
		switch (period_pclks)
		{
			case 12:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 14:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 16:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 18:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			default:
				return false;  // Invalid VCSEL period for pre-range
		}
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

		// Apply new VCSEL period for pre-range
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

		// Update timeouts for pre-range
		uint16_t new_pre_range_timeout_mclks = timeout_microseconds_to_mclks(timeouts.pre_range_us, period_pclks);
		new_pre_range_timeout_mclks = encode_timeOut(new_pre_range_timeout_mclks);
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_pre_range_timeout_mclks);

		// Update MSRC timeout
		uint16_t new_msrc_timeout_mclks = timeout_microseconds_to_mclks(timeouts.msrc_dss_tcc_us, period_pclks);
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
	}
	else if (type == VcselPeriodFinalRange)
	{
		// Set phase check limits for final-range VCSEL period
		switch (period_pclks)
		{
			case 8:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 10:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 12:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 14:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			default:
				return false;  // Invalid VCSEL period for final-range
		}

		// Apply new VCSEL period for final-range
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		if (i2c_return != I2C_OK) {
			return false;
		}
		// Update timeouts for final-range
		uint16_t new_final_range_timeout_mclks = timeout_microseconds_to_mclks(timeouts.final_range_us, period_pclks);
		if (enables.pre_range)
		{
			new_final_range_timeout_mclks += timeouts.pre_range_mclks;
		}
		new_final_range_timeout_mclks = encode_timeOut(new_final_range_timeout_mclks);
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_final_range_timeout_mclks);
		if (i2c_return != I2C_OK) {
			return false;
		}
	}
	else
	{
		return false;  // Invalid type
	}

	// Re-apply the timing budget
	//setMeasurementTimingBudget(measurement_timing_budget_us);

	// Perform phase calibration if needed

	uint8_t sequence_config;
	i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, &sequence_config);
		if (i2c_return != I2C_OK) {
			return false; // Return false if the I2C read fails
		}
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (i2c_return != I2C_OK) {
		return false;
	}
	TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_VHV);
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);
	if (i2c_return != I2C_OK) {
		return false;
	}
	return true;
}


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
bool TOF_set_signal_rate_limit(TOFSensor_t* TOFSENS, float signalRateLimit)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	float limitMCPS = signalRateLimit;
	  if (limitMCPS < 0 || limitMCPS > 511.99) {
		  return false;
	  }

	  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	  i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limitMCPS * (1 << 7));
	  if (i2c_return != I2C_OK) {
	          return false; // Return false if the I2C read fails
	      }
	  return true;

}


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
bool TOF_get_sequence_step_enables(TOFSensor_t* TOFSENS, SequenceStepEnables *enables)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

    I2C_RETURN_CODE_t i2c_return;
    uint8_t sequence_config;

    // Read the byte from the TOF_REG_SYSTEM_SEQUENCE_CONFIG register
    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, &sequence_config);

    // Check if the I2C read was successful
    if (i2c_return != I2C_OK)
    {
        return false;  // Return false if there was an error
    }

    // Extract bit values using shifts and masks
    enables->tcc         = (sequence_config >> 4) & 0x1;  // Extract the TCC bit
    enables->dss         = (sequence_config >> 3) & 0x1;  // Extract the DSS bit
    enables->msrc        = (sequence_config >> 2) & 0x1;  // Extract the MSRC bit
    enables->pre_range   = (sequence_config >> 6) & 0x1;  // Extract the PRE_RANGE bit
    enables->final_range = (sequence_config >> 7) & 0x1;  // Extract the FINAL_RANGE bit

    return true;  // Return true if everything succeeded
}


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
bool TOF_get_sequence_step_timeouts(TOFSensor_t* TOFSENS, SequenceStepEnables *enables, SequenceStepTimeouts *timeouts)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t data;
	I2C_RETURN_CODE_t i2c_return;

    timeouts->pre_range_vcsel_period_pclks = TOF_get_vcsel_pulse_period(TOFSENS, VcselPeriodPreRange);

    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, &data);
    timeouts->msrc_dss_tcc_mclks = data;

	if (i2c_return != I2C_OK)
	{
		return false;
	}
    timeouts->msrc_dss_tcc_mclks += 1;
    timeouts->msrc_dss_tcc_us = timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data);
    timeouts->pre_range_mclks = data;
    if (i2c_return != I2C_OK)
    	{
    		return false;
    	}
    timeouts->pre_range_mclks = decode_timeout(timeouts->pre_range_mclks);
    timeouts->pre_range_us = timeout_mclks_to_microseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = TOF_get_vcsel_pulse_period(TOFSENS, VcselPeriodFinalRange);


    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data);
    timeouts->pre_range_mclks = data;
    if (i2c_return != I2C_OK)
    	{
    		return false;
    	}
    timeouts->final_range_mclks = decode_timeout(timeouts->final_range_mclks);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeout_mclks_to_microseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);

    return true;
}


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
bool TOF_set_measurement_timing_budget(TOFSensor_t* TOFSENS, uint32_t budget_us)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t StartOverhead     = 1910;
    uint16_t EndOverhead       = 960;
    uint16_t MsrcOverhead      = 660;
    uint16_t TccOverhead       = 590;
    uint16_t DssOverhead       = 690;
    uint16_t PreRangeOverhead  = 660;
    uint16_t FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    // Get sequence step enables and timeouts
    TOF_get_sequence_step_enables(TOFSENS, &enables);
    TOF_get_sequence_step_timeouts(TOFSENS, &enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us)
        {
            // Requested timeout too big
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // Convert the final range timeout to MCLks
        uint32_t final_range_timeout_mclks =
        timeout_microseconds_to_mclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        // Write the final range timeout to the register

        final_range_timeout_mclks = encode_timeOut(final_range_timeout_mclks);
        i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, final_range_timeout_mclks);
        if (i2c_return != I2C_OK){
            		return false;
            	}
        // Store the timing budget for internal reuse
        //uint32_t measurement_timing_budget_us = budget_us;
    }

    return true;
}


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
uint8_t TOF_get_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

    uint8_t vcsel_period = 255;  // Default to 255 (error case)

    if (type == VcselPeriodPreRange)
    {
        // Read the pre-range VCSEL period register and decode
        i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period);
    	if (i2c_return != I2C_OK){
    		return false;
    	}
        vcsel_period = decodeVcselPeriod(vcsel_period);
    }
    else if (type == VcselPeriodFinalRange)
    {
        // Read the final range VCSEL period register and decode
        i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period);
    	if (i2c_return != I2C_OK){
    		return false;
    	}
    	vcsel_period = decodeVcselPeriod(vcsel_period);
    }

    return vcsel_period;
}


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
uint16_t encode_timeOut(uint16_t final_range_timeout_mclks)
{
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (final_range_timeout_mclks > 0){
	ls_byte = final_range_timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0){
	      ls_byte >>= 1;
	      ms_byte++;
	    }

	    return (ms_byte << 8) | (ls_byte & 0xFF);
	  }
	  else { return 0; }
}


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
uint16_t decode_timeout(uint16_t reg_val)
{
    // Formula: (LSByte * 2^MSByte) + 1
    // reg_val is a 16-bit value; the MSByte (Most Significant Byte) is the upper 8 bits
    // and the LSByte (Least Significant Byte) is the lower 8 bits.

    uint8_t msb = (reg_val >> 8) & 0xFF;  // Extract the most significant byte
    uint8_t lsb = reg_val & 0xFF;         // Extract the least significant byte

    // Calculate the timeout as per the formula
    uint16_t timeout = (lsb << msb) + 1;

    return timeout;
}


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
uint32_t timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert the timeout period in MCLKs to microseconds

    return (uint32_t)((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}


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
uint32_t timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert timeout from microseconds to MCLKs
    // The formula uses rounding by adding (macro_period_ns / 2) before dividing
    uint32_t return_value = (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    return return_value;
}
