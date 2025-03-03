/**
 ******************************************************************************
 * @file           : SensorTOF.c
 * @author         : Andreas Ladner & Philipp Röhlke
 * @brief          : This library file handles communication with a Time-of-Flight (TOF) sensor.
 *                    Currently adapted to the VL53LOX sensor.
 *                    The library supports both single-shot and continuous measurement modes.
 *                    Configuration is managed via the appropriate register settings for the sensor.
 * @date           : December 2024
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
#include <SensorTOF.h>
#include <ST7735.h>
#include <Balancer.h>

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

bool TOF_configure_interrupt(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	/* Interrupt on new sample ready */
	i2c_return = i2cSendByteToSlaveReg(
			TOF_i2c, TOF_address_used,
			TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    if (i2c_return != I2C_OK)
    {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
    uint8_t gpio_hv_mux_active_high[1];
    i2c_return = i2cBurstRegRead(
		TOF_i2c, TOF_address_used,
		TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH,
		gpio_hv_mux_active_high, 1);
    if (i2c_return != I2C_OK)
	{
		return false;
	}

    gpio_hv_mux_active_high[0] &= ~0x10;
    i2c_return = i2cSendByteToSlaveReg(
		TOF_i2c, TOF_address_used,
		TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH,  gpio_hv_mux_active_high[0]);
    if (i2c_return != I2C_OK)
	{
		return false;
	}

    i2c_return = i2cSendByteToSlaveReg(
		TOF_i2c, TOF_address_used,
		TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
	if (i2c_return != I2C_OK)
	{
		return false;
	}

    return true;
}


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
	I2C_RETURN_CODE_t i2c_return = i2cBurstRegRead(
			TOF_i2c, TOF_address_used,
			TOF_REG_IDENTIFICATION_MODEL_ID,
			device_id, 1);

	if (i2c_return != I2C_OK)
	{
		// returns false, if i2cBurstRegRead was not successful
		return false;
	}

	//returns true, if correct TOF sensor is connected, otherwise false
	return (device_id[0] == TOF_VL53L0X_EXPECTED_DEVICE_ID);
}


bool TOF_data_init(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success = false;

	/* Set 2v8 mode */
	uint8_t vhv_config_scl_sda = 0;
	success = i2cReadByteFromSlaveReg(
			TOF_i2c, TOF_address_used,
			TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda);
	if (success != I2C_OK)
	{
		return false;
	}

	vhv_config_scl_sda |= 0x01;

	success = i2cSendByteToSlaveReg(
		TOF_i2c, TOF_address_used,
		TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda);
	if (success != I2C_OK)
	{
		return false;
	}

	/* Set I2C standard mode */
	success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x88, 0x00);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);
	success &= i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, 0x91, &TOF_stop_variable);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
	success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x00);

	if (success != I2C_OK)
	{
		return false;
	}

	return true;
}


bool TOF_get_spad_info_from_nvm(TOFSensor_t* TOFSENS, uint8_t * count, bool * type_is_aperture)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t tmp;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);

	uint8_t data = 0;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x06);
	i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, 0x83, &data);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x83, data | 0x04);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x07);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x81, 0x01);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x94, 0x6b);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x83, 0x00);

	data = 0;
	I2C_RETURN_CODE_t i2c_return;

	do
		{
			i2c_return = i2cBurstRegRead(
					TOF_i2c, TOF_address_used,
					0x83,
					&data, 1);
		} while (i2c_return == I2C_OK && data == 0x00);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x83, 0x01);
	i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, 0x92, &tmp);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	data = 0;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x81, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x06);
	i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, 0x83, &data);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x83, data  & ~0x04);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x00);

	return true;
}


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

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
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


bool TOF_load_default_tuning_settings(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x09, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x10, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x11, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x24, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x25, 0xFF);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x75, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x4E, 0x2C);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x48, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x30, 0x20);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x30, 0x09);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x54, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x31, 0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x32, 0x03);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x40, 0x83);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x46, 0x25);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x60, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x27, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x50, 0x06);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x51, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x52, 0x96);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x56, 0x08);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x57, 0x30);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x61, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x62, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x64, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x65, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x66, 0xA0);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x22, 0x32);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x47, 0x14);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x49, 0xFF);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x4A, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x7A, 0x0A);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x7B, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x78, 0x21);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x23, 0x34);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x42, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x44, 0xFF);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x45, 0x26);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x46, 0x05);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x40, 0x40);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x0E, 0x06);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x20, 0x1A);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x43, 0x40);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x34, 0x03);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x35, 0x44);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x31, 0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x4B, 0x09);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x4C, 0x05);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x4D, 0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x44, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x45, 0x20);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x47, 0x08);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x48, 0x28);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x67, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x70, 0x04);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x71, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x72, 0xFE);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x76, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x77, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x0D, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x01, 0xF8);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x8E, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
    success &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x00);

    if (success != I2C_OK)
	{
		return false;
	}

	return true;
}


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

    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, sysrange_start);
    if (success != I2C_OK)
    {
        return false;
    }

    /* Wait for interrupt */
    uint8_t interrupt_status = 0;
    do {
        success = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success == I2C_OK && ((interrupt_status & 0x07) == 0));
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


bool TOF_perform_ref_calibration(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if (!TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_VHV)) {
		return false;
	}
	if (!TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_PHASE)) {
		return false;
	}
	/* Restore sequence steps enabled */
	if (!TOF_set_sequence_steps_enabled(TOFSENS, TOF_RANGE_SEQUENCE_STEP_DSS +
			TOF_RANGE_SEQUENCE_STEP_PRE_RANGE +
			TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}
	return true;
}


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

	if (!TOF_set_sequence_steps_enabled(TOFSENS, TOF_RANGE_SEQUENCE_STEP_DSS +
			TOF_RANGE_SEQUENCE_STEP_PRE_RANGE +
			TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}

    if (!TOF_perform_ref_calibration(TOFSENS)) {
        return false;
    }

	return true;
}


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
	if (!TOF_init_device(TOFSENS))
	{
		return false;
	}

	// return true, if everything was fine
	return true;
}


bool TOF_start_continuous(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint32_t period_ms = TOFSENS->Ranging_Profile_time;
	I2C_RETURN_CODE_t i2c_return;

	TOF_address_used = TOFSENS->TOF_address_used;

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x91, TOF_stop_variable);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x00);

	if (period_ms != 0)
	{
		// continuous timed mode

		uint8_t readBuffer[2];

		i2c_return = i2cBurstRegRead(
				TOF_i2c, TOF_address_used,
				TOF_OSC_CALIBRATE_VAL,
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

		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_SYSTEM_INTERMEASUREMENT_PERIOD, bytes[0]);
		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_SYSTEM_INTERMEASUREMENT_PERIOD + 1, bytes[1]);
		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_SYSTEM_INTERMEASUREMENT_PERIOD + 2, bytes[2]);
		i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_SYSTEM_INTERMEASUREMENT_PERIOD + 3, bytes[3]);

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


bool TOF_stop_continuous(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if(!TOF_continuous_mode)
	{
		return false;
	}

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x91, 0x00);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);
	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);

	TOF_continuous_mode = false;

	return true;
}


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


bool TOF_read_single_distance(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x91, TOF_stop_variable);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x00);
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


uint16_t TOF_read_distance_Task(TOFSensor_t* TOFSENS)
{

	I2C_RETURN_CODE_t i2c_return;
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;
	uint32_t interrupt_status = 0;		//Vlt clearn??
	uint8_t interrupt_bit = 0;
	uint16_t taskdistance;

	//check the readydata Flag

	i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used,	TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
	if (i2c_return != I2C_OK)
	{
		return false;
	}
	interrupt_bit = interrupt_status & 0x01;


	//readydata Flag high ?
	if(interrupt_bit != 0)			//hier muss dann das interrupt_bit stehen
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

		i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used,	TOF_REG_RESULT_INTERRUPT_STATUS, interrupt_status);
		//Successfull Measurement start new one
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x91, TOF_stop_variable);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
		i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x00);
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

		TOFSENS->TOF_measuringage = 0; 		//reset measuring age
	}

	//readydata Flag LOW !
	else
	{
		TOFSENS->TOF_measuringage ++;

	}









//-------------------------------------------
//Read Single distance


}



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
    return true; // Ensure the function always returns a value
}


bool TOF_read_distance_timed(TOFSensor_t* TOFSENS, uint16_t time, uint16_t *range)
{

	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	delayms(time);

	I2C_RETURN_CODE_t i2c_return;

	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x91, TOF_stop_variable);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x00, 0x01);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0xFF, 0x00);
	i2c_return &= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x80, 0x00);
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
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 14:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 16:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 18:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			default:
				return false;  // Invalid VCSEL period for pre-range
		}
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

		// Apply new VCSEL period for pre-range
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

		// Update timeouts for pre-range
		uint16_t new_pre_range_timeout_mclks = timeout_microseconds_to_mclks(timeouts.pre_range_us, period_pclks);
		new_pre_range_timeout_mclks = encode_timeOut(new_pre_range_timeout_mclks);
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_pre_range_timeout_mclks);

		// Update MSRC timeout
		uint16_t new_msrc_timeout_mclks = timeout_microseconds_to_mclks(timeouts.msrc_dss_tcc_us, period_pclks);
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
	}
	else if (type == VcselPeriodFinalRange)
	{
		// Set phase check limits for final-range VCSEL period
		switch (period_pclks)
		{
			case 8:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 10:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 12:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 14:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			default:
				return false;  // Invalid VCSEL period for final-range
		}

		// Apply new VCSEL period for final-range
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
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
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_final_range_timeout_mclks);
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
	i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
		if (i2c_return != I2C_OK) {
			return false; // Return false if the I2C read fails
		}
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (i2c_return != I2C_OK) {
		return false;
	}
	TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_VHV);
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, SYSTEM_SEQUENCE_CONFIG, sequence_config);
	if (i2c_return != I2C_OK) {
		return false;
	}
	return true;
}


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
	  i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x44, limitMCPS * (1 << 7));
	  if (i2c_return != I2C_OK) {
	          return false; // Return false if the I2C read fails
	      }
	  return true;

}


bool TOF_get_sequence_step_enables(TOFSensor_t* TOFSENS, SequenceStepEnables *enables)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

    I2C_RETURN_CODE_t i2c_return;
    uint8_t sequence_config;

    // Read the byte from the SYSTEM_SEQUENCE_CONFIG register
    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, SYSTEM_SEQUENCE_CONFIG, &sequence_config);

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


bool TOF_get_sequence_step_timeouts(TOFSensor_t* TOFSENS, SequenceStepEnables *enables, SequenceStepTimeouts *timeouts)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t data;
	I2C_RETURN_CODE_t i2c_return;

    timeouts->pre_range_vcsel_period_pclks = TOF_get_vcsel_pulse_period(TOFSENS, VcselPeriodPreRange);

    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, MSRC_CONFIG_TIMEOUT_MACROP, &data);
    timeouts->msrc_dss_tcc_mclks = data;

	if (i2c_return != I2C_OK)
	{
		return false;
	}
    timeouts->msrc_dss_tcc_mclks += 1;
    timeouts->msrc_dss_tcc_us = timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data);
    timeouts->pre_range_mclks = data;
    if (i2c_return != I2C_OK)
    	{
    		return false;
    	}
    timeouts->pre_range_mclks = decode_timeout(timeouts->pre_range_mclks);
    timeouts->pre_range_us = timeout_mclks_to_microseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = TOF_get_vcsel_pulse_period(TOFSENS, VcselPeriodFinalRange);


    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data);
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
        i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, final_range_timeout_mclks);
        if (i2c_return != I2C_OK){
            		return false;
            	}
        // Store the timing budget for internal reuse
        //uint32_t measurement_timing_budget_us = budget_us;
    }

    return true;
}


uint8_t TOF_get_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

    uint8_t vcsel_period = 255;  // Default to 255 (error case)

    if (type == VcselPeriodPreRange)
    {
        // Read the pre-range VCSEL period register and decode
        i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period);
    	if (i2c_return != I2C_OK){
    		return false;
    	}
        vcsel_period = decodeVcselPeriod(vcsel_period);
    }
    else if (type == VcselPeriodFinalRange)
    {
        // Read the final range VCSEL period register and decode
        i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period);
    	if (i2c_return != I2C_OK){
    		return false;
    	}
    	vcsel_period = decodeVcselPeriod(vcsel_period);
    }

    return vcsel_period;
}



//--------------------- ADDITIONAL NON TOF FUNCTIONS ---------------------


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


uint32_t timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert the timeout period in MCLKs to microseconds

    return (uint32_t)((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}


uint32_t timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert timeout from microseconds to MCLKs
    // The formula uses rounding by adding (macro_period_ns / 2) before dividing
    uint32_t return_value = (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    return return_value;
}
