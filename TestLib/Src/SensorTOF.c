/**
 ******************************************************************************
 * @file           : SensorTOF.c
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
 * This code and the pregister addresses are based on the official ai which can be found here:
 * https://www.st.com/en/embedded-software/stsw-img005.html#get-software
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

// variables for i2c and TOF address
TOF_ADDR_t TOF_address_used = TOF_ADDR_NONE;
I2C_TypeDef *TOF_i2c;

// TOF stop variable for register store value
static uint8_t TOF_stop_variable = 0;

// flag for continuous_mode on or off
bool TOF_continuous_mode = false;

//---------------------INTERNAL FUNCTIONS---------------------

/*
 * @function:	 TOF_configure_interrupt
 *
 * @brief: 		 configure interrupt
 *
 * @returns:	 bool: true if successful
 */
bool TOF_configure_interrupt(void)
{
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

/*
 * @function:	 TOF_init_address
 *
 * @brief: 		 init address
 *
 * @returns:	 bool: true if successful
 */
bool TOF_init_address()
{
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

/*
 * @function:	 TOF_data_init
 *
 * @brief: 		 data init
 *
 * @returns:	 bool: true if successful
 */
bool TOF_data_init()
{
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
bool TOF_get_spad_info_from_nvm(uint8_t * count, bool * type_is_aperture)
{
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

/*
 * @function:	 TOF_set_spads_from_nvmvoid
 *
 * @brief: 		 set spads from nvm
 *
 * @returns:	 bool: true if successful
 */
bool TOF_set_spads_from_nvm(void)
{
	uint8_t spad_count;
	bool spad_type_is_aperture;
	if (!TOF_get_spad_info_from_nvm(&spad_count, &spad_type_is_aperture))
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

/*
 * @function:	 TOF_load_default_tuning_settings
 *
 * @brief: 		 Load tuning settings (same as default tuning settings provided by ST api code
 *
 * @returns:	 bool: true if successful
 */
bool TOF_load_default_tuning_settings(void)
{
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

/*
 * @function:	 TOF_set_sequence_steps_enabled
 *
 * @brief: 		 Enable (or disable) specific steps in the sequence
 *
 * @returns:	 bool: true if successful
 */
bool TOF_set_sequence_steps_enabled(uint8_t sequence_step)
{
	bool result = false;

	I2C_RETURN_CODE_t success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);

	if(success == I2C_OK)
	{
		result = true;
	}

	return result;
}

/*
 * @function:	 TOF_perform_single_ref_calibration
 *
 * @brief: 		 perform calibration
 *
 * @parameters:	 STOF_calibration_type_t calib_type:	calibration type
 *
 * @returns:	 bool: true if successful
 */
bool TOF_perform_single_ref_calibration(TOF_calibration_type_t calib_type)
{
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

/*
 * @function:	 TOF_perform_ref_calibration
 *
 * @brief: 		 perform reference calibration
 *
 * @returns:	 bool: true if successful
 */
bool TOF_perform_ref_calibration()
{
	if (!TOF_perform_single_ref_calibration(TOF_CALIBRATION_TYPE_VHV)) {
		return false;
	}
	if (!TOF_perform_single_ref_calibration(TOF_CALIBRATION_TYPE_PHASE)) {
		return false;
	}
	/* Restore sequence steps enabled */
	if (!TOF_set_sequence_steps_enabled(TOF_RANGE_SEQUENCE_STEP_DSS +
			TOF_RANGE_SEQUENCE_STEP_PRE_RANGE +
			TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}
	return true;
}

/*
 * @function:	 TOF_init_device
 *
 * @brief: 		 do different device init
 *
 * @returns:	 bool: true if successful
 */
bool TOF_init_device()
{
	if (!TOF_data_init())
	{
		return false;
	}

	if (!TOF_set_spads_from_nvm()) {
		return false;
	}

	if (!TOF_load_default_tuning_settings())
	{
		return false;
	}

	if (!TOF_configure_interrupt())
	{
		return false;
	}

	if (!TOF_set_sequence_steps_enabled(TOF_RANGE_SEQUENCE_STEP_DSS +
			TOF_RANGE_SEQUENCE_STEP_PRE_RANGE +
			TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}

    if (!TOF_perform_ref_calibration()) {
        return false;
    }

	return true;
}

/*
 * @function:	 TOF_getMeasurement
 *
 * @brief: 		 get the measured distance
 *
 * @parameters:	 uint16_t *range:		measured range
 *
 * @returns:	 bool: true if successful
 */
bool TOF_getMeasurement(uint16_t *range)
{
	I2C_RETURN_CODE_t i2c_return;

	uint8_t interrupt_status[1];
	do
	{
		i2c_return = i2cBurstRegRead(
				TOF_i2c, TOF_address_used,
				TOF_REG_RESULT_INTERRUPT_STATUS,
				interrupt_status, 1);
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
bool TOF_init(I2C_TypeDef *i2c, TOF_ADDR_t addr)
{
	TOF_address_used = addr;
	TOF_i2c = i2c;

	// Init i2c address and check connectivity
	if (!TOF_init_address())
	{
		return false;
	}

	//device initialization
	if (!TOF_init_device())
	{
		return false;
	}

	// return true, if everything was fine
	return true;
}

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
bool TOF_startContinuous(uint32_t period_ms)
{
	I2C_RETURN_CODE_t i2c_return;

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

/*
 * @function:	 TOF_stopContinuous
 *
 * @brief: 		 stops continuous measurment
 *
 * @returns:	 bool: true if successful
 */
bool TOF_stopContinuous()
{
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

/*
 * @function:	 TOF_ReadContinuousDistance
 *
 * @brief: 		 get distance in continuous mode
 *
 * @parameters:	 uint16_t *range:	variable with measurement
 *
 * @returns:	 bool: true if successful
 */
bool TOF_ReadContinuousDistance(uint16_t *range)
{
	if(!TOF_continuous_mode)
	{
		return false;
	}

	if(!TOF_getMeasurement(range))
	{
		return false;
	}

	return true;
}

/*
 * @function:	 TOF_ReadSingleDistance
 *
 * @brief: 		 get distance in single mode
 *
 * @parameters:	 uint16_t *range:	variable with measurement
 *
 * @returns:	 bool: true if successful
 */
bool TOF_ReadSingleDistance(uint16_t *range)
{
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


	i2c_return = i2cSendByteToSlaveReg(
			TOF_i2c, TOF_address_used,
			TOF_REG_SYSRANGE_START, 0x01);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}

	uint8_t sysrange_start[1];
	do
	{
		i2c_return = i2cBurstRegRead(
				TOF_i2c, TOF_address_used,
				TOF_REG_SYSRANGE_START,
				sysrange_start, 1);
	} while (i2c_return == I2C_OK && (sysrange_start[0] & 0x01));
	if (i2c_return != I2C_OK)
	{
		return false;
	}

	TOF_getMeasurement(range);

	return true;
}







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
bool TOF_SetAddress(uint8_t new_Addr) {
    I2C_RETURN_CODE_t i2c_return;
    uint8_t newaddr = new_Addr;

    // Send the new address to the device
    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x8A, newaddr & 0x7F);

    if (i2c_return != I2C_OK) {
        return false; // Return false if the operation fails
    }
    TOF_address_used = newaddr;
    return true; // Ensure the function always returns a value
}

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
bool TOF_ReadDistanceTimed( uint16_t time, uint16_t *range)
{

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

	TOF_getMeasurement(range);

	return true;
}

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
bool SetRangingProfile(uint16_t Ranging_Profiles_t) {
    // Switch case for RangingProfile
    switch (Ranging_Profiles_t) {
    case DEFAULT_MODE_D:
    	if(!setMeasurementTimingBudget(30)){return false;}

        break;

    case HIGH_SPEED_MODE_S:
        setMeasurementTimingBudget(20000);
        break;

    case HIGH_ACCURACY_MODE_A:
        setMeasurementTimingBudget(200000);
        break;

    case LONG_RANGE_MODE_R:
    	//if(!setSignalRateLimit(0.1)){return false;}
    	if(!setVcselPulsePeriod(VcselPeriodPreRange, 18)){return false;}
        if(!setVcselPulsePeriod(VcselPeriodFinalRange, 14)){return false;}
        break;

    default:
        // Handle an invalid profile case
        return false;
    }

    return true; // Return true for valid profiles
}

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
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
	{
	    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
	    I2C_RETURN_CODE_t i2c_return;

	    SequenceStepEnables enables;
	    SequenceStepTimeouts timeouts;

	    // Get the current sequence step enables and timeouts from the sensor
	    getSequenceStepEnables(&enables);
	    getSequenceStepTimeouts(&enables, &timeouts);

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
	        uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
	        new_pre_range_timeout_mclks = encodeTimeOut(new_pre_range_timeout_mclks);
	        i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_pre_range_timeout_mclks);

	        // Update MSRC timeout
	        uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
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
	        uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
	        if (enables.pre_range)
	        {
	            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
	        }
	        new_final_range_timeout_mclks = encodeTimeOut(new_final_range_timeout_mclks);
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
	    TOF_perform_single_ref_calibration(TOF_CALIBRATION_TYPE_VHV);
	    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, SYSTEM_SEQUENCE_CONFIG, sequence_config);
	    if (i2c_return != I2C_OK) {
			return false;
		}
	    return true;
}

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
uint16_t encodeTimeOut(uint16_t final_range_timeout_mclks) {

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
uint16_t decodeTimeout(uint16_t reg_val)
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


/*
 * @function:	 setSignalRateLimit
 *
 * @brief: 		 get distance in single mode with preset time delay
 *
 * @parameters:	 uint16_t *range :	variable with measurement
 * 				 uint16_t time :	variable with time preset
 *
 * @returns:	 bool: true if successful
 */
bool setSignalRateLimit(float *signalRateLimit) {
	I2C_RETURN_CODE_t i2c_return;

	float limitMCPS = *signalRateLimit;
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
bool getSequenceStepEnables(SequenceStepEnables *enables)
{
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
bool getSequenceStepTimeouts(SequenceStepEnables *enables, SequenceStepTimeouts *timeouts)
{
	uint8_t data;
	I2C_RETURN_CODE_t i2c_return;

    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, MSRC_CONFIG_TIMEOUT_MACROP, &data);
    timeouts->msrc_dss_tcc_mclks = data;

	if (i2c_return != I2C_OK)
	{
		return false;
	}
    timeouts->msrc_dss_tcc_mclks += 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data);
    timeouts->pre_range_mclks = data;
    if (i2c_return != I2C_OK)
    	{
    		return false;
    	}
    timeouts->pre_range_mclks = decodeTimeout(timeouts->pre_range_mclks);
    timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);


    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data);
    timeouts->pre_range_mclks = data;
    if (i2c_return != I2C_OK)
    	{
    		return false;
    	}
    timeouts->final_range_mclks = decodeTimeout(timeouts->final_range_mclks);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);

    return true;
}


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
#define calcMacroPeriod(vcsel_period_pclks) (((uint32_t)(2304) * (vcsel_period_pclks) * 1655 + 500) / 1000)
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert the timeout period in MCLKs to microseconds

    return (uint32_t)((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}


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
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
uint8_t getVcselPulsePeriod(vcselPeriodType type)
{

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
bool setMeasurementTimingBudget(uint32_t budget_us)
{
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
    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

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
        timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        // Write the final range timeout to the register

        final_range_timeout_mclks = encodeTimeOut(final_range_timeout_mclks);
        i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, final_range_timeout_mclks);
        if (i2c_return != I2C_OK){
            		return false;
            	}
        // Store the timing budget for internal reuse
        //uint32_t measurement_timing_budget_us = budget_us;
    }

    return true;
}


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
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert timeout from microseconds to MCLKs
    // The formula uses rounding by adding (macro_period_ns / 2) before dividing
    uint32_t return_value = (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    return return_value;
}







