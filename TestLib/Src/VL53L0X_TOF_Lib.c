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
 *
 * DOKU: Seite 24 VL5310X.pdf
*/


bool SetRangingProfile(uint16_t Ranging_Profiles_t){

	// switch case for RangingProfile
	switch(Ranging_Profiles_t)
	{
	case DEFAULT_MODE_D:
		if(!TOF_set_timing_budget(30000)){
			return false;
		}
			break;


	case HIGH_SPEED_MODE_S:
		if(!TOF_set_timing_budget(20000)){
					return false;
				}
			break;


	case HIGH_ACCURACY_MODE_A:
		if(!TOF_set_timing_budget(200000)){
					return false;
				}
			break;


	case LONG_RANGE_MODE_R:
		if(!setSignalRateLimit(0.1)){
			return false;
		}

		if(!setVcselPulsePeriod(VcselPeriodPreRange, 18)){
					return false;
				}

		if(!setVcselPulsePeriod(VcselPeriodFinalRange, 14)){
					return false;
				}
			break;

	}
}


bool TOF_set_timing_budget(uint16_t timing_budget_us) {

	uint16_t StartOverhead     = 1910;
	uint16_t EndOverhead        = 960;
	uint16_t MsrcOverhead       = 660;
	uint16_t TccOverhead        = 590;
	uint16_t DssOverhead        = 690;
	uint16_t PreRangeOverhead   = 660;
	uint16_t FinalRangeOverhead = 550;

	uint32_t used_budget_us = StartOverhead + EndOverhead;

	getSequenceStepEnables(&enables);
	getSequenceStepTimeouts(&enables, &timeouts);

	if (enables.tcc){
	used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss){
	used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}

	else if (enables.msrc){
	used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range){
	used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range){
	used_budget_us += FinalRangeOverhead;

	if (used_budget_us > budget_us){
	  // "Requested timeout too big."
	  return false;
	}

	uint32_t final_range_timeout_us = budget_us - used_budget_us;

	uint32_t final_range_timeout_mclks =
	  timeoutMicrosecondsToMclks(final_range_timeout_us,
								 timeouts.final_range_vcsel_period_pclks);

	if (enables.pre_range)
	{
	  final_range_timeout_mclks += timeouts.pre_range_mclks;
	}

	i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_TimingBudget, encodeTimeout(final_range_timeout_mclks)); // so muss die ANsprache des Sensors über I2C aussehen

	// set_sequence_step_timeout() end

	measurement_timing_budget_us = budget_us; // store for internal reuse
	}
	return true;
}


bool setSignalRateLimit(float limit_Mcps) {
    // Sicherstellen, dass der Wert im gültigen Bereich liegt
    if (limit_Mcps < 0 || limit_Mcps > 511.99) {
        return false;  // Ungültiger Wert
    }

    // Q9.7 Fixed-Point Format: 9 Ganzzahlbits und 7 Fraktionsbits
    uint16_t value = (uint16_t)(limit_Mcps * (1 << 7));  // Multiplizieren mit 128 (1 << 7)
    return i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SIGNAL_RATE_LIMIT, signal_rate_limit);  // Den Wert ins Register schreiben
}


bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) {
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    // Hole die Konfiguration für die Sequenz und Timeouts
    if (!getSequenceStepEnables(&enables) || !getSequenceStepTimeouts(&enables, &timeouts)) {
        return false;
    }

    // Anwendungslogik für die VCSEL-Periode
    if (type == VcselPeriodPreRange) {
        // Setze spezifische Phase-Check-Grenzen für Pre-Range
        switch (period_pclks) {
            case 12:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x18);
                break;
            case 14:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x30);
                break;
            case 16:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x40);
                break;
            case 18:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x50);
                break;
            default:
                return false; // Ungültige Periode
        }
        writeReg(TOF_REG_VALID_PHASE_LOW, 0x08);

        // Wende die neue VCSEL-Periode an
        writeReg(TOF_REG_PreRange, vcsel_period_reg);

        // Berechne und wende die neuen Timeouts an
        uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
        writeReg16Bit(TOF_REG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

        uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
        writeReg(TOF_REG_MSRC_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
    }
    else if (type == VcselPeriodFinalRange) {
        // Setze spezifische Phase-Check-Grenzen für Final-Range
        switch (period_pclks) {
            case 8:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x10);
                writeReg(TOF_REG_VALID_PHASE_LOW,  0x08);
                break;
            case 10:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x28);
                writeReg(TOF_REG_VALID_PHASE_LOW,  0x08);
                break;
            case 12:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x38);
                writeReg(TOF_REG_VALID_PHASE_LOW,  0x08);
                break;
            case 14:
                writeReg(TOF_REG_VALID_PHASE_HIGH, 0x48);
                writeReg(TOF_REG_VALID_PHASE_LOW,  0x08);
                break;
            default:
                return false; // Ungültige Periode
        }

        // Wende die neue VCSEL-Periode für den Final-Range an
        writeReg(TOF_REG_FinalRange, vcsel_period_reg);

        // Berechne und wende die neuen Timeouts an
        uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
        if (enables.pre_range) {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }
        writeReg16Bit(TOF_REG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
    }
    else {
        return false; // Ungültiger Typ
    }

    return true;
}


bool encodeVcselPeriod(uint8_t period_pclks, uint8_t* encoded_period) {
    // Überprüfen, ob der übergebene Wert im gültigen Bereich liegt (angenommen 1-255)
    if (period_pclks < 1 || period_pclks > 255) {
        return false;  // Ungültiger Wert für period_pclks
    }

    // Umwandlung des VCSEL-Pulszeitraums in ein Registerformat
    *encoded_period = period_pclks - 1;
    return true;  // Erfolgreiche Umwandlung
}


bool getSequenceStepEnables(SequenceStepEnables *enables) {
    uint8_t sequence_config;
    if (!i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, &sequence_config)) {
        return false;
    }

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;

    return true;

	i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, &data, 1);
	uint8_t sequence_config = *data;
}


bool getSequenceStepTimeouts(SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts) {
    // Pre-Range VCSEL Period
    uint8_t pre_range_vcsel_period_raw;
    if (!TOF_read_register_8bit(PRE_RANGE_CONFIG_VCSEL_PERIOD, &pre_range_vcsel_period_raw)) {
        return false;
    }
    timeouts->pre_range_vcsel_period_pclks = decodeVcselPeriod(pre_range_vcsel_period_raw);

    // MSRC/DSS/TCC Timeout
    uint8_t msrc_config_timeout;
    if (!TOF_read_register_8bit(MSRC_CONFIG_TIMEOUT_MACROP, &msrc_config_timeout)) {
        return false;
    }
    timeouts->msrc_dss_tcc_mclks = msrc_config_timeout + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(
        timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks
    );

    // Pre-Range Timeout
    uint16_t pre_range_timeout_mclks;
    if (!TOF_read_register_16bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &pre_range_timeout_mclks)) {
        return false;
    }
    timeouts->pre_range_mclks = decodeTimeout(pre_range_timeout_mclks);
    timeouts->pre_range_us = timeoutMclksToMicroseconds(
        timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks
    );

    // Final Range VCSEL Period
    uint8_t final_range_vcsel_period_raw;
    if (!TOF_read_register_8bit(FINAL_RANGE_CONFIG_VCSEL_PERIOD, &final_range_vcsel_period_raw)) {
        return false;
    }
    timeouts->final_range_vcsel_period_pclks = decodeVcselPeriod(final_range_vcsel_period_raw);

    // Final Range Timeout
    uint16_t final_range_timeout_mclks;
    if (!TOF_read_register_16bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &final_range_timeout_mclks)) {
        return false;
    }
    timeouts->final_range_mclks = decodeTimeout(final_range_timeout_mclks);

    // Berücksichtigung von Pre-Range
    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }
    timeouts->final_range_us = timeoutMclksToMicroseconds(
        timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks
    );

    return true;
}



bool timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks, uint32_t* mclks)
{
    // Prüfen, ob der Zeitzählerwert im akzeptablen Bereich liegt
    if (timeout_period_us == 0 || vcsel_period_pclks == 0)
    {
        return false;  // Ungültige Eingabewerte
    }

    // Berechne die Makro-Periode anhand der VCSEL-Periode
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Wenn die Makro-Periode ungültig ist (0), gebe false zurück
    if (macro_period_ns == 0)
    {
        return false;
    }

    // Umrechnung von Mikrosekunden in Makro-Taktzyklen (MClks)
    *mclks = ((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns;

    return true;  // Erfolgreiche Berechnung
}


bool calcMacroPeriod(uint8_t vcsel_period_pclks, uint32_t* macro_period_ns)
{
    // Prüfen, ob die VCSEL-Periode gültig ist
    if (vcsel_period_pclks == 0)
    {
        return false;  // Ungültige Eingabe
    }

    // Beispielhafte Berechnung der Makro-Periode basierend auf der VCSEL-Periode
    *macro_period_ns = 1000 * vcsel_period_pclks;  // Beispiel: 1000 ns pro PCLK

    return true;  // Erfolgreiche Berechnung
}


bool encodeTimeout(uint32_t timeout_mclks, uint16_t* encoded_timeout)
{
    // Format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    // Wenn timeout_mclks > 0, berechne die Kodierung
    if (timeout_mclks > 0)
    {
        // Subtrahiere 1 von timeout_mclks, um auf das richtige Format zu kommen
        ls_byte = timeout_mclks - 1;

        // Bestimme die Anzahl der Verschiebungen, um die Zahl im erlaubten Bereich zu halten
        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        // Kombiniere das Ergebnis und gebe es über den Pointer zurück
        *encoded_timeout = (ms_byte << 8) | (ls_byte & 0xFF);

        return true;  // Erfolgreiche Berechnung
    }
    else
    {
        // Bei ungültigem Timeout-Wert (0), setze den codierten Timeout-Wert auf 0 und gib false zurück
        *encoded_timeout = 0;
        return false;  // Fehler: Timeout-Wert gleich 0
    }
}

