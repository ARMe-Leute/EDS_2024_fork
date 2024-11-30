/*
 * MPU6050.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Müller, Berenspöhler
 */
#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <mcalSysTick.h>
#include <math.h>

#include <MPU6050.h>

//--------------------------- Function Declarations ---------------------------

/**
 * @function MPU_init
 *
 * @brief Initializes the MPU6050 sensor with specified configurations.
 *
 * This function configures the MPU6050 sensor by setting its I2C address, gyroscope scale, accelerometer range,
 * low-pass filter settings, and power management options. Additionally, it handles optional sensor restart and
 * measurement enable/disable states.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure to hold the sensor's configuration and state.
 * @param i2cBus       Pointer to the I2C bus (e.g., `I2C_TypeDef*`) used to communicate with the sensor.
 * @param i2cAddr      I2C address of the sensor. If not set to `0x68`, the function defaults to `0x68` as the standard address.
 * @param gyroScale    Desired gyroscope full-scale range. Accepts predefined values:
 *                     - `MPU6050_GYRO_FSCALE_250` (±250°/s)
 *                     - `MPU6050_GYRO_FSCALE_500` (±500°/s)
 *                     - `MPU6050_GYRO_FSCALE_1000` (±1000°/s)
 *                     - `MPU6050_GYRO_FSCALE_2000` (±2000°/s)
 *                     - `DISABLE` to disable gyroscope measurements.
 * @param accelRange   Desired accelerometer range. Accepts predefined values:
 *                     - `MPU6050_ACCEL_RANGE_2` (±2g)
 *                     - `MPU6050_ACCEL_RANGE_4` (±4g)
 *                     - `MPU6050_ACCEL_RANGE_8` (±8g)
 *                     - `MPU6050_ACCEL_RANGE_16` (±16g)
 *                     - `DISABLE` to disable accelerometer measurements.
 * @param lPconfig     Low-pass filter configuration. Accepts predefined values corresponding to filter cutoff frequencies.
 *                     - `MPU6050_LPBW_260`, `MPU6050_LPBW_44`, etc.
 * @param restart      A non-zero value triggers a software reset of the sensor before initialization.
 *
 * @return int8_t
 * - Returns `0` on successful initialization.
 * - Returns a negative value if the initialization process is incomplete or a step fails.
 *
 * @details
 * 1. Configures the I2C bus and verifies the I2C address. If the address is not `0x68`, the function defaults it to `0x68`.
 * 2. Sets the gyroscope scale, accelerometer range, and low-pass filter configuration using the provided parameters.
 * 3. Manages sensor power states based on the `gyroScale` and `accelRange` settings:
 *    - Disables unused sensors if their configurations are set to `DISABLE`.
 *    - Configures power management registers to enable required sensors.
 * 4. Supports optional sensor restart by triggering a software reset and reinitializing the configuration registers.
 * 5. Handles the initialization process in discrete steps (`step` variable) to ensure sequential and stable setup.
 *
 * @note
 * - The function expects the sensor instance (`MPU6050_t`) to be properly allocated before use.
 * - Ensure the I2C bus is initialized before calling this function.
 * - Refer to the MPU6050 datasheet for valid configuration values and register settings.
 * - The `step` mechanism allows reentry into the initialization process for debugging or staged setup.
 *
 * @warning
 * - Incorrect configuration values may lead to undefined sensor behavior or inaccurate measurements.
 * - Modifications to the I2C address may require hardware changes (e.g., soldering the AD0 pin).
 */
int8_t MPU_init(MPU6050_t* sensor, I2C_TypeDef* i2cBus, uint8_t i2cAddr, uint8_t gyroScale, uint8_t accelRange, uint8_t lPconfig, uint8_t restart) {

	sensor->i2c = i2cBus;

	if(i2cAddr == i2cAddr_MPU6050) {
		sensor->i2cAddress = i2cAddr;
	}
	else {
		/**
		 * To change I2C Address of the MPU6050, the AD0-Pin of the sensor must be set high
		 * This pin is not connected to the board
		 * therefore, the standard address is always used
		 * ToDo: Abklären, ob Addressse setzen nötig, Sensor entsprechend umlöten
		 */
		sensor->i2cAddress = (uint8_t) i2cAddr_MPU6050;
	}

	switch (gyroScale) {
	case MPU6050_GYRO_FSCALE_250:
		sensor->GyroScale = (uint8_t) MPU6050_GYRO_FSCALE_250;
		break;
	case MPU6050_GYRO_FSCALE_500:
		sensor->GyroScale = (uint8_t) MPU6050_GYRO_FSCALE_500;
		break;
	case MPU6050_GYRO_FSCALE_1000:
		sensor->GyroScale = (uint8_t) MPU6050_GYRO_FSCALE_1000;
		break;
	case MPU6050_GYRO_FSCALE_2000:
		sensor->GyroScale = (uint8_t) MPU6050_GYRO_FSCALE_2000;
		break;
	case 0xff:
		sensor->GyroScale = (uint8_t) DISABLE;
		break;
	default:
		sensor->GyroScale = (uint8_t) MPU6050_GYRO_FSCALE_250;
		break;
	}

	switch (accelRange) {
	case MPU6050_ACCEL_RANGE_2:
		sensor->AccelRange = (uint8_t) MPU6050_ACCEL_RANGE_2;
		break;
	case MPU6050_ACCEL_RANGE_4:
		sensor->AccelRange = (uint8_t) MPU6050_ACCEL_RANGE_4;
		break;
	case MPU6050_ACCEL_RANGE_8:
		sensor->AccelRange = (uint8_t) MPU6050_ACCEL_RANGE_8;
		break;
	case MPU6050_ACCEL_RANGE_16:
		sensor->AccelRange = (uint8_t) MPU6050_ACCEL_RANGE_16;
		break;
	case DISABLE:
		sensor->AccelRange = (uint8_t) DISABLE;
	default:
		sensor->AccelRange = (uint8_t) MPU6050_ACCEL_RANGE_16;
		break;
	}

	switch (lPconfig) {
	case MPU6050_LPBW_260:
		sensor->LPFiltConfig = (uint8_t) MPU6050_LPBW_260;
		break;
	case MPU6050_LPBW_184:
		sensor->LPFiltConfig = (uint8_t) MPU6050_LPBW_184;
		break;
	case MPU6050_LPBW_94:
		sensor->LPFiltConfig = (uint8_t) MPU6050_LPBW_94;
		break;
	case MPU6050_LPBW_44:
		sensor->LPFiltConfig = (uint8_t) MPU6050_LPBW_44;
		break;
	case MPU6050_LPBW_21:
		sensor->LPFiltConfig = (uint8_t) MPU6050_LPBW_21;
		break;
	case MPU6050_LPBW_5:
		sensor->LPFiltConfig = (uint8_t) MPU6050_LPBW_5;
		break;
	default:
		sensor->LPFiltConfig = (uint8_t) MPU6050_LPBW_260;
		break;
	}

	static int8_t step = -5;
	if ((restart != 0) && (step == -5)) {
		step = step-1;
	}

	for (int8_t i = step; i < 0; i++) {

		switch (step) {
		case -6:		// CLK Speed von I2C auf 400kHz
			i2cSetClkSpd(sensor->i2c, I2C_CLOCK_1Mz); //set I2C Clock 1000kHz
			step = -5;
			break;

		case -5:		// SW Reset
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_SWRESET)); // reboot memory content
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_MST_CTRL, (MPU6050_MST_P_NSR)); // tell sensor to expect stop bit
			step = -4;
			break;

		case -4: // ToDo: Bitmasken für Sleep-Modi überprüfen, vorher Flämig fragen
			// PWR Mngt
			/*
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_2, (MPU6050_PWR2_ACConXY_GYonZ)); //ToDo: Fragen, wo der Sinn liegt?
			 */
			if (sensor->AccelRange == (uint8_t) DISABLE) { // Disable acceleration measurement
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_2, (0b00111000));
			}
			else {
				if (sensor->GyroScale == (uint8_t) DISABLE) { // Disable gyroscope
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_2, (0b00000111));
				}
				else { // enable all measurements
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_2, (0b00000000));
				}
			}
			step = -3;
			break;

		case -3:
			// GYRO Config
			if (sensor->GyroScale == DISABLE) {
				step=-2;
			}
			else {
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_GYRO_CONFIG, sensor->GyroScale); 	// set scale range of gyroscope
				step = -2;
			}
			break;

		case -2:
			// ACCEL Config
			if (sensor->AccelRange == DISABLE) {
				step = -1;
			}
			else {
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_ACCEL_CONFIG, sensor->AccelRange); 	// set scale range of accelerometer
				step = -1;
			}
			break;

		case -1:	// LowPass Config
			MPU_init_lowpass_filter(sensor);
			step = 0;
			break;

		default:
			step = -5;
		}
	}

	return step;
}

/**
 * @function MPU_get_acceleration
 *
 * @brief Reads the accelerometer data from the MPU6050 sensor, if enabled.
 *
 * This function retrieves acceleration values along the X, Y, and Z axes from the MPU6050 sensor, converts them to
 * physical units in terms of g (gravitational acceleration), and stores them in the `AccelXYZ` array of the provided
 * sensor instance. If the accelerometer is disabled, the function skips the I2C read and returns an error code.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure containing the sensor configuration and state.
 *
 * @return int16_t
 * - Returns the result of the I2C transaction (`I2C_RETURN_CODE_t`) if the accelerometer is enabled.
 *   A value of `I2C_SUCCESS` (0) indicates successful data retrieval.
 * - Returns `1` if the accelerometer is disabled (`AccelRange` set to `DISABLE`).
 *
 * @details
 * 1. Checks if the accelerometer is enabled by verifying the `AccelRange` parameter in the sensor instance.
 *    If disabled, the function immediately returns `1`.
 * 2. Reads 6 bytes of acceleration data from the MPU6050 registers via I2C communication.
 * 3. Combines the bytes to form 16-bit signed values representing the acceleration for each axis (X, Y, Z).
 * 4. Converts the raw acceleration data to physical acceleration in units of g using the scaling factors:
 *    - `MPU6050_ACCEL_RANGE_2`: ±2g, scaling factor = 16384 LSB/g
 *    - `MPU6050_ACCEL_RANGE_4`: ±4g, scaling factor = 8192 LSB/g
 *    - `MPU6050_ACCEL_RANGE_8`: ±8g, scaling factor = 4096 LSB/g
 *    - `MPU6050_ACCEL_RANGE_16`: ±16g, scaling factor = 2048 LSB/g
 * 5. The converted values are stored in the `AccelXYZ` array of the `sensor` instance.
 *
 * @note
 * - Ensure the MPU6050 sensor has been properly initialized using `initMPU` before calling this function.
 * - This function assumes the accelerometer is properly configured during initialization.
 */
int16_t MPU_get_acceleration(MPU6050_t* sensor) {
	if (sensor->AccelRange != (uint8_t) DISABLE) {
		I2C_RETURN_CODE_t i2c_return;
		uint8_t readBuffer[6];
		int16_t X, Y, Z;

		i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2cAddress, MPU6050_AccXYZ, readBuffer, 6);
		X = ((readBuffer[0]<<8) | readBuffer[1]);
		Y = ((readBuffer[2]<<8) | readBuffer[3]);
		Z = ((readBuffer[4]<<8) | readBuffer[5]);

		switch(sensor->AccelRange) {
		case MPU6050_ACCEL_RANGE_2:		// Faktor 16384 LSB/g
			sensor->AccelXYZ[0] = (float) X / 16384;
			sensor->AccelXYZ[1] = (float) Y / 16384;
			sensor->AccelXYZ[2] = (float) Z / 16384;
			break;
		case MPU6050_ACCEL_RANGE_4:		// Faktor 8192 LSB/g
			sensor->AccelXYZ[0] = (float) X / 8192;
			sensor->AccelXYZ[1] = (float) Y / 8192;
			sensor->AccelXYZ[2] = (float) Z / 8192;
			break;
		case MPU6050_ACCEL_RANGE_8:		// Faktor 4096 LSB/g
			sensor->AccelXYZ[0] = (float) X / 4096;
			sensor->AccelXYZ[1] = (float) Y / 4096;
			sensor->AccelXYZ[2] = (float) Z / 4096;
			break;
		case MPU6050_ACCEL_RANGE_16:	// Faktor 2048 LSB/g
			sensor->AccelXYZ[0] = (float) X / 2048;
			sensor->AccelXYZ[1] = (float) Y / 2048;
			sensor->AccelXYZ[2] = (float) Z / 2048;
			break;
		}
		return (int16_t) i2c_return;
	}
	else {
		return (int16_t) 1;
	}
}

/**
 * @function MPU_get_angle_from_acceleration
 *
 * @brief Computes the tilt angles (alpha and beta) of the MPU6050 sensor based on acceleration data.
 *
 * This function calculates the tilt angles of the MPU6050 sensor in radians using acceleration data
 * retrieved from the X, Y, and Z axes. The calculated angles are stored in the `AlphaBeta` array of
 * the provided sensor instance.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure, which contains the sensor's
 *                     configuration and current state.
 *
 * @return int16_t     Returns the result of the `MPU_get_acceleration` function, indicating the status
 *                     of the I2C transaction. Typically:
 *                     - 0 (`I2C_SUCCESS`) indicates success.
 *                     - Negative values indicate an error.
 *
 * @details
 * 1. Calls the `MPU_get_acceleration` function to update the sensor's acceleration data.
 * 2. Extracts the X, Y, and Z-axis acceleration values from the `AccelXYZ` array of the sensor instance.
 * 3. Computes tilt angles in radians:
 *    - `AlphaBeta[0]`: Tilt angle in the X-Z plane (alpha) using the formula `atan2(X, Z)`.
 *    - `AlphaBeta[1]`: Tilt angle in the Y-Z plane (beta) using the formula `atan2(Y, Z)`.
 * 4. The calculated angles are stored in the `AlphaBeta` array for further use.
 */
int16_t MPU_get_angle_from_acceleration(MPU6050_t* sensor) {
	int16_t returnValue = MPU_get_acceleration(sensor);
	float X = (float) sensor->AccelXYZ[0];
	float Y = (float) sensor->AccelXYZ[1];
	float Z = (float) sensor->AccelXYZ[2];

	sensor->AlphaBeta[0] = atan2(X, Z);
	sensor->AlphaBeta[1] = atan2(Y, Z);

	return returnValue;
}

/**
 * @function MPU_get_gyro
 *
 * @brief Reads the gyroscope data from the MPU6050 sensor, if enabled.
 *
 * This function retrieves the rotational velocity values for the X, Y, and Z axes from the MPU6050 sensor,
 * converts them to physical values in degrees per second (°/s), and stores them in the `GyroXYZ` array
 * of the provided sensor instance. If the gyroscope is disabled, the function skips the I2C read and
 * returns an error code.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure containing the sensor configuration and state.
 *
 * @return int16_t
 * - Returns the result of the I2C transaction (`I2C_RETURN_CODE_t`) if the gyroscope is enabled.
 *   A value of `I2C_SUCCESS` (0) indicates successful data retrieval.
 * - Returns `1` if the gyroscope is disabled (`GyroScale` set to `DISABLE`).
 *
 * @details
 * 1. Checks if the gyroscope is enabled by verifying the `GyroScale` parameter in the sensor instance.
 *    If disabled, the function immediately returns `1`.
 * 2. Reads 6 bytes of gyroscope data from the MPU6050 registers via I2C communication.
 * 3. Combines the bytes to form 16-bit signed values representing the rotational velocity for each axis (X, Y, Z).
 * 4. Converts the raw gyroscope data to physical rotational velocity in °/s using the scaling factors:
 *    - `MPU6050_GYRO_FSCALE_250`: ±250°/s, scaling factor = 131 LSB/°/s
 *    - `MPU6050_GYRO_FSCALE_500`: ±500°/s, scaling factor = 65.5 LSB/°/s
 *    - `MPU6050_GYRO_FSCALE_1000`: ±1000°/s, scaling factor = 32.8 LSB/°/s
 *    - `MPU6050_GYRO_FSCALE_2000`: ±2000°/s, scaling factor = 16.4 LSB/°/s
 * 5. The converted values are stored in the `GyroXYZ` array of the `sensor` instance.
 *
 * @note
 * - Ensure the MPU6050 sensor has been properly initialized using `initMPU` before calling this function.
 * - This function assumes the gyroscope is properly configured during initialization.
 */
int16_t MPU_get_gyro(MPU6050_t* sensor) {
	if (sensor->GyroScale != (uint8_t) DISABLE) {
		uint8_t readBuffer[6];
		int16_t X, Y, Z;
		I2C_RETURN_CODE_t i2c_return;
		i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2cAddress, MPU6050_GyroXYZ, readBuffer, 6);
		X = ((readBuffer[0]<<8) | readBuffer[1]);
		Y = ((readBuffer[2]<<8) | readBuffer[3]);
		Z = ((readBuffer[4]<<8) | readBuffer[5]);

		switch (sensor->GyroScale) {
		case MPU6050_GYRO_FSCALE_250:		// Factor 131 LSB/°/s
			sensor->GyroXYZ[0] = (float) X / 131;
			sensor->GyroXYZ[1] = (float) Y / 131;
			sensor->GyroXYZ[2] = (float) Z / 131;
			break;
		case MPU6050_GYRO_FSCALE_500:		// Factor 65.5 LSB/°/s
			sensor->GyroXYZ[0] = (float) X / 65.5;
			sensor->GyroXYZ[1] = (float) Y / 65.5;
			sensor->GyroXYZ[2] = (float) Z / 65.5;
			break;
		case MPU6050_GYRO_FSCALE_1000:		// Factor 32.8 LSB/°/s
			sensor->GyroXYZ[0] = (float) X / 32.8;
			sensor->GyroXYZ[1] = (float) Y / 32.8;
			sensor->GyroXYZ[2] = (float) Z / 32.8;
			break;
		case MPU6050_GYRO_FSCALE_2000:		// Factor 16.4 LSB/°/s
			sensor->GyroXYZ[0] = (float) X / 16.4;
			sensor->GyroXYZ[1] = (float) Y / 16.4;
			sensor->GyroXYZ[2] = (float) Z / 16.4;
			break;
		}
		return (int16_t) i2c_return;
	}
	else {
		return (uint16_t) 1;
	}
}

/**
 * @function MPU_get_temperature
 *
 * @brief Reads the temperature data from the MPU6050 sensor.
 *
 * This function retrieves the raw temperature data from the MPU6050 sensor, converts it to degrees Celsius,
 * and stores the result in the `TempOut` field of the provided sensor instance.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure containing the sensor configuration and state.
 *
 * @return int16_t     Returns the result of the I2C transaction (`I2C_RETURN_CODE_t`).
 *                     A value of `I2C_SUCCESS` (0) indicates successful data retrieval.
 *
 * @details
 * 1. Reads 2 bytes of temperature data from the MPU6050's temperature register via I2C communication.
 * 2. Combines the bytes to form a 16-bit signed value representing the raw temperature.
 * 3. Converts the raw temperature data to degrees Celsius using the formula:
 *    \[
 *    \text{Temperature} = \frac{\text{RawTemp}}{340} + 36.35
 *    \]
 * 4. Stores the converted temperature value in the `TempOut` field of the `sensor` instance.
 *
 * @note
 * - Ensure the MPU6050 sensor has been properly initialized using `MPU_init` before calling this function.
 * - The temperature measurement reflects the internal sensor temperature, which may not correspond to the ambient temperature.
 */
int16_t MPU_get_temperature(MPU6050_t* sensor) {
	uint8_t readBuffer[2];
	int16_t rawTemp;
	I2C_RETURN_CODE_t i2c_return;
	i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2cAddress, MPU6050_Temp, readBuffer, 2);
	rawTemp = (int16_t) (readBuffer[0]<<8) + readBuffer[1];
	sensor->TempOut = (float) rawTemp/340 + 36.35;
	return (int16_t) i2c_return;
}

/**
 * @function MPU_init_lowpass_filter
 *
 * ToDo: Abklären, ob eigene Funktion nötig, oder in Init Funktion schreiben?
 *
 * @brief Initializes the low-pass filter settings for the MPU6050 sensor.
 *
 * This function configures the MPU6050's digital low-pass filter (DLPF) by writing the specified configuration
 * value (`LPFiltConfig`) to the sensor's **CONFIG** register. The DLPF affects both the gyroscope and accelerometer
 * measurements, providing noise reduction and bandwidth control.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure containing the sensor configuration and state.
 *
 * @details
 * 1. The function sends the value of `sensor->LPFiltConfig` to the **CONFIG** register of the MPU6050 using I2C communication.
 * 2. The low-pass filter setting controls the cutoff frequency for gyroscope and accelerometer signals, as defined by the
 *    MPU6050 datasheet.
 * 3. The user must set the desired filter configuration value in the `LPFiltConfig` field of the `sensor` structure
 *    before calling this function.
 *
 * @note
 * - Ensure the MPU6050 sensor is properly initialized and powered before calling this function.
 * - Refer to the MPU6050 datasheet for valid DLPF configuration values and their corresponding cutoff frequencies.
 */
void MPU_init_lowpass_filter(MPU6050_t* sensor) {
	i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_CONFIG, sensor->LPFiltConfig);
}
