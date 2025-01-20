/*
 * MPU6050.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Müller, Berenspöhler
 *       updated: T Flaemig
 */
#include <mcalGPIO.h>
#include <mcalI2C.h>
//#include <mcalSysTick.h>
#include <math.h>

#include <i2cMPU.h>

/*
 * filterAccData
 * calc from raw data into filtered (float) with in adjusted range

*/
void filterAccData(int16_t *XYZ, float *XYZFiltert, float kFilt, float fRange)
{
	uint8_t i;
	for (i = 0; i<3; i++)
	{
		XYZFiltert[i] *= (1-kFilt);
		XYZFiltert[i] += ((float)XYZ[i] * kFilt* fRange);
	}
}


//--------------------------- Function Declarations ---------------------------

/**
 * @function mpuInit
 *
 * @brief Configures and initializes the MPU6050 sensor with specified settings.
 *
 * This function sets up the MPU6050 sensor by configuring its I2C address, gyroscope and accelerometer settings,
 * low-pass filter, and power management. It also supports optional sensor reset and staged initialization for debugging.
 *
 * @param sensor       Pointer to the `MPU6050_t` structure to store the sensor's configuration and state.
 * @param i2cBus       Pointer to the I2C bus interface (e.g., `I2C_TypeDef*`) used for communication.
 * @param i2cAddr      I2C address of the sensor. Defaults to `0x68` if not explicitly configured.
 * @param gyroScale    Gyroscope full-scale range. Valid values:
 *
 * @param accelRange   Accelerometer range. Valid values:
 *
 * @param lPconfig     Low-pass filter configuration (e.g., `MPU6050_LPBW_260`, `MPU6050_LPBW_44`).
 * @param restart      If non-zero, performs a software reset before initialization.
 *
 * @return int8_t
 * - `0` if initialization is successful.
 * - A negative value if a step fails or if initialization is incomplete.
 * - '2' if a wrong input for the gyroscope scaling is passed
 * - '4' if a wrong input for the acceleration range is passed
 *
 * @details
 * - Initializes the I2C address; defaults to `0x68` if the specified address is not supported.
 * - Configures the gyroscope and accelerometer based on user input, disabling unused components.
 * - Sets the low-pass filter configuration for noise reduction.
 * - Manages sensor power states according to the selected configurations.
 * - Supports staged initialization using a `step` mechanism for debugging and reentrant calls.
 *
 * @note
 * - Ensure the I2C bus is initialized before calling this function.
 * - The `MPU6050_t` structure must be allocated and initialized.
 * - Refer to the MPU6050 datasheet for valid register values and further configuration details.
 *
 * @warning
 * - Modifying the I2C address may require hardware adjustments (e.g., soldering the AD0 pin).
 * - Disabling both gyroscope and accelerometer may lead to an unusable sensor state.
 */

int8_t mpuInit(MPU6050_t* sensor, I2C_TypeDef* i2cBus, uint8_t i2cAddr, MPUfscale gyroScale, MPUaccel accelRange, MPUlpbw lPconfig, uint8_t restart)
{

	sensor->i2c = i2cBus;

	// default Orientation
	sensor->RPY[0] = 1;
	sensor->RPY[1] = 2;
	sensor->RPY[2] = -3;					// -3 means Sensor Y-Axis goes into top direction
	sensor->pitchZero = 0;					// assemble offset MPU vs chassis
	sensor->pitchFilt = 0.98;				// weight of Gyro for Pitch Calc in combination to Accel-Angle Calc (Static Bias)
	sensor->pitch = 0;						// assemble offset MPU vs chassis
	sensor-> swLowPassFilt =0.8;			// Accel Filter koeffizent
	sensor->accel[0] = 0;
	sensor->accel[1] = 0;
	sensor->accel[2] = 0;
	if(i2cAddr == i2cAddr_MPU6050) {
		sensor->i2c_address = i2cAddr;
	}
	else {
		/**
		 * To change I2C Address of the MPU6050, the AD0-Pin of the sensor must be set high
		 * This pin is not connected to the board
		 * therefore, the standard address is always used to prevent communication errors.
		 */
		sensor->i2c_address = (uint8_t) i2cAddr_MPU6050;
	}

	uint8_t gyroReturn;
	gyroScale = sensor->gyro_scale;
	switch (gyroScale) {
	case FSCALE_250:
		sensor->gyro_scale_factor = (float) 1 / 131;	// 131 LSB/°/s
		gyroReturn = 0;
		break;
	case FSCALE_500:
		sensor->gyro_scale_factor = (float) 1 / 65.5;	// 65.5 LSB/°/s
		gyroReturn = 0;
		break;
	case FSCALE_1000:
		sensor->gyro_scale_factor = (float)	0.0305175 ;	// 35.8 LSB/°/s
		gyroReturn = 0;
		break;
	case FSCALE_2000:
		sensor->gyro_scale_factor = (float) 1 / 16.4;	// 16.4 LSB/°/s
		gyroReturn = 0;
		break;
	default:
		sensor->gyro_scale = FSCALE_250;
		sensor->gyro_scale_factor = (float) 1 / 131;	// 131 LSB/°/s
		gyroReturn = 2;							// Error handling for wrong user input
		break;
	}
	i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_GYRO_CONFIG, sensor->gyro_scale); 	// set scale range of gyroscope


	uint8_t accelReturn;
	sensor->accel_range = accelRange;
	switch (accelRange) {
	case ACCEL_2g:
		sensor->accel_range_factor = (float) 1 / 16384;		// 16384 LSB/g
		accelReturn = 0;
		break;
	case ACCEL_4g:
		sensor->accel_range_factor = (float) 1 / 8192;		// 8192 LSB/g
		accelReturn = 0;
		break;
	case ACCEL_8g:
		sensor->accel_range_factor = (float) 1 / 4096;		// 4096 LSB/g
		accelReturn = 0;
		break;
	case ACCEL_16g:
		sensor->accel_range_factor = (float) 1 / 2048;		// 2048 LSB/g
		accelReturn = 0;
		break;
	default:
		sensor->accel_range = ACCEL_2g;
		sensor->accel_range_factor = (float) 1 / 16384;
		accelReturn = 4;
		break;
	}
	i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_ACCEL_CONFIG, sensor->accel_range);	// set scale range of accelerometer

	sensor->LowPassFilt = lPconfig;
	mpuSetLpFilt(sensor);

	static int8_t step = -3;

	//TF for (int8_t i = step; i < 0; i++) {

		switch (step) {
		case -3:		// increase CLK Speed for I2C up to  1000kHz
			i2cSetClkSpd(sensor->i2c, I2C_CLOCK_1Mz); //set I2C Clock 1Mz
			step = -2;
			break;

		case -2:		// SW Reset
			if (restart != 0)
			{
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_PWR_MGMT_1, (MPU6050_SWRESET)); // reboot memory content

			}
			else
			{
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_MST_CTRL, (MPU6050_MST_P_NSR));
			}
			step = -1;
			break;

		case -1:
			// PWR Mngt
			if (sensor->accel_range == ACCEL_OFF)
			{ // Disable acceleration measurement
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_PWR_MGMT_2, (0b00000111));
			}
			else
			{
				if (sensor->gyro_scale == GYRO_OFF)
				{ // Disable gyroscope
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_PWR_MGMT_2, (0b00111000));
				}
				else
				{ // enable all measurements
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_PWR_MGMT_2, (0b00000000));
				}
			}
			step = 0;
			break;
		default:
			step = -3;
		}
	//TF }

	int8_t returnValue = 0;
	if(step != 0) {
		returnValue = step;
		/**
		 * I2C initialization error
		 */
	}
	else {
		returnValue = gyroReturn + accelReturn;
		/**
		 * Wrong user input for gyroscope and/or acceleration measurement
		 */
	}
	return returnValue;
}

/**
 * @function mpuGetAcceleration
 *
 * @brief Reads the accelerometer data from the MPU6050 sensor, if enabled.
 *
 * This function retrieves acceleration values along the X, Y, and Z axes from the MPU6050 sensor, converts them to
 * physical units in terms of g (gravitational acceleration), and stores them in the `accel_xyz` array of the provided
 * sensor instance. If the accelerometer is disabled, the function skips the I2C read and returns an error code.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure containing the sensor configuration and state.
 *
 * @return int16_t
 * - Returns the result of the I2C transaction (`I2C_RETURN_CODE_t`) if the accelerometer is enabled.
 *   A value of `I2C_SUCCESS` (0) indicates successful data retrieval.
 * - Returns `1` if the accelerometer is disabled (`accel_range` set to `DISABLE`).
 *
 * @details
 * 1. Checks if the accelerometer is enabled by verifying the `AccelRange` parameter in the sensor instance.
 *    If disabled, the function immediately returns `1`.
 * 2. Reads 6 bytes of acceleration data from the MPU6050 registers via I2C communication.
 * 3. Combines the bytes to form 16-bit signed values representing the acceleration for each axis (X, Y, Z).
 * 4. Converts the raw acceleration data to physical acceleration in units of g using the scaling factors:
 *    - `2`: ±2g, scaling factor = 16384 LSB/g
 *    - `4`: ±4g, scaling factor = 8192 LSB/g
 *    - `8`: ±8g, scaling factor = 4096 LSB/g
 *    - `16`: ±16g, scaling factor = 2048 LSB/g
 * 5. The converted values are stored in the `accel_xyz` array of the `sensor` instance.
 *
 * @note
 * - Ensure the MPU6050 sensor has been properly initialized using `initMPU` before calling this function.
 * - This function assumes the accelerometer is properly configured during initialization.
 */
int16_t mpuGetAccel(MPU6050_t* sensor) {
	I2C_RETURN_CODE_t i2c_return;
	uint8_t readBuffer[6];
	int16_t *XYZ;
	float *XYZFiltert;

	if (sensor->accel_range != (uint8_t) DISABLE)
	{
		i2cBurstRegRead(sensor->i2c, sensor->i2c_address, MPU6050_AccXYZ,readBuffer, 6);
		sensor->accel_raw[0] = (readBuffer[0]<<8) + readBuffer[1];
		sensor->accel_raw[1] = (readBuffer[2]<<8) + readBuffer[3];
		sensor->accel_raw[2] = (readBuffer[4]<<8) + readBuffer[5];
		XYZ = sensor->accel_raw;
		XYZFiltert =sensor->accel;
		filterAccData(XYZ, XYZFiltert, sensor->swLowPassFilt, sensor->accel_range_factor);

		/*sensor->accel[0] = (float) sensor->accel_raw[0] * sensor->accel_range_factor;
		sensor->accel[1] = (float) sensor->accel_raw[1] * sensor->accel_range_factor;
		sensor->accel[2] = (float) sensor->accel_raw[2] * sensor->accel_range_factor;
		*/
	}
	else
	{
		i2c_return = 1;
	}
	return (int16_t)i2c_return;
}

/**
 * @function mpuGetAngleFromAcceleration
 *
 * @brief Computes the tilt angles (alpha and beta) of the MPU6050 sensor based on acceleration data.
 *
 * This function calculates the tilt angles of the MPU6050 sensor in radians using acceleration data
 * retrieved from the X, Y, and Z axes. The calculated angles are stored in the `alpha_beta` array of
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
 * 2. Computes tilt angles in radians:
 *    - `AlphaBeta[0]`: Tilt angle in the X-Z plane (alpha) using the formula `atan2(X, Z)`.
 *    - `AlphaBeta[1]`: Tilt angle in the Y-Z plane (beta) using the formula `atan2(Y, Z)`.
 * 3. The calculated angles are stored in the `alpha_beta` array for further use.
 */
int16_t mpuGetRPfromAccel(MPU6050_t* sensor) {
	int16_t returnValue = mpuGetAccel(sensor);
	int argYaw;
	float yawAxis;
	float longAxis = sensor->accel[sensor->RPY[0]-1];
	float latAxis = sensor->accel[sensor->RPY[1]-1];
	if (sensor->RPY[2] < 0)
	{
		argYaw = -1-(sensor->RPY[2]);
		yawAxis =  (sensor->accel[argYaw])*-1;

	}
	else
	{
		yawAxis =  sensor->accel[sensor->RPY[2]-1];
	}
//#define RP_SQRT
#ifdef RP_SQRT		// This variant reduce the calc failure at dynamic movements , first tests and measurements had no benefits
	float sqrt_yawlat = sqrt(yawAxis*yawAxis+latAxis*latAxis);
	sensor->pitch = atan2(longAxis, sqrt_yawlat);
	float sqrt_yawlon = sqrt(yawAxis*yawAxis+longAxis*longAxis);
	sensor->roll = atan2(latAxis, sqrt_yawlon);
#else
	sensor->pitch = atan2(longAxis, yawAxis)-sensor->pitchZero;
	sensor->roll = atan2(latAxis, yawAxis);
#endif

	return returnValue;
}

/**
 * @function mpuGetGyro
 *
 * @brief Reads the gyroscope data from the MPU6050 sensor, if enabled.
 *
 * This function retrieves the rotational velocity values for the X, Y, and Z axes from the MPU6050 sensor,
 * converts them to physical values in degrees per second (°/s), and stores them in the `gyro_xyz` array
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
 * 1. Checks if the gyroscope is enabled by verifying the `gyro_scale` parameter in the sensor instance.
 *    If disabled, the function immediately returns `1`.
 * 2. Reads 6 bytes of gyroscope data from the MPU6050 registers via I2C communication.
 * 3. Combines the bytes to form 16-bit signed values representing the rotational velocity for each axis (X, Y, Z).
 * 4. Converts the raw gyroscope data to physical rotational velocity in °/s using the scaling factors:
 *    - `1`: ±250°/s, scaling factor = 131 LSB/°/s
 *    - `2`: ±500°/s, scaling factor = 65.5 LSB/°/s
 *    - `3`: ±1000°/s, scaling factor = 32.8 LSB/°/s
 *    - `4`: ±2000°/s, scaling factor = 16.4 LSB/°/s
 * 5. The converted values are stored in the `gyro_xyz` array of the `sensor` instance.
 *
 * @note
 * - Ensure the MPU6050 sensor has been properly initialized using `initMPU` before calling this function.
 * - This function assumes the gyroscope is properly configured during initialization.
 */
int16_t mpuGetGyro(MPU6050_t* sensor) {
	if (sensor->gyro_scale != (uint8_t) DISABLE) {
		uint8_t readBuffer[6];

		I2C_RETURN_CODE_t i2c_return;
		i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2c_address, MPU6050_GyroXYZ, readBuffer, 6);
		sensor->gyro_raw[0] = (readBuffer[0]<<8) + readBuffer[1];
		sensor->gyro_raw[1] = (readBuffer[2]<<8) + readBuffer[3];
		sensor->gyro_raw[2] = (readBuffer[4]<<8) + readBuffer[5];

		sensor->gyro[0] = (float) sensor->gyro_raw[0] * sensor->gyro_scale_factor;
		sensor->gyro[1] = (float) sensor->gyro_raw[1] * sensor->gyro_scale_factor;
		sensor->gyro[2] = (float) sensor->gyro_raw[2] * sensor->gyro_scale_factor;
		return (int16_t) i2c_return;
	}
	else {
		return (uint16_t) 1;
	}
}

int16_t mpuGetPitch(MPU6050_t* sensor) {
	I2C_RETURN_CODE_t i2c_return;
	int16_t *XYZ;
	float *XYZFiltert;
	uint8_t readBuffer[14];
	if (sensor->accel_range != (uint8_t) DISABLE)
	{
		i2cBurstRegRead(sensor->i2c, sensor->i2c_address, MPU6050_AccXYZ,readBuffer, 14);
		sensor->accel_raw[0] = (readBuffer[0]<<8) + readBuffer[1];
		sensor->accel_raw[1] = (readBuffer[2]<<8) + readBuffer[3];
		sensor->accel_raw[2] = (readBuffer[4]<<8) + readBuffer[5];

		XYZ = sensor->accel_raw;
		XYZFiltert = sensor->accel;
		filterAccData(XYZ, XYZFiltert, sensor->swLowPassFilt, sensor->accel_range_factor);

		sensor->temp_raw = (int16_t) (readBuffer[6]<<8) + readBuffer[7];

		sensor->gyro_raw[0] = (readBuffer[8]<<8) + readBuffer[9];
		sensor->gyro_raw[1] = (readBuffer[10]<<8) + readBuffer[11];
		sensor->gyro_raw[2] = (readBuffer[12]<<8) + readBuffer[13];

		sensor->gyro[0] = (float) sensor->gyro_raw[0] * sensor->gyro_scale_factor;
		sensor->gyro[1] = (float) sensor->gyro_raw[1] * sensor->gyro_scale_factor;
		sensor->gyro[2] = (float) sensor->gyro_raw[2] * sensor->gyro_scale_factor;
	}
	else
	{
		i2c_return = 1;
	}
	int argYaw;
	float yawAxis;
	float longAxis = sensor->accel[sensor->RPY[0]-1];
	//float latAxis = sensor->accel_raw[sensor->RPY[1]-1];
	if (sensor->RPY[2] < 0)
	{
		argYaw = -1-(sensor->RPY[2]);
		yawAxis =  (sensor->accel[argYaw])*-1;

	}
	else
	{
		yawAxis =  sensor->accel[sensor->RPY[2]-1];
	}
	sensor->pitchAccel = atan2(longAxis, yawAxis)-sensor->pitchZero;

	float gyroPitch =  _deg2rad * sensor->gyro[sensor->RPY[1]-1];			// rad/s
	sensor->pitch = (sensor->pitchFilt * (sensor->pitch + gyroPitch * sensor->timebase)) + ((1- sensor->pitchFilt) * sensor->pitchAccel);


	return i2c_return;

}




/**
 * @function mpuGetTemp
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
 * 3. Converts the raw temperature data to degrees Celsius.
 * 4. Stores the converted temperature value in the `TempOut` field of the `sensor` instance.
 *
 * @note
 * - Ensure the MPU6050 sensor has been properly initialized using `MPU_init` before calling this function.
 * - The temperature measurement reflects the internal sensor temperature, which may not correspond to the ambient temperature.
 */
float mpuGetTemp(MPU6050_t* sensor)
{
	uint8_t readBuffer[2];
	I2C_RETURN_CODE_t i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2c_address, MPU6050_Temp, readBuffer, 2);
	if (i2c_return == I2C_OK)
	{
		sensor->temp_raw = (int16_t) (readBuffer[0]<<8) + readBuffer[1];
	}
	else
	{
		sensor->temp_raw = (int16_t) 0;
	}
	sensor->temperature = mpuTemp(sensor);
	return (sensor->temperature);
}


float mpuTemp(MPU6050_t* sensor)
{
	const float temp_factor = (float)1.0 / 340;
	const float temp_offset = 36.35;

	sensor->temperature = (float) (sensor->temp_raw) * temp_factor + temp_offset;
	return (sensor->temperature);
}



/**
 * @function mpuInitLowpassFilter
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
void mpuSetLpFilt(MPU6050_t* sensor) {
	i2cSendByteToSlaveReg(sensor->i2c, sensor->i2c_address, MPU6050_CONFIG, sensor->LowPassFilt);
}
