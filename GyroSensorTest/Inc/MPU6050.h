/*
 * MPU6050.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Müller, Berenspöhler
 */

#ifndef MPU6050_H_
#define MPU6050_H_

//**********Defines for Preset Values**********

#define _pi 3.141
#define _g 9.81

#define i2cAddr_MPU6050 		0x68

// Registeradressen für Config
#define MPU6050_PWR_MGMT_1		0x6B
#define MPU6050_PWR_MGMT_2		0x6C
#define MPU6050_CONFIG 			0x1A
#define MPU6050_GYRO_CONFIG 	0x1B
#define MPU6050_ACCEL_CONFIG 	0x1C
#define MPU6050_FIFO_EN			0x23
#define MPU6050_MST_CTRL		0x24
#define MPU6050_AccXYZ			0x3B
#define MPU6050_GyroXYZ			0x43
#define MPU6050_Temp			0x41

//	Bitmasken

// SW RESET
#define MPU6050_SWRESET				0b10000000
#define MPU6050_PLL_AXIS_X			0b1
#define MPU6050_PWR1_CLKSEL			0b00000000
#define MPU6050_PWR1_TEMP_dis		0b00001000
#define MPU6050_PWR2_ACConXY_GYonZ	0b00001110

// Kommunikationsprotokoll
#define MPU6050_MST_P_NSR			0b00010000

// GYRO_CONFIG	(FS_SEL)
#define MPU6050_GYRO_FSCALE_250		0b00000   //full scale range of gyroscope = ± 250 °/s
#define MPU6050_GYRO_FSCALE_500		0b01000   //full scale range of gyroscope = ± 500 °/s
#define MPU6050_GYRO_FSCALE_1000	0b10000   //full scale range of gyroscope = ± 1000 °/s
#define MPU6050_GYRO_FSCALE_2000	0b11000   //full scale range of gyroscope = ± 2000 °/s

// ACCEL_CONFIG (AFS_SEL)
#define MPU6050_ACCEL_RANGE_2		0b00000   //full scale range of accelerometer = ± 2g
#define MPU6050_ACCEL_RANGE_4		0b01000   //full scale range of accelerometer = ± 4g
#define MPU6050_ACCEL_RANGE_8		0b10000   //full scale range of accelerometer = ± 8g
#define MPU6050_ACCEL_RANGE_10		0b11000   //full scale range of accelerometer = ± 10g

//Registeradressen Externe Sensoren
#define MPU6050_EXT_SENS_DATA_00 0x49

// Weitere Register
#define MPU6050_I2C_SLV0_DO 0x63
#define MPU6050_I2C_SLV1_DO 0x64
#define MPU6050_I2C_SLV2_DO 0x65
#define MPU6050_I2C_SLV3_DO 0x66

#define DISABLE 0xff
//**********TypeDefs**********

// Pre- declare MPU6050 Type
// typedef struct MPU6050 MPU6050_t;

typedef struct {
    uint8_t ACCEL_Burst_Reg;  	// Burst-Register für die Beschleunigung
    uint8_t ACCEL_XOUT_H;      	// MSB des Beschleunigungswertes in X-Richtung
    uint8_t ACCEL_XOUT_L;      	// LSB des Beschleunigungswertes in X-Richtung
    uint8_t ACCEL_YOUT_H;      	// MSB des Beschleunigungswertes in Y-Richtung
    uint8_t ACCEL_YOUT_L;      	// LSB des Beschleunigungswertes in Y-Richtung
    uint8_t ACCEL_ZOUT_H;      	// MSB des Beschleunigungswertes in Z-Richtung
    uint8_t ACCEL_ZOUT_L;      	// LSB des Beschleunigungswertes in Z-Richtung
} AccelRegisters_t;

typedef struct {
    uint8_t GYRO_XOUT_H;  		// MSB des Gyroskopwertes in X-Richtung
    uint8_t GYRO_XOUT_L;  		// LSB des Gyroskopwertes in X-Richtung
    uint8_t GYRO_YOUT_H; 		// MSB des Gyroskopwertes in Y-Richtung
    uint8_t GYRO_YOUT_L;  		// LSB des Gyroskopwertes in Y-Richtung
    uint8_t GYRO_ZOUT_H;  		// MSB des Gyroskopwertes in Z-Richtung
    uint8_t GYRO_ZOUT_L;  		// LSB des Gyroskopwertes in Z-Richtung
} GyroRegisters_t;

typedef struct {
    uint8_t TEMP_OUT_H;  		// MSB des Temperaturwertes
    uint8_t TEMP_OUT_L;  		// LSB des Temperaturwertes
} TempRegisters_t;

typedef struct MPU6050 {
	I2C_TypeDef* 		i2c;
	uint8_t 			i2cAddress;
	uint8_t				GyroScale;
	uint8_t				AccelRange;
	// uint8_t				LPFiltConfig;
	float	 			TempOut;
	float				GyroXYZ[3];
	float				AccelXYZ[3];
	float				AlphaBeta[2];
} MPU6050_t;

//---------------------------INTERNAL FUNCTIONS---------------------------

/**
 * @function MPU_init
 *
 * @brief Initializes the MPU6050 sensor with the specified configuration values.
 *
 * This function configures the MPU6050 sensor, including its I2C address, gyroscope scale,
 * accelerometer range, and power/reset management. It ensures proper initialization
 * and prepares the sensor for data acquisition. The initialization process is divided
 * into multiple steps, allowing for flexibility and fault tolerance.
 *
 * @param sensor       Pointer to an instance of the MPU6050_t structure, which stores the sensor's configuration and state.
 * @param i2cBus       Pointer to the I2C bus instance (I2C_TypeDef) used for communication with the sensor.
 * @param i2cAddr      The I2C address to assign to the sensor. If an invalid address is provided, the default (0x68) is used.
 * @param gyroScale    A bitmask to configure the gyroscope's sensitivity range:
 *                     - `MPU6050_GYRO_FSCALE_250`: ±250°/s
 *                     - `MPU6050_GYRO_FSCALE_500`: ±500°/s
 *                     - `MPU6050_GYRO_FSCALE_1000`: ±1000°/s
 *                     - `MPU6050_GYRO_FSCALE_2000`: ±2000°/s
 *                     - `DISABLE`: Disable gyroscope measurement
 * @param accelRange   A bitmask to configure the accelerometer's measurement range:
 *                     - `MPU6050_ACCEL_RANGE_2`: ±2g
 *                     - `MPU6050_ACCEL_RANGE_4`: ±4g
 *                     - `MPU6050_ACCEL_RANGE_8`: ±8g
 *                     - `MPU6050_ACCEL_RANGE_10`: ±16g
 *                     - `DISABLE`: Disable accelerometer measurement
 * @param restart      Indicates whether to restart the initialization process. Non-zero values trigger a restart.
 *
 * @return int8_t      Returns 0 if all initialization steps complete successfully, or a negative value if an error occurs during the process.
 *
 * @details
 * The function follows these steps:
 * 1. Configures the sensor's I2C bus and validates the I2C address.
 * 2. Sets the gyroscope scale and accelerometer range according to the provided bitmasks.
 * 3. Manages power and reset settings:
 *    - Resets the sensor memory.
 *    - Configures the power management registers based on the measurement mode.
 * 4. Applies the gyroscope and accelerometer configurations to the sensor's internal registers.
 * 5. Configures the low-pass filter settings for the sensor.
 *
 * If a `restart` is requested, the initialization process begins from the initial step.
 * Any errors encountered during initialization will result in a negative return value.
 */
int8_t MPU_init(MPU6050_t* sensor, I2C_TypeDef* i2cBus, uint8_t i2cAddress, uint8_t gyroScale, uint8_t accelRange, uint8_t restart);

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
int16_t MPU_get_acceleration(MPU6050_t* sensor);

/**
 * @function MPU_get_angle_from_acceleration
 *
 * @brief Calculates the tilt angles of the MPU6050 sensor based on acceleration data.
 *
 * This function computes the tilt angles (alpha and beta) of the MPU6050 sensor in radians using the
 * acceleration data from the X, Y, and Z axes. The angles are stored in the `AlphaBeta` array of
 * the provided sensor instance.
 *
 * @param sensor       Pointer to an instance of the `MPU6050_t` structure containing the sensor configuration and state.
 *
 * @return int16_t     Returns the result of the `getAcceleration` function, which indicates the
 *                     status of the I2C transaction. A value of `I2C_SUCCESS` (0) indicates success.
 *
 * @details
 * 1. Calls the `getAcceleration` function to retrieve the latest acceleration data from the sensor.
 * 2. Converts the acceleration values to radians by multiplying by a constant factor (`π/180`).
 * 3. Calculates the tilt angles:
 *    - `AlphaBeta[0]`: Tilt angle in the X-Z plane (alpha) using the formula `atan(X / Z)`.
 *    - `AlphaBeta[1]`: Tilt angle in the Y-Z plane (beta) using the formula `atan(Y / Z)`.
 * 4. Adjusts the calculated angles based on the sign of the Z-axis acceleration to handle angle wrapping:
 *    - Adds or subtracts π as needed for correct angle representation.
 */
int16_t MPU_get_angle_from_acceleration(MPU6050_t* sensor);

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
int16_t MPU_get_gyro(MPU6050_t* sensor);

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
int16_t MPU_get_temperature(MPU6050_t* sensor);


int16_t MPU_init_lowpass_filter(MPU6050_t* sensor);

// extern const MPU6050_t sensor;

#endif /* MPU6050_H_ */
