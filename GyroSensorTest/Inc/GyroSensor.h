/*
 * GyroSensor.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Berenspöhler
 */

#ifndef GYROSENSOR_H_
#define GYROSENSOR_H_

/*
 * Internal Defines
 */
#define PI (float)3.141

void getAngleFromAcc(int16_t *xyz, float *AlphaBeta);
void getAccData(I2C_TypeDef *i2c, int16_t *xyz);
void getFiltertAccData(int16_t *XYZ, float *XYZFiltert, float kFilt);

extern int8_t i2cMPU6050_init(I2C_TypeDef *i2c, int8_t restart);
extern int16_t i2cMPU6050XYZ(I2C_TypeDef *i2c, int16_t *xyz);
extern int16_t i2cMPU6050GYRO(I2C_TypeDef *i2c, int16_t *xyz);
extern void i2cMPU6050LpFilt(I2C_TypeDef *i2c, uint8_t DLPF_CFG);

// Struktur für die Beschleunigungsregister
typedef struct {
    uint8_t ACCEL_Burst_Reg;  // Burst-Register für die Beschleunigung
    uint8_t ACCEL_XOUT_H;      // MSB des Beschleunigungswertes in X-Richtung
    uint8_t ACCEL_XOUT_L;      // LSB des Beschleunigungswertes in X-Richtung
    uint8_t ACCEL_YOUT_H;      // MSB des Beschleunigungswertes in Y-Richtung
    uint8_t ACCEL_YOUT_L;      // LSB des Beschleunigungswertes in Y-Richtung
    uint8_t ACCEL_ZOUT_H;      // MSB des Beschleunigungswertes in Z-Richtung
    uint8_t ACCEL_ZOUT_L;      // LSB des Beschleunigungswertes in Z-Richtung
} AccelRegisters;

// Struktur für die Gyroskopregister
typedef struct {
    uint8_t GYRO_XOUT_H;  // MSB des Gyroskopwertes in X-Richtung
    uint8_t GYRO_XOUT_L;  // LSB des Gyroskopwertes in X-Richtung
    uint8_t GYRO_YOUT_H;  // MSB des Gyroskopwertes in Y-Richtung
    uint8_t GYRO_YOUT_L;  // LSB des Gyroskopwertes in Y-Richtung
    uint8_t GYRO_ZOUT_H;  // MSB des Gyroskopwertes in Z-Richtung
    uint8_t GYRO_ZOUT_L;  // LSB des Gyroskopwertes in Z-Richtung
} GyroRegisters;

// Struktur für die Temperaturregister
typedef struct {
    uint8_t TEMP_OUT_H;  // MSB des Temperaturwertes
    uint8_t TEMP_OUT_L;  // LSB des Temperaturwertes
} TempRegisters;

extern const AccelRegisters registerAddresses[];
extern const GyroRegisters gyroRegisters[];
extern const TempRegisters tempRegisters[];
// extern AccelerometerSensor currentSensor;
extern const uint16_t sensorScale[];
extern const uint8_t i2cAddr_Sensor[];
extern const uint8_t DummyReg[];
extern const uint8_t dummyVal[];
extern const int8_t stepStart[];

#define i2cAddr_MPU6050 		0x68
// Registeradressen für Config
#define MPU6050_PWR_MGMT_1		0x6B
#define MPU6050_PWR_MGMT_2		0x6C
#define MPU6050_CONFIG 			0x1A
#define MPU6050_GYRO_CONFIG 	0x1B
#define MPU6050_ACCEL_CONFIG 	0x1C
#define MPU6050_FIFO_EN			0x23
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

extern bool enableGyroSensor;

#endif /* GYROSENSOR_H_ */
