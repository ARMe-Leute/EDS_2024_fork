/*
 * MPU6050.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Müller, Berenspöhler
 */

#ifndef MPU6050_H_
#define MPU6050_H_

//********** Defines for Preset Values **********

#define _pi 3.141
#define _g 9.81

#define i2cAddr_MPU6050         0x68

// Register addresses for configuration
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_FIFO_EN         0x23
#define MPU6050_MST_CTRL        0x24
#define MPU6050_AccXYZ          0x3B
#define MPU6050_GyroXYZ         0x43
#define MPU6050_Temp            0x41

// Bit masks
#define MPU6050_SWRESET                 0b10000000
#define MPU6050_PLL_AXIS_X              0b1
#define MPU6050_PWR1_CLKSEL             0b00000000
#define MPU6050_PWR1_TEMP_dis           0b00001000
#define MPU6050_PWR2_ACConXY_GYonZ      0b00001110

// Communication protocol
#define MPU6050_MST_P_NSR               0b00010000

// Gyroscope configuration (FS_SEL)
#define MPU6050_GYRO_FSCALE_250         0b00000 // ±250°/s
#define MPU6050_GYRO_FSCALE_500         0b01000 // ±500°/s
#define MPU6050_GYRO_FSCALE_1000        0b10000 // ±1000°/s
#define MPU6050_GYRO_FSCALE_2000        0b11000 // ±2000°/s

// Accelerometer configuration (AFS_SEL)
#define MPU6050_ACCEL_RANGE_2           0b00000 // ±2g
#define MPU6050_ACCEL_RANGE_4           0b01000 // ±4g
#define MPU6050_ACCEL_RANGE_8           0b10000 // ±8g
#define MPU6050_ACCEL_RANGE_16          0b11000 // ±16g

// Low-pass filter configuration
#define MPU6050_LPBW_260                0b00000 // Accelerometer: 260Hz, Gyroscope: 256Hz
#define MPU6050_LPBW_184                0b00001 // Accelerometer: 184Hz, Gyroscope: 188Hz
#define MPU6050_LPBW_94                 0b00010 // Accelerometer: 94Hz, Gyroscope: 98Hz
#define MPU6050_LPBW_44                 0b00011 // Accelerometer: 44Hz, Gyroscope: 42Hz
#define MPU6050_LPBW_21                 0b00100 // Accelerometer: 21Hz, Gyroscope: 20Hz
#define MPU6050_LPBW_10                 0b00101 // Accelerometer: 10Hz, Gyroscope: 10Hz
#define MPU6050_LPBW_5                  0b00110 // Accelerometer: 5Hz, Gyroscope: 5Hz

// External sensor register addresses
#define MPU6050_EXT_SENS_DATA_00        0x49

// Additional registers
#define MPU6050_I2C_SLV0_DO             0x63
#define MPU6050_I2C_SLV1_DO             0x64
#define MPU6050_I2C_SLV2_DO             0x65
#define MPU6050_I2C_SLV3_DO             0x66

#define DISABLE                         0xFF
#define RESTART							0x01
#define NO_RESTART						0x00

//********** Type Definitions **********

typedef struct MPU6050 {
    I2C_TypeDef* i2c;
    uint8_t i2c_address;
    uint8_t gyro_scale;
    float gyro_scale_factor;
    uint8_t accel_range;
    float accel_range_factor;
    uint8_t low_pass_filt_config;
    float temperature_out;
    float temperature_factor;
    float temperature_offset;
    float gyro_xyz[3];
    float accel_xyz[3];
    float alpha_beta[2];
} MPU6050_t;

//--------------------------- Function Declarations ---------------------------

int8_t mpuInit(MPU6050_t* sensor, I2C_TypeDef* i2cBus, uint8_t i2cAddress, uint8_t gyroScale, uint8_t accelRange, uint8_t lPconfig, uint8_t restart);

int16_t mpuGetAcceleration(MPU6050_t* sensor);

int16_t mpuGetAngleFromAcceleration(MPU6050_t* sensor);

int16_t mpuGetGyro(MPU6050_t* sensor);

int16_t mpuGetTemperature(MPU6050_t* sensor);

void mpuInitLowpassFilter(MPU6050_t* sensor);

#endif /* MPU6050_H_ */
