/*
 * 3DG_Senor.h
 *
 *  Created on: 01.03.2024
 *      Author: jakob, micha
 * edited: 30.3.2024 T Flämig
 */

#ifndef SENSOR3DG_H_
#define SENSOR3DG_H_

#define neu

/* Choose your Sensor*/
#define BMA020
#define LIS3DH
#define MPU6050
/******************/

#define _pi (float)3.141


void getAngleFromAcc(int16_t *xyz, float *AlphaBeta);
void getAccData(I2C_TypeDef *i2c, int16_t *xyz);
void getFiltertAccData(int16_t *XYZ, float *XYZFiltert, float kFilt);
extern int8_t sensor_init(I2C_TypeDef *i2c, int8_t restart);
extern int8_t i2cBMA020_init(I2C_TypeDef *i2c, int8_t restart);
extern int16_t i2cBMA020XYZ(I2C_TypeDef *i2c, int16_t *xyz);

extern int8_t i2cMPU6050_init(I2C_TypeDef *i2c, int8_t restart);
extern int16_t i2cMPU6050XYZ(I2C_TypeDef *i2c, int16_t *xyz);
extern int16_t i2cMPU6050GYRO(I2C_TypeDef *i2c, int16_t *xyz);
extern void i2cMPU6050LpFilt(I2C_TypeDef *i2c, uint8_t DLPF_CFG);

extern int8_t i2cLIS3DH_init(I2C_TypeDef *i2c, int8_t restart);
extern int8_t i2cLIS3DH_Temp(I2C_TypeDef *i2c);
extern int16_t i2cLIS3DH_XYZ(I2C_TypeDef *i2c, int16_t *xyz);




typedef enum {
    SENSOR_NONE,
    SENSOR_BMA020,
    SENSOR_MPU6050,
    SENSOR_LIS3DH
} AccelerometerSensor;

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
extern AccelerometerSensor currentSensor;
extern const uint16_t sensorScale[];
extern const uint8_t i2cAddr_Sensor[];
extern const uint8_t DummyReg[];
extern const uint8_t dummyVal[];
extern const int8_t stepStart[];

/*
 * individuelle Adressen
 */

#define i2cAddr_LIS3DH 0x18
// LIS3DH

#define StatusReg 	0x27
#define AcclReg 	0x28
#define ADC1Reg 	0x08
#define ADC3Temp 	0x0C
#define ADCConfReg 	0x1F
#define CtrlReg1 	0x20
#define CtrlReg4	0x23
#define CtrlReg5 	0x24
#define StatusReg 	0x27
#define OUTxyz 		0x28
#define I2Crepeat 	0x80
#define XYZopt





/*
 * BMA020
 */

#define i2cAddr_BMA020 		0x38
#define BMA020_DR			0x02
#define BMA020_CR1			0x15
#define BMA020_CR2			0x14
#define BMA020_CR3			0x0A

// Bitmasken

//	Range
#define BMA020_ACCEL_RANGE_2g 	0b00000
#define BMA020_ACCEL_RANGE_4g 	0b01000
#define BMA020_ACCEL_RANGE_8g 	0b10000

//	Bandwidth
#define BMA020_ACCEL_BANDW_25	0b000
#define BMA020_ACCEL_BANDW_50	0b001
#define BMA020_ACCEL_BANDW_100	0b010
#define BMA020_ACCEL_BANDW_190	0b011
#define BMA020_ACCEL_BANDW_375	0b100
#define BMA020_ACCEL_BANDW_750	0b101
#define BMA020_ACCEL_BANDW_1500	0b110

// Shadowing
#define BMA020_SHADW_DIS	0b1000

// SW-Reset
#define BMA020_SWRESET		0b10	//BMA020 is reset each time this bit (address 0Ah, bit 1) is written to 1




/*
 * MPU6050
 */
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




/*
 * LIS3DH
 */

#define i2cAddr_LIS3DH 		0x18

#define LIS3DH_ADC3Temp 	0x0C
#define LIS3DH_ADCConfReg 	0x1F
#define LIS3DH_CtrlReg1 	0x20
#define LIS3DH_CtrlReg4		0x23
#define LIS3DH_CtrlReg5 	0x24
#define LIS3DH_I2Crepeat 	0x80

extern bool enable3DGSensor;

#endif /* 3DG_SENOR_H_ */
