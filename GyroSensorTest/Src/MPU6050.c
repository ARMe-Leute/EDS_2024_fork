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

uint8_t initMPU(MPU6050_t* sensor, I2C_TypeDef* i2cBus, uint8_t i2cAddr, uint8_t gyroScale, uint8_t accelRange, uint8_t restart) {
	//ToDo: Restliche Sensorinitialisierung auch noch in diese Methode? oder einzelne?
	sensor->i2c = i2cBus;

	if(i2cAddr == 0x68) {
		sensor->i2cAddress = i2cAddr;
	}
	else {
		// ToDo: set method for non standard i2c address
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
		//ToDo: Funktion implementieren, die Gyroregister bei Eingabe 0xff zu deaktivieren
		break;
	default: //ToDo: maybe adjust handling of wrong user input
		sensor->GyroScale = (uint8_t) MPU6050_GYRO_FSCALE_2000;
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
	case MPU6050_ACCEL_RANGE_10:
			sensor->AccelRange = (uint8_t) MPU6050_ACCEL_RANGE_10;
			break;
	default: //ToDo: maybe adjust handling of wrong user input
			sensor->AccelRange = (uint8_t) MPU6050_ACCEL_RANGE_10;
			break;
	}

	static int8_t step = -5;
		if ((restart != 0) && (step == -5))
		{
			step += 1;
		}
		switch (step)
			{

				case -6:		// CLK Speed von I2C auf 400kHz
				{
					i2cSetClkSpd(sensor->i2c, I2C_CLOCK_1Mz); //set I2C Clock 1000kHz
					step = -5;
				}
				break;
				case -5:		// SW Reset
				{
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_SWRESET)); // reboot memory content
					step = -4;
				}
				break;
				case -4:
				{		// PWR Mngt
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_TEMP_dis|MPU6050_PWR1_CLKSEL));
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_2, (MPU6050_PWR2_ACConXY_GYonZ));
					step = -3;
				}
				break;
				case -3:
				{		// GYRO Config
					if (sensor->GyroScale == 0xff) {
						//ToDo: Funktion zum Ausschalten finden und einfügen
						step=-2;
					}
					else {
						i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_GYRO_CONFIG, sensor->GyroScale); 	// set scale range of gyroscope
						step = -2;
					}
				}
				break;
				case -2:
				{		// ACCEL Config
					i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_ACCEL_CONFIG, sensor->AccelRange); 	// set scale range of accelerometer
					step = -1;
				}
				break;
				case -1:
				{		// LowPass Config
					// i2cMPU6050LpFilt(sensor->i2c, 4); //ToDo: Funktion überprüfen
					step = 0;
				}
				break;

				default:
				{
					step = -5;
				}
			}
		return step;
}
