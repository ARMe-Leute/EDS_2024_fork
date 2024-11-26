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

/**
 * Sets given Values to instance of MPU6050 Gyro Sensor
 *
 * @param sensor:		Pointer to instance of MPU6050_t type
 * @param i2cBus:		Pointer to instance of I2C_TypeDef type
 * @param i2cAddr:		I2C Address to be assigned to Sensor
 * @param gyroScale:	Bit mask for setting the scaling of the gyroscope measurement
 * @param accelRange: 	Bit mask for setting the range of the acceleration measurement
 * @param restart:		//ToDo: Figure out why this is necessary
 */
int8_t initMPU(MPU6050_t* sensor, I2C_TypeDef* i2cBus, uint8_t i2cAddr, uint8_t gyroScale, uint8_t accelRange, uint8_t restart) {

	sensor->i2c = i2cBus;

	if(i2cAddr == 0x68) {
		sensor->i2cAddress = i2cAddr;
	}
	else {
		// ToDo: set method for non standard i2c address?
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
			step = -4;
			break;

		case -4:
			// PWR Mngt
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_TEMP_dis|MPU6050_PWR1_CLKSEL));
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_2, (MPU6050_PWR2_ACConXY_GYonZ));
			step = -3;
			break;

		case -3:
			// GYRO Config
			if (sensor->GyroScale == 0xff) {
				//ToDo: Funktion zum Ausschalten finden und einfügen
				step=-2;
			}
			else {
				i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_GYRO_CONFIG, sensor->GyroScale); 	// set scale range of gyroscope
				step = -2;
			}
			break;

		case -2:
			// ACCEL Config
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_ACCEL_CONFIG, sensor->AccelRange); 	// set scale range of accelerometer
			step = -1;
			break;

		case -1:	// LowPass Config
			// i2cMPU6050LpFilt(sensor->i2c, 4); //ToDo: Funktion überprüfen
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_CONFIG, (0b0111 & 4));
			step = 0;
			break;

		default:
			step = -5;
		}
	}

	return step;
}

int16_t getAcceleration(MPU6050_t* sensor) {

	uint8_t readBuffer[6];
	i2cBurstRegRead(sensor->i2c, sensor->i2cAddress, MPU6050_AccXYZ, readBuffer, 6);
	sensor->AccelXYZ[0] = (readBuffer[0]<<8) + readBuffer[1];
	sensor->AccelXYZ[1] = (readBuffer[2]<<8) + readBuffer[3];
	sensor->AccelXYZ[2] = (readBuffer[4]<<8) + readBuffer[5];

	/* ToDo: Kommentierte alte Funktion entfernen, wenn Funktion tut
	*xyz = (readBuffer[0]<<8) + readBuffer[1];
	xyz++;
	*xyz = (readBuffer[2]<<8) + readBuffer[3];
	xyz++;
	*xyz = (readBuffer[4]<<8) + readBuffer[5];
	*/
	return 0;
}

int16_t getAngleFromAcc(MPU6050_t* sensor) {
	int16_t returnValue = getAcceleration(sensor);
	float X = (float) sensor->AccelXYZ[0]/160;
	float Y = (float) sensor->AccelXYZ[1]/160;
	float Z = (float) sensor->AccelXYZ[2]/160;

	sensor->AlphaBeta[0] = atan(X / Z);
	if (Z < 0) {
		if (X < 0) {
			sensor->AlphaBeta[0] -= _pi;
		} else {
			sensor->AlphaBeta[0] += _pi;
		}
	}

	sensor->AlphaBeta[1] = atan(Y / Z);
	if (Z < 0) {
		if (Y < 0) {
			sensor->AlphaBeta[1] -= _pi;
		} else {
			sensor->AlphaBeta[1] += _pi;
		}
	}
	return returnValue;

	/* ToDo: Kommentierte alte Funktion entfernen
	float X = (float) xyz[0]/160;
	float Y = (float) xyz[1]/160;
	float Z = (float) xyz[2]/160;

	AlphaBeta[0] = atan(X / Z);
	if (Z < 0) {
		if (X < 0) {
			AlphaBeta[0] -= _pi;
		} else {
			AlphaBeta[0] += _pi;
		}
	}

	AlphaBeta[1] = atan(Y / Z);
	if (Z < 0) {
		if (Y < 0) {
			AlphaBeta[1] -= _pi;
		} else {
			AlphaBeta[1] += _pi;
		}
	}
	*/
}
