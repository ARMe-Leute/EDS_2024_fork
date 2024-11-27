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
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_MST_CTRL, (MPU6050_MST_P_NSR)); // tell sensor to expect stop bit
			step = -4;
			break;

		case -4:
			// PWR Mngt
			//i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_TEMP_dis|MPU6050_PWR1_CLKSEL));
			i2cSendByteToSlaveReg(sensor->i2c, sensor->i2cAddress, MPU6050_PWR_MGMT_1, (MPU6050_PWR1_CLKSEL));
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

int16_t getAcceleration(MPU6050_t* sensor) { //ToDo: Überprüfen, wieso Z-Wert immer 0x00 (Sensor?)
	I2C_RETURN_CODE_t i2c_return;
	uint8_t readBuffer[6];
	int16_t X, Y, Z;
	i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2cAddress, MPU6050_AccXYZ, readBuffer, 6);
	X = (readBuffer[0]<<8) + readBuffer[1];
	Y = (readBuffer[2]<<8) + readBuffer[3];
	Z = (readBuffer[4]<<8) + readBuffer[5];

	switch(sensor->AccelRange) {
	case MPU6050_ACCEL_RANGE_2:		// Faktor 16384 LSB/g
		sensor->AccelXYZ[0] = (float) X / 16384;
		sensor->AccelXYZ[1] = (float) Y / 16384;
		sensor->AccelXYZ[2] = (float) Z / 16384;
	case MPU6050_ACCEL_RANGE_4:		// Faktor 8192 LSB/g
		sensor->AccelXYZ[0] = (float) X / 8192;
		sensor->AccelXYZ[1] = (float) Y / 8192;
		sensor->AccelXYZ[2] = (float) Z / 8192;
	case MPU6050_ACCEL_RANGE_8:		// Faktor 4096 LSB/g
		sensor->AccelXYZ[0] = (float) X / 4096;
		sensor->AccelXYZ[1] = (float) Y / 4096;
		sensor->AccelXYZ[2] = (float) Z / 4096;
	case MPU6050_ACCEL_RANGE_10:	// Faktor 2048 LSB/g
		sensor->AccelXYZ[0] = (float) X / 2048;
		sensor->AccelXYZ[1] = (float) Y / 2048;
		sensor->AccelXYZ[2] = (float) Z / 2048;
	}
	return (int16_t) i2c_return;
}

int16_t getAngleFromAcc(MPU6050_t* sensor) { //ToDo: Berechnung der Winkel checken
	int16_t returnValue = getAcceleration(sensor);
	float X = (float) sensor->AccelXYZ[0]*(_pi/180);
	float Y = (float) sensor->AccelXYZ[1]*(_pi/180);
	float Z = (float) sensor->AccelXYZ[2]*(_pi/180);

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
}

int16_t getGyro(MPU6050_t* sensor) {
	uint8_t readBuffer[6];
	int16_t X, Y, Z;
	I2C_RETURN_CODE_t i2c_return;
	i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2cAddress, MPU6050_GyroXYZ, readBuffer, 6);
	X = (readBuffer[0]<<8) + readBuffer[1];
	Y = (readBuffer[2]<<8) + readBuffer[3];
	Z = (readBuffer[4]<<8) + readBuffer[5];

	switch (sensor->GyroScale) {
	case MPU6050_GYRO_FSCALE_250:		// Faktor 131 LSB/°/s
		sensor->GyroXYZ[0] = (float) X / 131;
		sensor->GyroXYZ[1] = (float) Y / 131;
		sensor->GyroXYZ[2] = (float) Z / 131;
	case MPU6050_GYRO_FSCALE_500:		// Faktor 65.5 LSB/°/s
		sensor->GyroXYZ[0] = (float) X / 65.5;
		sensor->GyroXYZ[1] = (float) Y / 65.5;
		sensor->GyroXYZ[2] = (float) Z / 65.5;
	case MPU6050_GYRO_FSCALE_1000:		// Faktor 32.8 LSB/°/s
		sensor->GyroXYZ[0] = (float) X / 32.8;
		sensor->GyroXYZ[1] = (float) Y / 32.8;
		sensor->GyroXYZ[2] = (float) Z / 32.8;
	case MPU6050_GYRO_FSCALE_2000:		// Faktor 16.4 LSB/°/s
		sensor->GyroXYZ[0] = (float) X / 16.4;
		sensor->GyroXYZ[1] = (float) Y / 16.4;
		sensor->GyroXYZ[2] = (float) Z / 16.4;

	}
	return (int16_t) i2c_return;
}

int16_t getTemperature(MPU6050_t* sensor) {
	uint8_t readBuffer[4];
	int16_t rawTemp;
	I2C_RETURN_CODE_t i2c_return;
	i2c_return = i2cBurstRegRead(sensor->i2c, sensor->i2cAddress, MPU6050_Temp, readBuffer, 2);
	rawTemp = (int16_t) (readBuffer[0]<<8) + readBuffer[1];
	sensor->TempOut = (float) rawTemp/340 + 36.35;
	return (int16_t) i2c_return;
}

