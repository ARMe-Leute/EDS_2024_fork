/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <stm32f4xx.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <RotaryPushButton.h>
#include <MPU6050.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void i2cActivate()
{
	I2C_TypeDef   *i2c  = I2C1;

	GPIO_TypeDef  *portB = GPIOB;
	// GPIOB-Bustakt aktivieren wegen der Verwendung von PB8/PB9 (I2C).
	i2cSelectI2C(i2c);                           // I2C1: Bustakt aktivieren
	//i2cDisableDevice(i2c);
	gpioInitPort(portB);
	gpioSelectPinMode(portB, PIN8, ALTFUNC);
	gpioSelectAltFunc(portB, PIN8, AF4);         // PB8 : I2C1 SCL
	gpioSelectPinMode(portB, PIN9, ALTFUNC);
	gpioSelectAltFunc(portB, PIN9, AF4);         // PB9 : I2C1 SDA
	/*
	 * Verwenden Sie auf keinen Fall die MCU-internen Pull-up-Widerstaende!
	 * Widerstandswerte: jeweils 4k7 fuer SDA und SCL!
	 */
	gpioSetOutputType(portB, PIN8, OPENDRAIN);   // Immer externe Pull-up-
	gpioSetOutputType(portB, PIN9, OPENDRAIN);   // Widerstaende verwenden!!!
	// Initialisierung des I2C-Controllers
	i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_50);
	i2cEnableDevice(i2c);   // MCAL I2C1 activ
}

int main(void)
{
	initRotaryPushButton();
	initRotaryPushButtonLED();
	setRotaryColor(LED_BLACK);

	/* I²C initialisieren */
	i2cActivate();

	MPU6050_t MPU1;
	int8_t testVal = MPU_init(&MPU1, I2C1, i2cAddr_MPU6050, MPU6050_GYRO_FSCALE_250, MPU6050_ACCEL_RANGE_2, 1);

	if (testVal == 0)
	{
		setRotaryColor(LED_GREEN);
		/* Erfolgreiche Kommunikation */
		for (;;)
		{
			testVal = MPU_get_angle_from_acceleration(&MPU1);
			testVal += MPU_get_temperature(&MPU1);
			testVal += MPU_get_gyro(&MPU1);
			// In Grad umrechnen, da besser greifbar
			MPU1.AlphaBeta[0] = MPU1.AlphaBeta[0]*180/_pi;
			MPU1.AlphaBeta[1] = MPU1.AlphaBeta[1]*180/_pi;
			if (testVal != 0) {
				setRotaryColor(LED_RED);
			}
		}
	}
	else
	{
		setRotaryColor(LED_RED);
		/* Fehlgeschlagene Kommunikation */
		for (;;)
		{
			/* Fehlerbehandlung, z. B. LED blinkt langsamer */
		}
	}
}
