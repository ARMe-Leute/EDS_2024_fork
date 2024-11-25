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
// #include <ST7735.h> //DEL
// #include "GyroSensor.h"
#include "RotaryPushButton.h"
#include "MPU6050.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/*
 * Defines
 */
#define MPU6050_i2cAdress i2cAddr_MPU6050 << 1 // 7 Bit Adresse
#define currentSensor 'SENSOR_MPU6050'

/*
 * Methods
 */
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

    /**
     * Verwenden Sie auf keinen Fall die MCU-internen Pull-up-Widerstaende!
     * Widerstandswerte: jeweils 4k7 fuer SDA und SCL!
     */
    gpioSetOutputType(portB, PIN8, OPENDRAIN);   // Immer externe Pull-up-
    gpioSetOutputType(portB, PIN9, OPENDRAIN);   // Widerstaende verwenden!!!
    // Initialisierung des I2C-Controllers
    i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_50);
    i2cEnableDevice(i2c);   // MCAL I2C1 activ
#ifdef BALA2024

    // GPIOB-Bustakt aktivieren wegen der Verwendung von PB10/PB3 (I2C).
    i2cSelectI2C(i2c2);                           // I2C2: Bustakt aktivieren
    gpioSelectPinMode(portB, PIN10, ALTFUNC);
    gpioSelectAltFunc(portB, PIN10, AF4);         // PB10 : I2C2 SCL
    gpioSelectPinMode(portB, PIN3, ALTFUNC);
    gpioSelectAltFunc(portB, PIN3, AF9);         // PB3 : 	I2C2 SDA

    /**
     * Verwenden Sie auf keinen Fall die MCU-internen Pull-up-Widerstaende!
     * Widerstandswerte: jeweils 4k7 fuer SDA und SCL!
     */
    gpioSetOutputType(portB, PIN10, OPENDRAIN);   // Immer externe Pull-up-
    gpioSetOutputType(portB, PIN3, OPENDRAIN);   // Widerstaende verwenden!!!
    // Initialisierung des I2C-Controllers
    i2cInitI2C(i2c2, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_50);
    i2cEnableDevice(i2c2);                        // MCAL I2C2 activ
#endif /* BALA2024 */
}

static bool checkI2CCommunication(void)
{
    int8_t sensorStatus = sensor_init(I2C1, 1);
    if (sensorStatus == 0)
    {
        return true; /* Kommunikation erfolgreich */
    }
    return false; /* Fehler bei der Kommunikation */
}

int main(void)
{
	initRotaryPushButton();
	setRotaryColor(LED_BLACK);
	bool communicationOk;

	/* I²C initialisieren */
	i2cActivate();

	MPU6050_t MPU1;
	uint8_t testVal = initMPU(&MPU1, I2C1, i2cAddr_MPU6050, MPU6050_GYRO_FSCALE_250, MPU6050_ACCEL_RANGE_2, 1);

	/* Kommunikation mit dem Sensor prüfen */
	communicationOk = checkI2CCommunication();

	if (communicationOk && testVal == 0)
	{
		setRotaryColor(LED_GREEN);
		/* Erfolgreiche Kommunikation */
		for (;;)
		{
			/* Blinkt z. B. eine LED oder setzt Debug-Ausgabe */
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
