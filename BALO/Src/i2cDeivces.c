/*
 * I2C.c
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

//#include <adcBAT.h>
#include <mcalGPIO.h>
#include <mcalI2C.h>
//#include <mcalSysTick.h>
//#include <i2cTOF.h>
#include "i2cDevices.h"



void activateI2C1()
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
    i2cInitI2C(i2c, IC2_DUTY_CYCLE_16_9, 15, I2C_CLOCK_200);
    i2cEnableDevice(i2c);                        // MCAL I2C1 activ
}

void activateI2C2()
{
	I2C_TypeDef   *i2c2  = I2C2;
	GPIO_TypeDef  *portB = GPIOB;
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
    i2cInitI2C(i2c2, IC2_DUTY_CYCLE_16_9, 15, I2C_CLOCK_200);
    i2cEnableDevice(i2c2);                        // MCAL I2C2 activ
}
/*
void setBMA020_Shadow(I2C_TypeDef *i2c, uint8_t ShadowDis)										// Ändern des Shadowbits (Genauigkeit)
{
	static uint8_t _shadow_dis = 0;
	uint8_t daten, ret;
	i2cReadByteFromSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR1, &ret);

	if (ShadowDis)																// niedrige Genauigkeit (8 Bit)
	{
		daten = (ret & 0b11110111);
		_shadow_dis = 0;
	}
	else																	// hohe Genauigkeit (10 Bit)
	{
		daten = (ret | BMA020_SHADW_DIS);
		_shadow_dis = 1;
	}
	i2cSendByteToSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR1,   daten);
}


int8_t i2cLIS3DH_presCheck(I2C_TypeDef *i2c)
{
	uint8_t ret;

	i2cReadByteFromSlaveReg(i2c, i2cAddr_LIS3DH, DummyReg, &ret);
	//spiReadRegBurst(spi, LIS3DH_CS_PORT, LIS3DH_CS, (DummyReg|spiRead), ret, 2);
	if (dummyVal == ret)
	{return 1;}
	else
	{return 0;}

}


int8_t LIS3DHINIT(I2C_TypeDef *i2c, int8_t restart)
{
	#define stepStart -7
	static int8_t step = stepStart ;

	if ((restart != 0) && (step == stepStart))  {step = stepStart+1;}
	switch (step)
	{
		case -7:
		{
			//i2cResetDevice(i2c);
			//i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_400);

			i2cSetClkSpd(i2c,  I2C_CLOCK_1Mz); //set I2C Clock 1000kHz fast Mode
			//i2cEnableDevice(i2c);
			step = -6;
			break;
		}
		case -6:
		{
			if (1 == i2cLIS3DH_presCheck(i2c))
			{
				step = -5;
			}
			else
			{
				return 1;		//1 = failure
			}
			break;
		}
		case -5:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg5,   0b10000000); // reboot memory content
			step = -4;
			break;
		}
		case -4:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg4,   0b10001100); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
			step = -3;
			break;
		}
		case -3:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg1,  (uint8_t) 0b10010111); //
			step = -2;
			break;
		}
		case -2:
		{
			uint8_t LIS3DH_Reg4 = 0b10001000;			// BDU BLE FS1 FS0 HR ST1 ST0 SIM
#ifdef XYZopt
//			LIS3DH_Reg4 |=        0b01000000;			// enable to switch High and Low Byte order. special Hint: High Byte first for a fast read sequence to array of int16
#endif
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg4,  LIS3DH_Reg4); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
			step = -1;
			break;
		}
		case -1:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, ADCConfReg,(uint8_t) 0b11000000); // ADC Temp 0 0 0 0 0 0

			step = 0;
			break;
		}
		default:
		{
			step = -7;
		}
	}
	return step;

}


*/


