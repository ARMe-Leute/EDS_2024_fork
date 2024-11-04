/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Andreas Ladner & Philipp Roehlke
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
#include <mcalI2C.h>

// General functions
extern I2C_RETURN_CODE_t i2cSelectI2C(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cDeselectI2C(I2C_TypeDef *i2c);

extern I2C_RETURN_CODE_t i2cSetClkSpd(I2C_TypeDef *i2c, I2C_CLOCKSPEED_t spd);
extern I2C_RETURN_CODE_t i2cEnableDevice(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cDisableDevice(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cSetPeripheralClockFreq(I2C_TypeDef *i2c, uint8_t pclk);
extern uint32_t          i2cGetPeripheralClockFrequ(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cSetDutyCycle(I2C_TypeDef *i2c, I2C_DUTY_CYCLE_t duty);
extern I2C_RETURN_CODE_t i2cSetRiseTime(I2C_TypeDef *i2c, uint8_t riseTime);

// Send functions
extern I2C_RETURN_CODE_t i2cSendByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t data);
extern I2C_RETURN_CODE_t i2cSendByteToSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t data);
extern I2C_RETURN_CODE_t i2cBurstWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data, uint8_t numBytes);

// Read functions
extern I2C_RETURN_CODE_t i2cReadByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data);
extern I2C_RETURN_CODE_t i2cReadByteFromSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data);
extern I2C_RETURN_CODE_t i2cBurstRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data, uint8_t num);
extern I2C_RETURN_CODE_t i2cBurstRegRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t num);

extern I2C_RETURN_CODE_t i2cResetDevice(I2C_TypeDef *i2c);
extern uint8_t           i2cFindSlaveAddr(I2C_TypeDef *i2c, uint8_t i2cAddr);
