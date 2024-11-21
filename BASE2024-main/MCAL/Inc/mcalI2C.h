/**
 * mcalI2C.h
 *
 *  Created on: Sep 22, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALI2C_H_
#define MCALI2C_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup iic4
 * @{
 */

/**
 * @note
 * The following definitions are not yet usable.
 */
//#define I2C_OK                      (0)
//#define I2C_TIMEOUT_FAILURE         (-1)
//#define I2C_SCL_FREQ_NOT_SUPPORTED  (-2)

typedef enum
{
    I2C_OK                       =  0,
    I2C_INVALID_TYPE             = -60,
    I2C_INVALID_DUTY_CYCLE       = -61,
    I2C_INVALID_CLOCK_SPEED      = -62,
    I2C_INVALID_PERIPHERAL_CLOCK = -63,
    I2C_INVALID_RISE_TIME
} I2C_RETURN_CODE_t;

/**
 * @brief I2C enumerations
 */
typedef enum
{
	I2C_CLOCK_50  = 0,
	I2C_CLOCK_100, 	      // SCL = 100 kHz
	I2C_CLOCK_200,
    I2C_CLOCK_400,           // SCL = 400 kHz
	I2C_CLOCK_1Mz			// 1000kHz _ Fast Mode
} I2C_CLOCKSPEED_t;

typedef enum
{
    I2C_DUTY_CYCLE_2            = 0,
    IC2_DUTY_CYCLE_16_9
} I2C_DUTY_CYCLE_t;

/**
 * @}
 */






/**
 * Function prototypes
 */
// Deprecated functions
extern I2C_RETURN_CODE_t i2cInitI2C(I2C_TypeDef *i2c, I2C_DUTY_CYCLE_t duty, uint8_t trise, I2C_CLOCKSPEED_t clock);

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


#ifdef __cplusplus
}
#endif



#endif /* MCALI2C_H_ */
