/*
 * I2C.h
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

#ifndef I2C_H_
#define I2C_H_



extern uint8_t *convDecByteToHex(uint8_t byte);
extern void i2c_activate_pb89(I2C_TypeDef *i2c);

/* RFID SL018 Function
 *
 *
 *
 */
#define i2cAddr_RFID	0x50

extern void RFID_LED(I2C_TypeDef *i2c, bool LEDon);
extern int8_t RFID_readCard(I2C_TypeDef *i2c, char *CardID);
extern int8_t RFID_readFWVersion(I2C_TypeDef *i2c, char *strFirmware);

#define i2cAddr_LIS3DH 0x18
extern int8_t i2cLIS3DH_init(I2C_TypeDef *i2c, int8_t restart);
extern int8_t i2cLIS3DH_Temp(I2C_TypeDef *i2c);
extern int16_t i2cLIS3DH_XYZ(I2C_TypeDef *i2c, int16_t *xyz);

#define i2cAddr_LIDAR	0x29


extern bool enableLIS3DH;
extern bool enableRFID;
extern bool enableLIDAR;
#endif /* I2C_H_ */
