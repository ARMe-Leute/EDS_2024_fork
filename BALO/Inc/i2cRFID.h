/*
 * I2CRFID.h
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

#ifndef I2CRFID_H_
#define I2CRFID_H_


extern uint8_t *convDecByteToHex(uint8_t byte);
extern void i2c_activate_pb89(I2C_TypeDef *i2c);

/* RFID SL018 Function
 *
 *
 *
 */
#define i2cAddr_RFID	0x50
extern bool enableRFID;

extern void RFID_LED(I2C_TypeDef *i2c, bool LEDon);
extern int8_t RFID_readCard(I2C_TypeDef *i2c, char *CardID);
extern int8_t RFID_readFWVersion(I2C_TypeDef *i2c, char *strFirmware);


#define _OK 1

#endif /* I2CRFID_H_ */



