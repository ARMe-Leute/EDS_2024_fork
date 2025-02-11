/*
 * I2C.h
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

#ifndef I2CDEVICES_H_
#define I2CDEVICES_H_



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

#define _OK 1


extern void low_pass(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp);




#define i2cAddr_LIDAR	0x29


extern bool enableRFID;
extern bool enableLIDAR;
#endif /* I2CDEVICES_H_ */



