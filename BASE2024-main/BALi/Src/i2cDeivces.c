/*
 * I2C.c
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <mcalSysTick.h>
#include "i2cDevices.h"




    //RFID SL018

	uint8_t 	*strFirmware = (uint8_t *) " ....        \0";
	uint8_t  RFIDcmd_LEDon[3] = {0x02, 0x40, 0xFF};
	uint8_t  RFIDcmd_LEDoff[3] = {0x02, 0x40, 0x00};
	uint8_t  RFIDcmd_getFirmwareVersion[2] = {0x01, 0xF0};
	uint8_t  RFIDcmd_getMifareUID[2] = {0x01, 0x01};

	bool enableRFID = false;




	// LIDAR
	bool enableLIDAR = false;

uint8_t *convDecByteToHex(uint8_t byte)
{
    static  uint8_t hex[2] = { 0 };

    uint8_t temp;

    temp = byte % 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[1] = temp;

    temp = byte / 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[0] = temp;

    return hex;
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








void RFID_LED(I2C_TypeDef *i2c, bool LEDon)
{
	if (LEDon == 1)
	{
		i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_LEDon, 3);
	}
	else
	{
		i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_LEDoff, 3);
	}
}

int8_t RFID_readCard(I2C_TypeDef *i2c, char *CardID)
{
	static  uint8_t step = 1;
	uint8_t readBuffer[14];
	uint8_t len, i, j = 0;
	int8_t typeCard = -1;
	static int8_t RFID_Status;
	char *p_out;

	switch (step)
	{
		case 1:
		{
			i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_getMifareUID, 2);
			step = 2;
			break;
		}
		case 2:
		{
			i2cBurstRead(i2c, i2cAddr_RFID, readBuffer, 0xC);
			len = readBuffer[0]-2;
			step = 1;
			if (RFID_Status != readBuffer[2])
			{
				typeCard = readBuffer[len];
				j = 0;
				for (i = 0; i< 8; i++ )
				{
					if (i < len)
					{
						if (i == len-1) { CardID[j++]='-'; }					// add the Type number after this  -
						p_out =	(char *) convDecByteToHex(readBuffer[i+3]);
						CardID[j++]  = (char)*(p_out++);
						CardID[j++] =(char)*p_out;
					}
					else
					{CardID[j++]='.'; CardID[j++]='.';}

				}
				CardID[j]='.';
				RFID_Status = readBuffer[2];
			}
			break;
		}
		default:
		{
			step = 1;
		}
	}
	return typeCard;
}

int8_t RFID_readFWVersion(I2C_TypeDef *i2c, char *strFirmware)
{
	static  uint8_t step = 1;
		uint8_t readBuffer[16];
		uint8_t i, len;
		int8_t status = -1;

		switch (step)
		{
			case 1:
			{
				RFID_LED(i2c,0);
				step = 2;
				break;
			}
			case 2:
			{
				i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_getFirmwareVersion, 2);
				step = 3;
				break;
			}
			case 3:
			{
				i2cBurstRead(i2c, i2cAddr_RFID, readBuffer, 0xF);
				len = readBuffer[0];
				status = readBuffer[2];
				step = 4;
				for (i = 0; i<= len; i++ )
				{
				 strFirmware[i] = readBuffer[i+3];
				}

				break;
			}
			case 4:
			{
				RFID_LED(i2c,0);
				step = 1;
				break;
			}

			default:
			{
				step = 1;
			}
		}
		return status;
}




// Tiefpassfilterung der drei Richtungsvektoren xyz
void low_pass(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp)
{
	static long _sto_xyz[3];
	uint8_t i;
	for (i=1;i<=3;i++)
	{
  	 	_sto_xyz[i] += (long) raw_data[i] - (filt_data[i] = _sto_xyz[i]/_tp);
	}
}

