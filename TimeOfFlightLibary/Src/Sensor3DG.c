/*
 * 3DG_Sensor.c
 *
 *  Created on: 01.03.2024
 *      Author: jakob, micha
 * edited on: 30.3.24  Tobias Flämig 
 * released for BMA020
 */

#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <mcalSysTick.h>
#include <math.h>

#include <Sensor3DG.h>

//Kommentar entfernen um Ausgabe von tftprint einzuschalten
//#define debugdisplay


//#define I2Crepeat 	0x80




// Definition der Registeradressen für jeden Sensor
const AccelRegisters registerAddresses[] = {
    // SENSOR_NONE
    {
        .ACCEL_Burst_Reg = 0x00,
        .ACCEL_XOUT_H = 0x00,
        .ACCEL_XOUT_L = 0x00,
        .ACCEL_YOUT_H = 0x00,
        .ACCEL_YOUT_L = 0x00,
        .ACCEL_ZOUT_H = 0x00,
        .ACCEL_ZOUT_L = 0x00
    },
    // SENSOR_BMA020
    {
        .ACCEL_Burst_Reg = 0x82,
        .ACCEL_XOUT_H = 0x03,
        .ACCEL_XOUT_L = 0x02,
        .ACCEL_YOUT_H = 0x05,
        .ACCEL_YOUT_L = 0x04,
        .ACCEL_ZOUT_H = 0x07,
        .ACCEL_ZOUT_L = 0x06
    },
    // SENSOR_MPU6050
    {
        .ACCEL_Burst_Reg = 0x3B,
        .ACCEL_XOUT_H = 0x3B,
        .ACCEL_XOUT_L = 0x3C,
        .ACCEL_YOUT_H = 0x3D,
        .ACCEL_YOUT_L = 0x3E,
        .ACCEL_ZOUT_H = 0x3F,
        .ACCEL_ZOUT_L = 0x40
    },
    // SENSOR_LIS3DH
    {
        .ACCEL_Burst_Reg = 0x28,//eventuell |0x80
        .ACCEL_XOUT_H = 0x29,
        .ACCEL_XOUT_L = 0x28,
        .ACCEL_YOUT_H = 0x2B,
        .ACCEL_YOUT_L = 0x2A,
        .ACCEL_ZOUT_H = 0x2D,
        .ACCEL_ZOUT_L = 0x2C
    }
};	//AccelRegisters registerAddresses[]

// Array der Gyroskopregisteradressen für jeden Sensor
const GyroRegisters gyroRegisters[] = {
    // SENSOR_NONE
    {
        .GYRO_XOUT_H = 0x00,
        .GYRO_XOUT_L = 0x00,
        .GYRO_YOUT_H = 0x00,
        .GYRO_YOUT_L = 0x00,
        .GYRO_ZOUT_H = 0x00,
        .GYRO_ZOUT_L = 0x00
    },
    // SENSOR_BMA
    {
        .GYRO_XOUT_H = 0x00,
        .GYRO_XOUT_L = 0x00,
        .GYRO_YOUT_H = 0x00,
        .GYRO_YOUT_L = 0x00,
        .GYRO_ZOUT_H = 0x00,
        .GYRO_ZOUT_L = 0x00
    },
    // SENSOR_MPU6050
    {
        .GYRO_XOUT_H = 0x43,
        .GYRO_XOUT_L = 0x44,
        .GYRO_YOUT_H = 0x45,
        .GYRO_YOUT_L = 0x46,
        .GYRO_ZOUT_H = 0x47,
        .GYRO_ZOUT_L = 0x48
    },
    // SENSOR_LIS3DH
    {
        .GYRO_XOUT_H = 0x00,
        .GYRO_XOUT_L = 0x00,
        .GYRO_YOUT_H = 0x00,
        .GYRO_YOUT_L = 0x00,
        .GYRO_ZOUT_H = 0x00,
        .GYRO_ZOUT_L = 0x00
    }
};	//GyroRegisters gyroRegisters[]

// Array der Temperaturregisteradressen für jeden Sensor
const TempRegisters tempRegisters[] = {
    // SENSOR_NONE
    {
        .TEMP_OUT_H = 0x00,
        .TEMP_OUT_L = 0x00
    },
    // SENSOR_BMA
    {
        .TEMP_OUT_H = 0x00,
        .TEMP_OUT_L = 0x00
    },
    // SENSOR_MPU6050
    {
        .TEMP_OUT_H = 0x41,
        .TEMP_OUT_L = 0x42
    },
    // SENSOR_LIS3DH
    {
        .TEMP_OUT_H = 0x0D,	//Channel 3 of the ADC can be connected to the temperature sensor
        .TEMP_OUT_L = 0x0C	//by setting the TEMP_EN bit (bit 6) to 1 in TEMP_CFG_REG (1Fh)
    }
};	//TempRegisters tempRegisters[]

const uint8_t i2cAddr_Sensor[] = {
    0x00,   // SENSOR_NONE
    0x38,   // SENSOR_BMA020
    0x68,   // SENSOR_MPU6050
    0x18    // SENSOR_LIS3DH
};

const uint8_t DummyReg[] = {
    0x00,   // SENSOR_NONE
    0x00,   // SENSOR_BMA020
    0x75,   // SENSOR_MPU6050
    0x0F    // SENSOR_LIS3DH
};

const uint8_t dummyVal[] = {
    0x00,   // SENSOR_NONE
    0x02,   // SENSOR_BMA020
    0x68,   // SENSOR_MPU6050
    0x33    // SENSOR_LIS3DH
};

const int8_t stepStart[] = {
    0,      // SENSOR_NONE
    -4,     // SENSOR_BMA020
    -5,     // SENSOR_MPU6050
    -7      // SENSOR_LIS3DH
};
const uint16_t sensorScale[] = {
    // 0,                // SENSOR_NONE (Sie können 0 belassen oder entfernen)
    256 * 0b1000000,    // SENSOR_BMA020
    16384,              // SENSOR_MPU6050
    0x3FFF              // SENSOR_LIS3DH
};

bool enable3DGSensor = false;
AccelerometerSensor currentSensor = SENSOR_NONE;
#define BMA020precision 1



int8_t sensor_presCheck(I2C_TypeDef *i2c)
{
	uint8_t ret = 0;

	i2cReadByteFromSlaveReg(i2c, i2cAddr_Sensor[currentSensor], DummyReg[currentSensor], &ret);
	//spiReadRegBurst(spi, LIS3DH_CS_PORT, LIS3DH_CS, (DummyReg|spiRead), ret, 2);
	if (dummyVal[currentSensor] == ret)
	{
		return 1;
	}
	else
	{
		return 0;
	}
	return 1;
}

int8_t sensor_init(I2C_TypeDef *i2c, int8_t restart)
{
	int8_t returnValue = 0; // Standardrückgabewert

	if(currentSensor == SENSOR_BMA020)
		{
			returnValue = i2cBMA020_init(i2c, restart);
		}
	if(currentSensor == SENSOR_LIS3DH)
		{
		returnValue = i2cLIS3DH_init(i2c, restart);
		}
	if(currentSensor == SENSOR_MPU6050)
		{
		returnValue = i2cMPU6050_init(i2c, restart);
		}
	return returnValue; // Rückgabewert zurückgeben
}

void setBMA020_Bandwidth(I2C_TypeDef *i2c, uint8_t Bandwidth)		// Ändern der Bandbreite (Bandwidth) 25 50 750 1500
{
	uint8_t daten;
	i2cReadByteFromSlaveReg(i2c, i2cAddr_Sensor[SENSOR_BMA020], BMA020_CR2, &daten);

	daten = (daten & 0b11111000);
	daten = (daten |(uint8_t) Bandwidth);

	i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_BMA020], BMA020_CR2, daten);
}


uint8_t setBMA020_Range(I2C_TypeDef *i2c, uint8_t Range)							// Ändern der Empfindlichkeit (Range) 2 4 8g
{
	static uint8_t _range;
	uint8_t daten;
	if (Range == 0)
	{
		return _range;
	}
	else
	{
		i2cReadByteFromSlaveReg(i2c, i2cAddr_Sensor[SENSOR_BMA020], BMA020_CR2, &daten);

		switch (Range)
		{
			case 2:															// Empfindlichkeit: +/- 2g
			{
				daten &= 0b11100111;
				_range = 2;
			}
			break;
			case 4:															// Empfindlichkeit: +/- 4g
			{
				daten &= 0b11101111;
				daten |= 0b00001000;
				_range = 4;
			}
			break;
			case 8:															// Empfindlichkeit: +/- 8g
			{
				daten &= 0b11110111;
				daten |= 0b00010000;
				_range = 8;
			}
			break;
			default:							// nur Rückgabe des aktuell eingestellten Messbereichs
			{
				return _range;
			}
		}
		i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_BMA020], BMA020_CR2, daten);
	}
	return _range;
}


int8_t i2cBMA020_init(I2C_TypeDef *i2c, int8_t restart)
{
	static int8_t step = stepStart[SENSOR_BMA020];


	if ((restart != 0) && (step == stepStart[SENSOR_BMA020]))
	{
		step = stepStart[SENSOR_BMA020] + 1;
	}


	switch (step)
	{
		case -4:		// CLK Speed von I2C auf Fast Mode
		{
			i2cSetClkSpd(i2c, I2C_CLOCK_1Mz); //set I2C Clock 1000kHz fast Mode
			step = -2;
		}
		break;
		case -3:
		{		// SW Reset
			i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_BMA020], BMA020_CR3, BMA020_SWRESET); // reboot memory content
			step = -2;
		}
		break;
		case -2:
		{
			setBMA020_Range(i2c, 4);			// Set Range +-4 g
			step = -1;
		}
		break;
		case -1:
		{		// Config Range & Bandwidth
			setBMA020_Bandwidth(i2c, BMA020_ACCEL_BANDW_50);
			//i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_BMA020], BMA020_CR2, (BMA020_ACCEL_RANGE_2g|BMA020_ACCEL_BANDW_190)); // Set Range to ±2g, Set Filter bandwidth to 190Hz
			step = 0;
		}
		break;
		default:
		{
			step = -4;
		}
	}
	return step;
}

int16_t i2cBMA020XYZ(I2C_TypeDef *i2c, int16_t *xyz)
{
	uint8_t _rdata[6];
	int16_t schiebeReg;
	i2cBurstRegRead(i2c, i2cAddr_Sensor[SENSOR_BMA020], BMA020_DR, _rdata, 6);

	float skal_div = 1/4.088;															// Zwischenspeicher für Bearbeitung der dez. Achsdaten
	int i;
	uint8_t range = setBMA020_Range(i2c, 0);
	switch (range)													// Skalierungsfaktoren für den Messberich +/-(2,4,8g) ermitteln
	{
	case 2:
		{
		skal_div =1/16.352;
		break;
		}
	case 4:
		{
		skal_div = 1/8.176;
		}
	}

	for (i=0;i<=2;i++)
	{

		schiebeReg = _rdata[2*i+1];									// MSB in Schieberegister schreiben
		schiebeReg <<= 8;											// Schieben an die ersten 8 Bit

		if(BMA020precision)
		{
			schiebeReg += _rdata[2*i];										// Addition des jeweiligen LSB zum MSB
			schiebeReg &= 0xFFC0; //b1111111111000000; 					// Abschneiden der geringeren Stellen, da diese keine Daten enthalten
		}
		xyz[i] = (int16_t)(((int16_t)schiebeReg) * skal_div);									// Konvertieren in Integer-Format und Messbereich einrechnen (2,4,8g) einberechnen
	}

	return 0;
}


int8_t i2cLIS3DH_init(I2C_TypeDef *i2c, int8_t restart)
{
	int8_t step = stepStart[SENSOR_LIS3DH];

	if ((restart != 0) && (step == stepStart[SENSOR_LIS3DH]))
	{
		step = stepStart[SENSOR_LIS3DH] + 1;
	}
	switch (step)
	{
	case -7:		// CLK Speed von I2C auf Fast Mode
	{
		//i2cResetDevice(i2c);
		//i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_400);

		i2cSetClkSpd(i2c, I2C_CLOCK_1Mz); //set I2C Clock 1000kHz fast Mode
		//i2cEnableDevice(i2c);
		step = -6;
		break;
	}
	case -6:		//
	{
		if (1 == sensor_presCheck(i2c))
		{
			step = -5;
		} else
		{
			return 1;		//1 = failure
		}
		break;
	}
	case -5:
	{
		i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[currentSensor], LIS3DH_CtrlReg5, 0b10000000); // reboot memory content
		step = -4;
		break;
	}
	case -4:
	{
		i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[currentSensor], LIS3DH_CtrlReg4, 0b10001100); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
		step = -3;
		break;
	}
	case -3:
	{
		i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[currentSensor], LIS3DH_CtrlReg1,
				(uint8_t) 0b10010111); //
		step = -2;
		break;
	}
	case -2:
	{
		uint8_t LIS3DH_Reg4 = 0b10001000;	// BDU BLE FS1 FS0 HR ST1 ST0 SIM
#ifdef XYZopt
//			LIS3DH_Reg4 |=        0b01000000;			// enable to switch High and Low Byte order. special Hint: High Byte first for a fast read sequence to array of int16
#endif
		i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[currentSensor], LIS3DH_CtrlReg4, LIS3DH_Reg4); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
		step = -1;
		break;
	}
	case -1:
	{
		i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[currentSensor], LIS3DH_ADCConfReg,
				(uint8_t) 0b11000000); // ADC Temp 0 0 0 0 0 0

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


int16_t i2cLIS3DH_XYZ(I2C_TypeDef *i2c, int16_t *xyz)
{
#ifdef XYZopt
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (OUTxyz|I2Crepeat),(uint8_t *) xyz, 6);
#else
	uint8_t readBuffer[6];
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (OUTxyz|I2Crepeat),readBuffer, 6);
	*xyz = (readBuffer[1]<<8) + readBuffer[0];
	xyz++;
	*xyz = (readBuffer[3]<<8) + readBuffer[2];
	xyz++;
	*xyz = (readBuffer[5]<<8) + readBuffer[4];
#endif
	return 0;
}

/**
- enable the bits 6 and 7 in TEMP_CFG_REG(1Fh): enable aux ADC and enable temperature sensor.
- enable Block Data Update, BDU, feature. CTRL_REG4(0x23) , BDU (bit#7)=1.
- read both the ADC output 3 registers (because of BDU enabled): OUT_ADC_3_L(0x0C) and OUT_ADC_3_H(0x0D).
Useful bits: 8, left aligned, hence useful data in OUT_ADC_3_H.
Temperature sensor output change vs temperature: 1digit/degrCelsius
*/
int8_t i2cLIS3DH_Temp(I2C_TypeDef *i2c)
{
	uint8_t readBuffer[2];
	//i2cReadByteFromSlaveReg(i2c, i2cAddr_LIS3DH, (ADC3Temp+1), (uint8_t *) &readBuffer[1]);  	// only Hihg byte with data
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (ADC3Temp|I2Crepeat), readBuffer, 2);					// but BDU =1 required for update data to read both, high and low Byte
	return ((int8_t) readBuffer[1]);					// send a delta Temp Value
}


/** set Low PassFilter DLPF Reg 0x1A
 *
 * Bit7 Bit6 	Bit5 Bit4 Bit3 		Bit2 Bit1 Bit0
 * - 	- 		EXT_SYNC_SET[2:0] 		DLPF_CFG[2:0]
 *  Register Map Doku page  13
 *  EXT_SYNC_SET: 	0  Input disabled
 *  					BWAccl/Hz	DelayAccl/ms		BWGyro/Hz	DelayGyro/ms
 *  DLPF_CFG[2:0]: 	0	260 		0					256			1
 *  				1	184			2					166			2
 *  				2	94			3					98			3
 *  				3	44			5					42			5
 *
 */
void i2cMPU6050LpFilt(I2C_TypeDef *i2c, uint8_t DLPF_CFG)
{
	i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_CONFIG, (0b0111 & DLPF_CFG));
}


int8_t i2cMPU6050_init(I2C_TypeDef *i2c, int8_t restart)
{
	static int8_t step = stepStart[SENSOR_MPU6050];
	if ((restart != 0) && (step == stepStart[SENSOR_MPU6050]))
	{
		step = stepStart[SENSOR_MPU6050] + 1;
	}
	switch (step)
		{

			case -6:		// CLK Speed von I2C auf 400kHz
			{
				i2cSetClkSpd(i2c, I2C_CLOCK_1Mz); //set I2C Clock 1000kHz
				step = -5;
			}
			break;
			case -5:		// SW Reset
			{
				i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_PWR_MGMT_1, (MPU6050_SWRESET)); // reboot memory content
				step = -4;
			}
			break;
			case -4:
			{		// PWR Mngt
				i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_PWR_MGMT_1, (MPU6050_PWR1_TEMP_dis|MPU6050_PWR1_CLKSEL));
				i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_PWR_MGMT_2, (MPU6050_PWR2_ACConXY_GYonZ));
				step = -3;
			}
			break;
			case -3:
			{		// GYRO Config
				i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_GYRO_CONFIG, MPU6050_GYRO_FSCALE_250); 	// full scale range of gyroscope = ± 250 °/s
				step = -2;
			}
			break;
			case -2:
			{		// ACCEL Config
				i2cSendByteToSlaveReg(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_RANGE_2); 	// full scale range of accelerometer = ± 2g
				step = -1;
			}
			break;
			case -1:
			{		// LowPass Config
				i2cMPU6050LpFilt(i2c, 4);
				step = 0;
			}
			break;

			default:
			{
				step = stepStart[SENSOR_MPU6050];
			}
		}
	return step;
}

int16_t i2cMPU6050XYZ(I2C_TypeDef *i2c, int16_t *xyz)
{
	//i2cBurstRegRead(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_AccXYZ,(uint8_t *) xyz, 6);
	uint8_t readBuffer[6];
	i2cBurstRegRead(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_AccXYZ,readBuffer, 6);
	*xyz = (readBuffer[0]<<8) + readBuffer[1];
	xyz++;
	*xyz = (readBuffer[2]<<8) + readBuffer[3];
	xyz++;
	*xyz = (readBuffer[4]<<8) + readBuffer[5];
	return 0;
}
int16_t i2cMPU6050GYRO(I2C_TypeDef *i2c, int16_t *xyz)
{
	uint8_t readBuffer[6];
	i2cBurstRegRead(i2c, i2cAddr_Sensor[SENSOR_MPU6050], MPU6050_GyroXYZ,readBuffer, 6);
	*xyz = (readBuffer[0]<<8) + readBuffer[1];
	xyz++;
	*xyz = (readBuffer[2]<<8) + readBuffer[3];
	xyz++;
	*xyz = (readBuffer[4]<<8) + readBuffer[5];
	return 0;
}

void getAngleFromAcc(int16_t *xyz, float *AlphaBeta) {

	float X = (float) xyz[0]/160;
	float Y = (float) xyz[1]/160;
	float Z = (float) xyz[2]/160;

	AlphaBeta[0] = atan(X / Z);
	if (Z < 0) {
		if (X < 0) {
			AlphaBeta[0] -= _pi;
		} else {
			AlphaBeta[0] += _pi;
		}
	}

	AlphaBeta[1] = atan(Y / Z);
	if (Z < 0) {
		if (Y < 0) {
			AlphaBeta[1] -= _pi;
		} else {
			AlphaBeta[1] += _pi;
		}
	}
}

void getFiltertAccData(int16_t *XYZ, float *XYZFiltert, float kFilt)
{
	uint8_t i;
	for (i = 0; i<3; i++)
	{
		XYZFiltert[i] *= (1-kFilt);
		XYZFiltert[i] += ((float)XYZ[i] * kFilt);
	}
}
void getAccData(I2C_TypeDef *i2c, int16_t *xyz)
{
	uint8_t readBuffer[6];
	uint8_t regAddress = registerAddresses[currentSensor].ACCEL_Burst_Reg;

	i2cBurstRegRead(i2c, i2cAddr_Sensor[currentSensor], regAddress, readBuffer, 6);
	// Die Daten werden dann aus dem lokalen Pufferarray in das xyz-Array konvertiert und gespeichert.

	switch (currentSensor)
	{
		case SENSOR_BMA020:
		{
		/*
		 *  Array			Achse - Bit
		 * readBuffer[0]	X-1:0
		 * readBuffer[1]	X-9:2
		 * readBuffer[2]	Y-1:0
		 * readBuffer[3]	Y-9:2
		 * readBuffer[4]	Z-1:0
		 * readBuffer[5]	Z-9:2
		 */

			*xyz = (readBuffer[1]<<8) + readBuffer[0]; // Zuerst wird das Low-Byte (readBuffer[0]) mit dem High-Byte (readBuffer[1]) kombiniert.
			xyz++; // Der Zeiger wird auf das nächste Element im Array verschoben.

			*xyz =(readBuffer[3]<<8) + readBuffer[2]; // Gleiches Verfahren für das nächste Datenpaar (Y-Daten).
			xyz++; // Zeiger wird verschoben.

				*xyz = (readBuffer[5]<<8) + readBuffer[4]; // Gleiches Verfahren für das letzte Datenpaar (Z-Daten).
		}
		break;

		case SENSOR_MPU6050:
		{
		/*
		 * Array			Achse - Bit
		 * readBuffer[0]	X-15:8
		 * readBuffer[1]	X-7:0
		 * readBuffer[2]	Y-15:8
		 * readBuffer[3]	Y-7:0
		 * readBuffer[4]	Z-15:8
		 * readBuffer[5]	Z-7:0
		 */

			*xyz = (readBuffer[0]<<8) + readBuffer[1]; // Zuerst wird das Low-Byte (readBuffer[1]) mit dem High-Byte (readBuffer[0]) kombiniert.
			xyz++; // Der Zeiger wird auf das nächste Element im Array verschoben.

			*xyz = (readBuffer[2]<<8) + readBuffer[3]; // Gleiches Verfahren für das nächste Datenpaar (Y-Daten).
			xyz++; // Zeiger wird verschoben.

			*xyz = (readBuffer[4]<<8) + readBuffer[5]; // Gleiches Verfahren für das letzte Datenpaar (Z-Daten).
		}
		break;
		case SENSOR_LIS3DH:
		{

			*xyz = (readBuffer[1]<<8) + readBuffer[0]; // Zuerst wird das Low-Byte (readBuffer[0]) mit dem High-Byte (readBuffer[1]) kombiniert.
			xyz++; // Der Zeiger wird auf das nächste Element im Array verschoben.

			*xyz = (readBuffer[3]<<8) + readBuffer[2]; // Gleiches Verfahren für das nächste Datenpaar (Y-Daten).
			xyz++; // Zeiger wird verschoben.

			*xyz = (readBuffer[5]<<8) + readBuffer[4]; // Gleiches Verfahren für das letzte Datenpaar (Z-Daten).
		}
		break;
		case SENSOR_NONE:
		{
		}

	}
}
