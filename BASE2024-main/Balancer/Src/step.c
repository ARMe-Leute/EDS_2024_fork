/**
 *      step.c
 * 
 *      @file step.c provides the methods to control a stepper via the `AMIS-30624`
 *      @author: Stefan Heinrich, Dennis Lotz
 *      Created on: Dez. 05, 2023
 */
#include <mcalGPIO.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <mcalI2C.h>
//#include <i2c.h>

static I2C_TypeDef   *i2c  = I2C1;

static uint8_t statusReg1[8];
static uint8_t statusReg2[8];

// Beschreibung der FUnkion Welcher datentyp und welche Range kommt rein + welcher Datentyp und welcher wertebereich kommt raus


typedef enum { FALSE, TRUE } BOOL;




/**
 * read the data of the first system register of the stepper
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
void getFullStatus1(uint8_t addr)
{
	uint8_t befehl = (uint8_t) 0x81;
	i2cBurstWrite(i2c,addr, &befehl, 1);
	i2cBurstRead(i2c, addr, statusReg1, 8);
}


/**
 * read the data of the second system register of the stepper
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
void getFullStatus2(uint8_t addr)
{
	uint8_t befehl = (uint8_t) 0xFC;
	i2cBurstWrite(i2c,addr, &befehl, 1);
	i2cBurstRead(i2c, addr, statusReg2, 8);
}


/**
 * read out the currently set operating current
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
uint8_t getIrun (uint8_t addr)
{
	uint8_t iRun;

	getFullStatus1(addr);
	iRun = statusReg1[1];
	iRun >>= 4;

	return iRun;
}


/**
 * set the operating current
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t Databyte - 0x0 .. 0xF --> 59mA .. 800mA - for more information view data sheet
 * @returns void
 */
void setIrun(uint8_t addr,uint8_t Databyte)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 from StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc from StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=Databyte;				// save Irun value to MotorParam
	schiebeReg<<=4;
	MotorParam[3]&=0b00001111;
	MotorParam[3]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}


/**
 * read out the currently set holding current
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
uint8_t getIhold (uint8_t addr)
{
	uint8_t iHold;

	getFullStatus1(addr);
	iHold = statusReg1[1];
	iHold &= 0b00001111;

	return iHold;
}


/**
 * set the holding current
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t Databyte - 0x0 .. 0xE, 0xF --> 59mA .. 673mA, 0mA - for more information view data sheet
 * @returns void
 */
void setIhold(uint8_t addr,uint8_t Databyte)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 from StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc from StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=Databyte;				// write Ihold into MotorParam value
	schiebeReg&=0b00001111;
	MotorParam[3]&=0b11110000;
	MotorParam[3]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}


/**
 * get the maximum speed
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns vMax - 0x0 .. 0xF --> 0 .. 15/32 (stepper ration), actual speed is depending on the steps (fullstep, halfstep, ...) - for more information view data sheet
 */
uint8_t getVmax (uint8_t addr)
{
	uint8_t vMax;

	getFullStatus1(addr);
	vMax = statusReg1[2];
	vMax >>= 4;

	return vMax;
}


/**
 * set the maximum speed
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t vMax - 0x0 .. 0xF --> 0 .. 15/32 (stepper ration), actual speed is depending on the steps (fullstep, halfstep, ...) - for more information view data sheet
 * @returns void
 */
void setVmax (uint8_t addr, uint8_t vMax)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 aus StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc aus StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=vMax;					// write Vmax into MotorParam value
	schiebeReg<<=4;
	MotorParam[4]&=0b00001111;
	MotorParam[4]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}


/**
 * get the currently set minimum speed
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
uint8_t getVmin (uint8_t addr)
{
	uint8_t vMin;

	getFullStatus1(addr);
	vMin = statusReg1[2];
	vMin &= 0b00001111;

	return vMin;
}


/**
 * set the minimum speed
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t vMin - 0x0 .. 0xF --> 0 .. 15/32 (stepper ration), actual speed is depending on the steps (fullstep, halfstep, ...) - for more information view data sheet
 * @returns void
 */
void setVmin (uint8_t addr, uint8_t vMin)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 aus StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc aus StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=vMin;					// write Vmin into MotorParam value
	schiebeReg&=0b00001111;
	MotorParam[4]&=0b11110000;
	MotorParam[4]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}

/**
 * get the currently set rotation direction cw / ccw
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns uint8_t shaft - returns 0 or 1
 */
uint8_t getShaft(uint8_t addr)
{
	uint8_t shaft;

	getFullStatus1(addr);
	shaft= statusReg1[3];
	shaft >>=4;
	shaft &= 0b00000001;

	return shaft;
}


/**
 * set the set rotation direction cw / ccw
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t Databyte - 0x0 / 0x1, this translates to cw or ccw - for more information view data sheet
 * @returns uint8_t shaft - returns 0 or 1
 */
void setShaft(uint8_t addr, uint8_t Databyte)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 from StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc from StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=Databyte;				// write shaft value into MotorParam value
	schiebeReg&=0b00000001;
	schiebeReg<<=4;
	MotorParam[5]&=0b11101111;
	MotorParam[5]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}


/**
 * get the currently set acceleration
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
uint8_t getAcceleration(uint8_t addr)
{
	uint8_t acceleration;

	getFullStatus1(addr);
	acceleration = statusReg1[3];
	acceleration &= 0b00001111;

	return acceleration;
}


/**
 * set the set acceleration
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t Databyte - 0x0 .. 0xF, 49Full−step/s^2 .. 40047Full−step/s^2 - for more information view data sheet
 * @returns uint8_t shaft - returns 0 or 1
 */
void setAcceleration(uint8_t addr, uint8_t Databyte)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 aus StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc aus StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=Databyte;				// write acceleration into MotorParam value
	schiebeReg&=0b00001111;
	MotorParam[5]&=0b11110000;
	MotorParam[5]+=schiebeReg;


	i2cBurstWrite(i2c, addr,MotorParam,8);
}




/**
 * get the currently set secure position. This is the position to which the motor is driven
 * if a HW pin connection is lost. If `<SecPos[10:2]> = 0x400`, secure positioning is disabled;
 * the stepper−motor will be kept in the position occupied at the moment these events occur.
 * 
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns uint32_t secPos - the secure position of the stepper 
 */
uint32_t getSecurePosition(uint8_t addr)
{
	uint32_t secPos;

	getFullStatus1(addr);
	secPos = statusReg2[6];
	secPos <<= 8;
	secPos +=statusReg2[5];
	secPos&=0b0000011111111111;

	return(secPos);
}


/**
 * set the secure position. This is the position to which the motor is driven
 * if a HW pin connection is lost. If `<SecPos[10:2]> = 0x400`, secure positioning is disabled;
 * the stepper−motor will be kept in the position occupied at the moment these events occur.
 * 
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint32_t Dataword - the secure position of the stepper 
 * @returns void
 */
void setSecurePosition(uint8_t addr, uint32_t Dataword)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint32_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 from StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc from StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=Dataword;				// Secure Position Wert in MotorParam schreiben
	MotorParam[6]=(uint8_t)schiebeReg;
	schiebeReg>>=3;
	schiebeReg&=0b0000000011100000;
	MotorParam[5]&=0b00011111;
	MotorParam[5]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}


/**
 * get the currently set acceleration shape
 * 
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns uint8_t `0`: normal acceleration from Vmin to Vmax, `1`: motion at Vmin without acceleration
 */
uint8_t getAccShape(uint8_t addr)
{
	uint8_t accshape;
	getFullStatus1(addr);
	accshape = statusReg1[3];
	accshape>>=7;

	return accshape;
}


/**
 * set the acceleration shape
 * 
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t Databyte - 0x0 / 0x1, `0`: normal acceleration from Vmin to Vmax, `1`: motion at Vmin without acceleration - for more information view data sheet 
 * @returns void
 */
void setAccShape(uint8_t addr, uint8_t Databyte)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 from StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc from StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=Databyte;				// AccShape Wert in MotorParam schreiben
	schiebeReg&=0b00000001;
	schiebeReg<<=4;
	MotorParam[7]&=0b11101111;
	MotorParam[7]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}


/**
 * get the currently set stepping mode
 *  `0x0`: 1/2  stepping
 *  `0x1`: 1/4  stepping
 *  `0x2`: 1/8  stepping
 *  `0x3`: 1/16 stepping
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns uint8_t stepMode `0x0` .. `0x3`
 */
uint8_t getStepMode(uint8_t addr)
{
	uint8_t stepMode;

	getFullStatus1(addr);
	stepMode = statusReg1[3];
	stepMode >>= 5;
	stepMode &= 0b00000011;

	return stepMode;
}


/**
 * set the new stepping mode
 *  `0x0`: 1/2  stepping
 *  `0x1`: 1/4  stepping
 *  `0x2`: 1/8  stepping
 *  `0x3`: 1/16 stepping
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t StepMode `0x0` .. `0x3` - for more information view data sheet 
 * @returns void
 */
void setStepMode(uint8_t addr, uint8_t StepMode)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];
	uint8_t schiebeReg;

	getFullStatus1(addr);
	getFullStatus2(addr);
	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=statusReg1[1];		// x|y x=Irun y=Ihold
	MotorParam[4]=statusReg1[2];		// x|y x=Vmax y=Vmin

	MotorParam[5]=0b00000000;
	schiebeReg=statusReg2[6];			// xxx|y|zzzz x=SecPosition 10:8 aus StatusReg2
	schiebeReg<<=5;
	MotorParam[5]=schiebeReg;

	schiebeReg=statusReg1[3];			// xxx|y|zzzz y=Shaft und z=Acc aus StatusReg1
	schiebeReg&=0b00011111;
	MotorParam[5]+=schiebeReg;

	MotorParam[6]=statusReg2[5];		// =SecPos(7:0)

	schiebeReg=statusReg1[3];			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	schiebeReg>>=3;
	MotorParam[7]=schiebeReg;

	schiebeReg=StepMode;				// StepMode Wert in MotorParam schreiben
	schiebeReg&=0b00000011;
	schiebeReg<<=2;
	MotorParam[7]&=0b11110011;
	MotorParam[7]+=schiebeReg;


	i2cBurstWrite(i2c,addr,MotorParam,8);
}



/**
 * get the current position of the stepper
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns uint16_t 16bit integer `-30000` .. `+30000`
 */
int16_t getActualPosition(uint8_t addr)
{
	int16_t actualPosition;

	getFullStatus2(addr);
	actualPosition = (int16_t) statusReg2[1];
	actualPosition <<= 8;
	actualPosition += (int16_t) statusReg2[2];

	return actualPosition;
}


/**
 * set the new target position of the stepper
 * 
 * 
 *  Stepping Mode     Position Range       Full Range Excursion   Number of Bits
 * 
 *  1/2  stepping:    −4096 to +4095          8192 half−steps           13
 *  1/4  stepping:    −8192 to +8191         16384 micro−steps          14
 *  1/8  stepping:   −16384 to +16383        32768 micro−steps          15
 *  1/16 stepping:   −32768 to +32767        65536 micro−steps          16
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @param int step - the step the stepper shall go to - for more information view data sheet 
 * @returns void
 */
void setPosition(uint8_t addr, int data)
{
	uint8_t befehl = (uint8_t) 0x8B;
	uint8_t sendData[5];
	uint32_t Register;
	sendData[0]= befehl;
	sendData[1]= 0xFF;
	sendData[2]= 0xFF;

	Register = (uint32_t) data;
	Register >>= 8;
	sendData[3] = (uint8_t) Register;

	Register = (uint32_t) data;
	Register &= 0b0000000011111111;
	sendData[4] = (uint8_t) Register;

	i2cBurstWrite(i2c,addr, sendData,5);
}


/**
 * get the current target position of the stepper
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns uint16_t 16bit integer `-30000` .. `+30000`
 */
int16_t getTargetPosition(uint8_t addr) // falls probleme auftreten zurück zu int als return type
{
	int16_t targetPosition;

	getFullStatus2(addr);
	targetPosition = statusReg2[3];
	targetPosition <<= 8;
	targetPosition += (int16_t) statusReg2[4];

	return targetPosition;
}


/**
 * returns the maximum achievable speed with the currently set Vmax
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns int rpm
 */
uint32_t getRpm(uint8_t addr)
{
//	int vMax[16];
	uint8_t actualVmax;
	uint32_t rpm;
	uint32_t wert;

//	vMax[0]=99;
//	vMax[1]=136;
//	vMax[2]=167;
//	vMax[3]=197;
//	vMax[4]=213;
//	vMax[5]=228;
//	vMax[6]=243;
//	vMax[7]=273;
//	vMax[8]=303;
//	vMax[9]=334;
//	vMax[10]=364;
//	vMax[11]=395;
//	vMax[12]=456;
//	vMax[13]=546;
//	vMax[14]=729;
//	vMax[15]=973;
//
	actualVmax = getVmax(addr);
//
	wert = (uint32_t) actualVmax;
//	rpm  =	vMax[wert];
	rpm = ((((uint32_t) 973) * wert) / ((uint32_t) 15)) + ((uint32_t) 99);
	rpm = rpm * 3.3;

	return rpm;
}


/**
 * This method is called once during initialization.
 * The motor is initially set to these values.
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t rotdir - `TRUE` / `FALSE`
 * @returns void
 */
static void initMotorParam(uint8_t addr, uint8_t rotdir)
{
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t MotorParam[8];

	MotorParam[0]=befehl;
	MotorParam[1]=0xFF;
	MotorParam[2]=0xFF;
	MotorParam[3]=0b11100111;					// xxxx|yyyy x=Irun y=Ihold
	MotorParam[4]=0b11100010;					// xxxx|yyyy x=Vmax y=Vmin
	if (rotdir == (BOOL) 1)
	{
		MotorParam[5]=0b00010011;		// xxx|y|zzzz   x=SecPos(10:8) y=Shaft z=Acc(3:0)
	}
	else
	{
		MotorParam[5]=0b00000011;		// xxx|y|zzzz   x=SecPos(10:8) y=Shaft z=Acc(3:0)
	}
	MotorParam[6]=0b00000000;			// =SecPos(7:0)
	MotorParam[7]=0b00001100;			// xxx|y|zz|xx  x=N/A y=AccShape z=StepMode
	i2cBurstWrite(i2c,addr,MotorParam,8);
}


/**
 * This method is used for initialization, sets Vmax and Vmin, 
 * then lets the motor move first to pos1 then to pos2
 * and then sets the current position to 0.
 * 
 * 
 *  Stepping Mode     Position Range       Full Range Excursion   Number of Bits
 * 
 *  1/2  stepping:    −4096 to +4095          8192 half−steps           13
 *  1/4  stepping:    −8192 to +8191         16384 micro−steps          14
 *  1/8  stepping:   −16384 to +16383        32768 micro−steps          15
 *  1/16 stepping:   −32768 to +32767        65536 micro−steps          16
 *
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @param uint8_t vMax - 0x0 .. 0xF --> 0 .. 15/32 (stepper ration), actual speed is depending on the steps (fullstep, halfstep, ...) - for more information view data sheet
 * @param uint8_t vMin - 0x0 .. 0xF --> 0 .. 15/32 (stepper ration), actual speed is depending on the steps (fullstep, halfstep, ...) - for more information view data sheet
 * @param uint32_t pos1 - the first target step - for more information view data sheet
 * @param uint32_t pos2 - the second target step - for more information view data sheet
 * @returns void
 */
void runInit(uint8_t mot_addr, uint8_t vMax, uint8_t vMin, uint32_t pos1, uint32_t pos2)
{
	uint8_t befehl = (uint8_t) 0x88;

	uint32_t Register;
	uint8_t initByte[8];

	initByte[0]=befehl;
	initByte[1]=0xFF;
	initByte[2]=0xFF;
	initByte[3]=vMax;
	initByte[3]<<=4;
	initByte[3]+=vMin;

	Register=pos1;
	Register&=0b1111111100000000;
	Register>>=8;
	initByte[4]=(uint8_t)Register;

	Register=pos1;
	Register&=0b0000000011111111;
	initByte[5]=(uint8_t)Register;


	Register=pos2;
	Register&=0b1111111100000000;
	Register>>=8;
	initByte[6]=(uint8_t)Register;

	Register=pos2;
	Register&=0b0000000011111111;
	initByte[7]=(uint8_t)Register;

	i2cBurstWrite(i2c,mot_addr, initByte, 8);
}


/**
 * resets the position of the stepper to `0`.
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
void resetPosition (uint8_t addr)
{
	uint8_t befehl = (uint8_t) 0x86;
	i2cBurstWrite(i2c,addr, &befehl, 1);
}


/**
 * sets the target position to the saved `Secure Position`
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
void gotoSecurePosition(uint8_t addr)
{
	uint8_t befehl = (uint8_t) 0x84;
	i2cBurstWrite(i2c, addr, &befehl,1);
}


/**
 * resets the stepper configuration to the default values (datasheet & OTP memory)
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
void resetToDefault(uint8_t addr)
{
	uint8_t befehl = (uint8_t) 0x87;
	i2cBurstWrite(i2c,addr, &befehl, 1);
}


/**
 * slows down the current movement to Vmin before stopping it completely
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
void softStop(uint8_t addr)
{
	uint8_t befehl = (uint8_t) 0x8F;
	i2cBurstWrite(i2c,addr, &befehl, 1);
}


/**
 * stops all movement immediately
 * after this command the `stepMotorInit(...)` has to be executed again to make the stepper function again
 *
 * @param uint8_t addr - 8bit adress of the stepper
 * @returns void
 */
void hardStop(uint8_t addr)
{
	uint8_t befehl = (uint8_t) 0x85;
	i2cBurstWrite(i2c,addr, &befehl, 1);
}


/**
 * all stepper initialization tasks are handled here
 *
 * @param uint8_t mot_addr - 8bit adress of the stepper
 * @param uint8_t rotdir - `TRUE` / `FALSE`
 * @returns void
 */
void stepMotorInit(uint8_t mot_addr, uint8_t rotdir)
{

/*   in den Geräteteriebern bitte Finger weg von den I2C-Schnittstellenparametern
 *
    i2cDisableDevice(i2c);
    i2cSelectI2C(i2c);                      // I2C1: activate bus clock

    // activate GPIOB bus clock because of the usage of PB8 & PB9 for i2c
    GPIO_TypeDef  *portB = GPIOB;
    gpioInitPort(portB);                        // configuration of gpio pins
    gpioSelectPinMode(portB, PIN8, ALTFUNC);    //
    gpioSelectAltFunc(portB, PIN8, AF4);        // using PB8 as SCL (I2C1)
    gpioSelectPinMode(portB, PIN9, ALTFUNC);    //
    gpioSelectAltFunc(portB, PIN9, AF4);        // using PB9 as SDA (I2C1)



       gpioSetOutputType(portB, PIN8, OPENDRAIN);   // do not use the interal pull up
    gpioSetOutputType(portB, PIN9, OPENDRAIN);   // do not use the interal pull up


	// initialization of the I2C-controllers
	i2cInitI2C(
        i2c,                                    //
        I2C_DUTY_CYCLE_2,                       //
        17,                                     // set rise time: 17 (experience value)
        I2C_CLOCK_400                           // bus clock
        );
*/
	// Init Motor-Params
	getFullStatus1(mot_addr);					// read the first system register of the stepper
	getFullStatus2(mot_addr); 					// read the second system register of the stepper

	initMotorParam(mot_addr, rotdir);
	resetPosition(mot_addr); 					/*
												 * reset stepper Position wird auf 0
												 * so that position cannot be the same as that of `runInit(...)`
												 * if the positions are the same --> DEADLOCK!
												 */

	runInit(mot_addr, 0x8,0x5,0x0002,0x0001); 	// go to reference position
}
