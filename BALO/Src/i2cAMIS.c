/**
 *      i2cAMIS.c
 * 
 *      @file i2cAMIS.c provides the methods to control a stepper via the `AMIS-30624`
 *      @author: Stefan Heinrich, Dennis Lotz
 *      Created on: Dez. 05, 2023
 */


#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <i2cAMIS.h>


// define the parameter names
char i2cBusName[] = "i2c Bus";
char i2cAddressName[] = "i2c Address";
char iRunName[] = "iRun";
char iHoldName[] = "iHold";
char vMinName[] = "vMin";
char vMaxName[] = "vMax";
char stepModeName[] = "StepMode";
char rotationDirectionName[] = "Rotation Directioin";
char accelerationName[] = "Acceleration";
char accelerationShapeName[] = "Acceleration Shape";
char positionName[] = "Position";
char securePositionName[] = "Secure Position";
char pwmFrequencyName[] = "PWM Frequency";
char pwmJitterName[] = "PWM Jitter";
char relativeMotionThresholdName[] = "Relative Motion Threshold";

// define the parameter descriptions
char i2cBusDescription[] = "The i2c bus the stepper is connected to.";
char i2cAddressDescription[] = "The i2c Address of the stepper.";
char iRunDescription[] = "Current amplitude value to be fed to each coil of the stepper";
char iHoldDescription[] = "Hold current for each coil of the stepper";
char vMinDescription[] = "Minimum speed of the stepper";
char vMaxDescription[] = "Maximum speed of the stepper";
char stepModeDescription[] = "The stepping mode of the stepper";
char rotationDirectionDescription[] = "This bit defines the rotation direction of the stepper";
char accelerationDescription[] = "The acceleration and deceleration between Vmin and Vmax";
char accelerationShapeDescription[] = "This bit defines if the acceleration is normal or if the stepper runs at Vmin";
char positionDescription[] = "The current position of the stepper";
char securePositionDescription[] = "The secure position of the stepper";
char pwmFrequencyDescription[] = "The PWM frequency of the stepper";
char pwmJitterDescription[] = "Whether jitter is added to the PWM signal";
char relativeMotionThresholdDescription[] = "The relative motion threshold used for motion detection";

// declare methods of the parameter and stepper structure
// has to be declared after the structre definintion
// because it depends on them

/**
 * set the i2c bus of the stepper inside the struct
 * @param Stepper_t* stepper - the instance of the stepper
 * @param I2C_TypeDef i2c - the i2c bus the stepper is connected to
 */
void setI2CBus(Stepper_t* stepper, I2C_TypeDef* i2c)
{
	stepper->i2cBus.i2c = i2c;
}

/**
 * get the i2c bus of the stepper
 * @param Stepper_t* stepper - the instance of the stepper
 * @returns I2C_TypeDef - the i2c bus the stepper is connected to
 */
I2C_TypeDef* getI2CBus(Stepper_t* stepper)
{
	return stepper->i2cBus.i2c;
}

/**
 * set the i2c address of the stepper inside the struct
 * @param Stepper_t* stepper - the instance of the stepper
 * @param uint8_t addr - the i2c address of the stepper
 */
void setI2CAddress(Stepper_t* stepper, uint8_t addr)
{
	stepper->i2cAddress.value = addr;
}

/**
 * get the i2c address of the stepper
 * @param Stepper_t* stepper - the instance of the stepper
 * @returns uint8_t - the i2c address of the stepper
 */
uint8_t getI2CAddress(Stepper_t* stepper)
{
	return stepper->i2cAddress.value;
}

/**
 * set the operating current
 * @param Stepper_t* stepper - the stepper to set the operating current
 * @param uint8_t value - the new operating current
 * @returns void
 */

/**
 * read the data of the first system register of the stepper
 * @param Stepper_t* stepper - the stepper to read the data from
 * @returns void
 */
void getFullStatus1(Stepper_t* stepper, uint8_t readArray[8])
{
	I2C_TypeDef   *i2c;
	uint8_t addr;
	i2c = stepper->i2cBus.i2c;
	addr = stepper->i2cAddress.value;
	uint8_t befehl = (uint8_t) 0x81; // 0x81 is the command to get the FullStatus1
	i2cBurstWrite(i2c, addr, &befehl, 1);
	//for(int i = 0; i<10000; i++){;}		// small delay
	i2cBurstRead(i2c, addr, readArray, 8);
}

/**
 * read the data of the second system register of the stepper
 * @param Stepper_t* stepper - the stepper to read the data from
 * @returns void
 */
void getFullStatus2(Stepper_t* stepper, uint8_t readArray[8])
{ // abspeichern als einzelne parameter im struct
	uint8_t befehl = (uint8_t) 0xFC; // 0xFc is the command to get the FullStatus2
	I2C_TypeDef   *i2c;
	uint8_t addr;
	i2c = stepper->i2cBus.i2c;
	addr = stepper->i2cAddress.value;
	i2cBurstWrite(i2c, addr, &befehl, 1);
	i2cBurstRead(i2c, addr, readArray, 8);
}
/**
 * This method is called once during initialization.
 * The motor is initially set to these values.
 *
 * @param uint8_t addr - 8bit Address of the stepper
 * @param uint8_t rotdir - `TRUE` / `FALSE`
 * @returns void
 */
void setMotorParam(Stepper_t* stepper)
{
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t data[8];

	data[0] = befehl;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = (stepper->iRun.value << 4) | (stepper->iHold.value);		// uint8_t xxxx|yyyy x=iRun y=iHold
	data[4] = (stepper->vMax.value << 4) | (stepper->vMin.value);			// uint8_t xxxx|yyyy x=vMax y=vMin
	data[5] = (stepper->securePosition.value >> 3) | (stepper->rotationDirection.value << 4) | stepper->acceleration.value;		// uint8_t xxx|y|zzzz   x=securePosition(10:8) y=rotationDirection z=acceleration(3:0)
	data[6] = (uint8_t) stepper->securePosition.value;					// =securePosition(7:0)
	// this line for set Stall Param 0x96
	data[7] = 0b10100010 | ((stepper->pwmFrequency.value) << 6) | ((stepper->accelerationShape.value) << 4) | ((stepper->stepMode.value) << 2) | (stepper->pwmJitter.value);
								// uint8_t 1|w|1|x|yy|1|z  w=pwmFrequency x=accelerationShape y=stepMode z=pwmJitter

	i2cBurstWrite(i2c, Addr, data, 8);
}
void setStallParam5(Stepper_t* stepper)
{
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	uint8_t befehl = (uint8_t) 0x96;
	uint8_t data[6];

	data[0] = befehl;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = (stepper->iRun.value << 4) | (stepper->iHold.value);		// uint8_t xxxx|yyyy x=iRun y=iHold
	data[4] = (stepper->vMax.value << 4) | (stepper->vMin.value);			// uint8_t xxxx|yyyy x=vMax y=vMin
	data[5] = ((stepper->securePosition.value >> 3)& 0b1110000) | ((stepper->rotationDirection.value & 0b1)<< 4) | (stepper->acceleration.value);		// uint8_t xxx|y|zzzz   XXXXXXX  z=acceleration(3:0)
	i2cBurstWrite(i2c, Addr, data, 6);
}
void setStallParam(Stepper_t* stepper)
{
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	uint8_t befehl = (uint8_t) 0x96;
	uint8_t data[4];

	data[0] = befehl;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = (stepper->iRun.value << 4) | (stepper->iHold.value);		// uint8_t xxxx|yyyy x=iRun y=iHold
	data[4] = (stepper->vMax.value << 4) | (stepper->vMin.value);			// uint8_t xxxx|yyyy x=vMax y=vMin
	data[5] = ((stepper->securePosition.value >> 3)& 0b1110000) | ((stepper->rotationDirection.value & 0b1)<< 4) | (stepper->acceleration.value);		// uint8_t xxx|y|zzzz   XXXXXXX  z=acceleration(3:0)
	data[6] = (uint8_t) stepper->securePosition.value;					// =securePosition(7:0)
	// this line for set Stall Param 0x96
	data[7] = 0b10100010 | (stepper->pwmFrequency.value << 6) | (stepper->accelerationShape.value << 4) | (stepper->stepMode.value << 2) | stepper->pwmJitter.value;	// uint8_t ooo|x|yy|d|z  o=FS2Stall x=accelerationShape y=stepMode d =DC100SfEn z=pwmJitter
	//data[7]=0b00001100;				// xxx|y|zz|xx  x=N/A y=AccShape z=stepMode
	i2cBurstWrite(i2c, Addr, data, 4);
}



void setIRun(Stepper_t* stepper, uint8_t value)
{
	stepper->iRun.value = value;
	setStallParam5(stepper);
}

/**
 * get the currently set operating current
 * @param Stepper_t* stepper - the instance of the stepper
 * @returns uint8_t - the currently set operating current
 */
uint8_t getIRun(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->iRun.value = data[1] >> 4; // iRun is in the second register
	ui_ret = stepper->iRun.value;
	return ui_ret;
}

/**
 * set the holding current
 * @param Stepper_t* stepper - the stepper to set the holding current
 * @param uint8_t value - the new holding current
 * @returns void
 */
void setIHold(Stepper_t* stepper, uint8_t value)
{
	stepper->iHold.value = value;
	setMotorParam(stepper);
}

/**
 * get the currently set holding current
 * @param Stepper_t* stepper - the stepper to read the current from
 * @returns uint8_t - the currently set holding current
 */
uint8_t getIHold(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->iHold.value = data[1] & (uint8_t) 0b00001111;
	ui_ret = stepper->iHold.value;
	return ui_ret;
}

/**
 * set the minimum speed
 * @param Stepper_t* stepper - the stepper to set the minimum speed
 * @param uint8_t value - the new minimum speed
 * @returns void
 */
void setVMin(Stepper_t* stepper, uint8_t value)
{
	stepper->vMin.value = value;
	setStallParam5(stepper);
}

/**
 * get the currently set minimum speed
 * @param Stepper_t* stepper - the stepper to read the current from
 * @returns uint8_t - the currently set minimum speed
 */
uint8_t getVMin(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->vMin.value = data[2] & (uint8_t) 0b00001111;
	ui_ret = stepper->vMin.value;
	return ui_ret;
}

/**
 * set the maximum speed
 * @param Stepper_t* stepper - the stepper to set the maximum speed
 * @param uint8_t value - the new maximum speed
 * @returns void
 */
void setVMax(Stepper_t* stepper, uint8_t value)
{
	stepper->vMax.value = value;
	setStallParam5(stepper);
}

/**
 * get the maximum speed
 * @param Stepper_t* stepper - the stepper to read the maximum speed from
 * @returns uint8_t - the currently set maximum speed
 */
uint8_t getVMax(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->vMax.value = data[2] >> 4;
	ui_ret = stepper->vMax.value;
	return ui_ret;
}

/**
 * set the new stepping mode
 *  `0x0`: 1/2  stepping
 *  `0x1`: 1/4  stepping
 *  `0x2`: 1/8  stepping
 *  `0x3`: 1/16 stepping
 *
 * @param Stepper_t* stepper - the stepper to set the stepping mode
 * @param uint8_t value `0x0` .. `0x3` - for more information view data sheet 
 * @returns void
 */
void setStepMode(Stepper_t* stepper, uint8_t value)
{
	stepper->stepMode.value = value;
	setStallParam(stepper);
}

/**
 * get the currently set stepping mode
 *  `0x0`: 1/2  stepping
 *  `0x1`: 1/4  stepping
 *  `0x2`: 1/8  stepping
 *  `0x3`: 1/16 stepping
 *
 * @param Stepper_t* stepper - the stepper to read the stepping mode from
 * @returns uint8_t - `0x0` .. `0x3`
 */
uint8_t getStepMode(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->stepMode.value = data[3] >> 5;
	ui_ret = stepper->stepMode.value;
	return ui_ret;
}

/**
 * set the set rotation direction cw / ccw
 * @param Stepper_t* stepper - the stepper to set the rotation direction
 * @param uint8_t value - 0x0 / 0x1, this translates to cw or ccw - for more information view data sheet
 * @returns void
 */
void setRotDir(Stepper_t* stepper, uint8_t value)
{
	stepper->rotationDirection.value = value;
	setStallParam5(stepper);
}

/**
 * get the currently set rotation direction cw / ccw
 * @param Stepper_t* stepper - the stepper to read the rotation direction from
 * @returns uint8_t - returns 0 or 1
 */
uint8_t getRotDir(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->rotationDirection.value = data[3] >> 4;
	stepper->rotationDirection.value = stepper->rotationDirection.value & 0b00000001;
	ui_ret = stepper->rotationDirection.value;
	return ui_ret;
}

/**
 * set the acceleration
 * @param Stepper_t* stepper - the stepper to set the acceleration
 * @param uint32_t value - the acceleration value
 * @returns void
 */
void setAccel(Stepper_t* stepper, uint8_t value)
{
	stepper->acceleration.value = value;
	setStallParam5(stepper);
}

/**
 * get the acceleration
 * @param Stepper_t* stepper - the stepper to read the acceleration from
 * @returns uint8_t - the acceleration value: 0x0 .. 0xF, 49Full−step/s^2 .. 40047Full−step/s^2 - for more information view data sheet
 */
uint8_t getAccel(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->acceleration.value = data[3] & (uint8_t) 0b00001111;
	ui_ret =stepper->acceleration.value;
	return ui_ret;
}

/**
 * set the secure position
 * @param Stepper_t* stepper - the stepper to set the secure position
 * @param uint8_t value - the shaft value
 * @returns void
 */
void setSecPos(Stepper_t* stepper, uint16_t value)
{
	stepper->securePosition.value = value;
	setStallParam(stepper);
}

/**
 * get the currently set secure position. This is the position to which the motor is driven
 * if a HW pin connection is lost. If `<SecPos[10:2]> = 0x400`, secure positioning is disabled;
 * the stepper−motor will be kept in the position occupied at the moment these events occur.
 * 
 * @param Stepper_t* stepper - the stepper to read the secure position from
 * @returns uint32_t - the secure position of the stepper 
 */
uint16_t getSecPos(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus2(stepper, data);
	stepper->securePosition.value = ((uint16_t)data[6] << 8) + (uint16_t)data[5];
	stepper->securePosition.value &= (uint16_t) 0x01FF;
	ui_ret = stepper->securePosition.value;
	return ui_ret;
}

/**
 * set the acceleration shape
 * 
 * @param Stepper_t* stepper - the stepper to set the acceleration shape
 * @param uint8_t value - 0x0 / 0x1, `0`: normal acceleration from vMin to vMax, `1`: motion at vMin without acceleration - for more information view data sheet 
 * @returns void
 */
void setAccelShape(Stepper_t *stepper, uint8_t value)
{
	stepper->accelerationShape.value = value;
	setStallParam(stepper);
}

/**
 * get the currently set acceleration shape
 * 
 * @param stepper -  the acceleration is read from the pointer stepper
 * @returns uint8_t
 * @retval 0 : normal acceleration from vMin to vMax, `
 * @retval 1 : motion at vMin without acceleration
 */
uint8_t getAccelShape(Stepper_t* stepper)
{
	uint8_t data[8], ui_ret;
	getFullStatus1(stepper, data);
	stepper->accelerationShape.value = data[3] >> 7;
	ui_ret = stepper->accelerationShape.value;
	return ui_ret;
}

/**
 * get the current position of the stepper
 *
 * @param 	stepper - the stepper to read the acceleration shape from
 * @returns uint16_t current Position `-30000` .. `+30000`
 * @brief	it takes at 100kHz of I2C Clock  1,16ms
 */
int16_t StepperGetPos(Stepper_t* stepper)
{
	uint8_t data[8];
	int16_t i_ret;
	uint8_t befehl = (uint8_t) 0xFC; // 0xFc is the command to get the FullStatus2
	I2C_TypeDef   *i2c;
	uint8_t addr;
	i2c = stepper->i2cBus.i2c;
	addr = stepper->i2cAddress.value;
	i2cBurstWrite(i2c, addr, &befehl, 1);
	i2cBurstRead(i2c, addr, data, 3);


	stepper->position.value = (int16_t) ((((uint16_t) data[1]) << 8) | (uint16_t) data[2]);
	i_ret = stepper->position.value;
	return i_ret;
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
 * @param Stepper_t* stepper - the stepper to set the position
 * @param int step - the step the stepper shall go to - for more information view data sheet 
 * @returns void
 */
void StepperSetPos(Stepper_t* stepper, int16_t value)
{ // was setPosition
	// it takes at 100KHz I2C Clock 1,18ms
	stepper->position.value = value;
	uint8_t befehl = (uint8_t) 0x8B;
	uint8_t data[5];
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;

	data[0] = befehl; 		// first byte is the 'SetMotorParam' command
	data[1] = 0xFF; 		// second byte is 0xFF
	data[2] = 0xFF; 		// third byte is 0xFF
	data[3] = (uint8_t) (stepper->position.value >> 8); 						// forth byte is the first 8 bits of the data
	data[4] = (uint8_t) (stepper->position.value & 0b0000000011111111); 		// fifth byte is the last 8 bits of the data
	i2cBurstWrite(i2c, Addr, data, 5); 	// send the data to the stepper
}

/**
 * get the PWM frequency not available
*/
/**
 * set the PWM frequency
 * @param Stepper_t* stepper - the stepper to set the PWM frequency
 * @param uint8_t value - the new PWM frequency: default 0x0 = 22kHz; 0x1 = 44kHz no more annoying motor sound - for more information view data sheet
*/
void setPWMFrequency(Stepper_t* stepper, uint8_t value)
{
	stepper->pwmFrequency.value = value;
	setMotorParam(stepper);
	/*
	uint8_t befehl = (uint8_t) 0x89;
	uint8_t data[8];
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	
	data[0] = befehl; 		// first byte is the 'SetMotorParam' command
	data[1] = 0xFF; 		// second byte is 0xFF
	data[2] = 0xFF; 		// third byte is 0xFF
	data[3] = (stepper->iRun.value << 4) | stepper->iHold.value; 	// forth byte is iRun and iHold
	data[4] = (stepper->vMax.value << 4) | stepper->vMin.value; 	// fifth byte is vMax and vMin
	data[5] = (stepper->securePosition.value >> 3) | (stepper->rotationDirection.value << 4) | stepper->acceleration.value; 	// sixth byte is securePosition(10:8), rotationDirection and acceleration
	data[6] = (uint8_t)(stepper->securePosition.value); 	// seventh byte is securePosition(7:0)
	data[7] = 0x10100010 | (stepper->pwmFrequency.value << 6) | (value << 4) | (stepper->stepMode.value) | stepper->pwmJitter.value;	// pwmFrequency, AccShape, stepMode, pwmJitter
	i2cBurstWrite(i2c, Addr, data, 8); 	// send the data to the stepper
	*/
}

/**
 * get the PWM Jitter
 * @param 	stepper - the stepper to read the PWM Jitter from
 * @returns uint8_t - the currently set PWM Jitter
*/
uint8_t getPWMJitter(Stepper_t* stepper) {
	uint8_t data[8];
	getFullStatus2(stepper, data);
	stepper->pwmJitter.value = data[7] & (uint8_t) 0x0000001F;
	return stepper->pwmJitter.value;
}


/**
 * set the PWM Jitter
 * @param Stepper_t* stepper - the stepper to set the PWM Jitter
 * @param uint8_t value - the new PWM Jitter: 0x0 / 0x1 - for more information view data sheet
*/
void setPWMJitter(Stepper_t* stepper, uint8_t value)
{
	stepper->pwmJitter.value = value;
	setMotorParam(stepper);
/**

	uint8_t befehl = (uint8_t) 0x89;
	uint8_t data[8];
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	
	data[0] = befehl; 		// first byte is the 'SetMotorParam' command
	data[1] = 0xFF; 		// second byte is 0xFF
	data[2] = 0xFF; 		// third byte is 0xFF
	data[3] = (stepper->iRun.value << 4) | stepper->iHold.value; 	// forth byte is iRun and iHold
	data[4] = (stepper->vMax.value << 4) | stepper->vMin.value; 	// fifth byte is vMax and vMin
	data[5] = (stepper->securePosition.value >> 3) | (stepper->rotationDirection.value << 4) | stepper->acceleration.value; 	// sixth byte is securePosition(10:8), rotationDirection and acceleration
	data[6] = (uint8_t)(stepper->securePosition.value); 	// seventh byte is securePosition(7:0)
	data[7] = 0b10100010 | (stepper->pwmFrequency.value << 6) | (	stepper->accelerationShape.value << 4) | (stepper->stepMode.value << 2) | stepper->pwmJitter.value;	// 1|w|1|x|yy|1|z  w = pwmFrequency, x = accelerationShape, y = stepMode, z = pwmJitter
	i2cBurstWrite(i2c, Addr, data, 8); 	// send the data to the stepper
*/
}

/**
 * get the current relative motion threshold
 * 
 * @param Stepper_t* stepper - the stepper to read the relative motion threshold from
 * @returns uint8_t - the currently set relative motion threshold
 */
uint8_t getRelativeMotionThreshold(Stepper_t* stepper) {
	uint8_t data[8];
	getFullStatus1(stepper, data);
	stepper->relativeMotionThreshold.value = data[7] & (uint8_t) 0x00001111;
	return stepper->relativeMotionThreshold.value;
}

/**
 * set the new relative motion threshold
 * 
 * @param Stepper_t* stepper - the stepper to set the relative motion threshold
 * @param uint8_t value - the new relative motion threshold
 * @returns void
 */
void setRelativeMotionThreshold(Stepper_t* stepper, uint8_t value)
{
	stepper->relativeMotionThreshold.value = value;
	uint8_t befehl = (uint8_t) 0x96;
	uint8_t data[8];
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;

	data[0] = befehl; 		// first byte is the 'SetStallParam' command
	data[1] = 0xFF; 		// second byte is 0xFF
	data[2] = 0xFF; 		// third byte is 0xFF
	data[3] = (stepper->iRun.value << 4) | stepper->iHold.value; 	// forth byte is iRun and iHold
	data[4] = (stepper->vMax.value << 4) | stepper->vMin.value; 	// fifth byte is vMax and vMin
	data[5] = (stepper->rotationDirection.value << 4) | stepper->acceleration.value; 	// xxx|y|zzzz x = MinSamples (default = 000), y = rotationDirection, z = acceleration
	data[6] = 0b00001111 & stepper->relativeMotionThreshold.value; 	// xxxx|yyyy x = absolute motion threshold (use 0), y = relative motion threshold
	data[7] = (stepper->accelerationShape.value << 4) | (stepper->stepMode.value << 2) | stepper->pwmJitter.value;	// vvv|w|xx|y|z    v = FS2StallEn (use 0), w = accelerationShape, x = stepMode, y = DC100StEn (default = 0), z = pwmJitter
	i2cBurstWrite(i2c, Addr, data, 8); 	// send the data to the stepper
}


// 
//  ======================================================================================== stepper methods below  ========================================================================================
//




/**
 * get the current target position of the stepper
 *
 * @param 	stepper - the stepper to read the data from
 * @returns uint16_t 16bit integer `-30000` .. `+30000`
 */
int16_t StepperGetTargetPosition(Stepper_t* stepper) { // falls probleme auftreten zurück zu int als return type
	uint8_t data[8];
	getFullStatus2(stepper, data);
	return (int16_t)((((uint16_t) data[3]) << 8) | (uint16_t) data[4]);
}

/**
 * gets the maxiumum achievable speed of the stepper in rpm with the current configuration
 *
 * @param Stepper_t* stepper - the stepper instance
 * @returns int rpm
 */
uint16_t   StepperGetMaxRPM(Stepper_t* stepper) {
	uint16_t rpm;
	uint16_t data = (uint16_t) stepper->vMax.value;

	rpm = ((((uint16_t) 973) * data) / ((uint16_t) 15)) + ((uint16_t) 99);
	rpm = rpm * 3.3;
	return rpm;
}


/**
 * resets the position of the stepper to `0`.
 *
 * @param Stepper_t* stepper - the stepper instance
 * @returns void
 */
void StepperResetPosition(Stepper_t* stepper)
{
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	uint8_t befehl = (uint8_t) 0x86;
	i2cBurstWrite(i2c, Addr, &befehl, 1);
}


/**
 * sets the target position to the saved `Secure Position`
 *
 * @param Stepper_t* stepper - the stepper instance
 * @returns void
 */
void StepperGoToSecurePosition(Stepper_t* stepper)
{
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	uint8_t befehl = (uint8_t) 0x84;
	i2cBurstWrite(i2c, Addr, &befehl,1);
}


/**
 * resets the stepper configuration to the default values (datasheet & OTP memory)
 *
 * @param Stepper_t* stepper - the stepper instance
 * @returns void
 */
void StepperResetToDefault(Stepper_t* stepper)
{
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	uint8_t befehl = (uint8_t) 0x87;
	i2cBurstWrite(i2c, Addr, &befehl, 1);
}


/**
 * slows down the current movement to vMin before stopping it completely
 *
 * @param Stepper_t* stepper - the stepper instance
 * @returns void
 */
void StepperSoftStop (Stepper_t* stepper)
{
	uint8_t befehl = (uint8_t) 0x8F;
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	i2cBurstWrite(i2c, Addr, &befehl, 1);
}

/**
 * stops all movement immediately
 * after this command the `stepMotorInit(...)` has to be executed again to make the stepper function again
 *
 * @param Stepper_t* stepper - the stepper instance
 * @returns void
 */
void StepperHardStop (Stepper_t* stepper)
{
	uint8_t befehl = (uint8_t) 0x85;
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	i2cBurstWrite(i2c, Addr, &befehl, 1);
}


/**
 * This method is used for initialization, sets vMax and vMin, 
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
 * @param Stepper_t* stepper - the stepper to initialize
 * @param uint16_t pos1 - the first target step - for more information view data sheet
 * @param uint16_t pos2 - the second target step - for more information view data sheet
 * @returns void
 */
void StepperSetDualPosition(Stepper_t* stepper, uint16_t pos1, uint16_t pos2)
{
	uint8_t befehl = (uint8_t) 0x88;
	uint8_t data[8];
	I2C_TypeDef   *i2c;
	uint8_t Addr = stepper->i2cAddress.value;
	i2c = stepper->i2cBus.i2c;
	data[0] = befehl; 		// first byte is the 'setDualPosition' command
	data[1] = 0xFF; 		// second byte is 0xFF
	data[2] = 0xFF;			// third byte is 0xFF
	data[3] = (stepper->vMax.value << 4) | stepper->vMin.value; // forth byte is xxxx|yyyy x=vMax and y=vMin
	data[4] = (uint8_t) (pos1 >> 8); 						// fifth byte is the first 8 bits of pos1
	data[5] = (uint8_t) (pos1 & (uint16_t) 0b0000000011111111); 	// sixth byte is the last 8 bits of pos1
	data[6] = (uint8_t) (pos2 >> 8); 						// seventh byte is the first 8 bits of pos2
	data[7] = (uint8_t) (pos2 & (uint16_t) 0b0000000011111111); 	// eighth byte is the last 8 bits of pos2

	i2cBurstWrite(i2c, Addr, data, 8);
}

/**
 * all stepper initialization tasks are handled here
 * the motor is set to the values provided by the user
 *
 * @param uint8_t mot_addr - 8bit Address of the stepper
 * @param uint8_t rotdir - `TRUE` / `FALSE`
 * @returns void
 */
void StepperInit(Stepper_t* stepper, I2C_TypeDef* i2cBus, uint8_t i2cAddress, uint8_t iRun, uint8_t iHold, uint8_t vMin, uint8_t vMax, uint8_t stepMode, uint8_t rotationDirection, uint8_t acceleration, uint16_t securePosition)
{
	stepper->i2cBus.i2c = i2cBus;
	stepper->i2cAddress.value = i2cAddress;
	stepper->iRun.value = iRun;
	stepper->iHold.value = iHold;
	stepper->vMin.value = vMin;
	stepper->vMax.value = vMax;
	stepper->stepMode.value = stepMode;
	stepper->rotationDirection.value = rotationDirection;
	stepper->acceleration.value = acceleration;
	stepper->securePosition.value = securePosition;
	/*stepper->iRun.set(stepper, iRun);
	stepper->iHold.set(stepper, iHold);
	stepper->vMin.set(stepper, vMin);
	stepper->vMax.set(stepper, vMax);
	stepper->stepMode.set(stepper, stepMode);
	stepper->rotationDirection.set(stepper, rotationDirection);
	stepper->acceleration.set(stepper, acceleration);
	stepper->securePosition.set(stepper, securePosition);*/

	uint8_t data[8];
	getFullStatus1(stepper, data);
	getFullStatus2(stepper, data);
	setMotorParam(stepper);

	StepperResetPosition(stepper); 			/*
												 * reset stepper Position wird auf 0
												 * so that position cannot be the same as that of `runInit(...)`
												 * if the positions are the same --> DEADLOCK!
												 */

	StepperSetDualPosition(stepper, 0x02,0x01); 	// set the target position to pos1 and pos2
}

/**
 * set the motor parameters to the values of the stepper
 * 
 * @param Stepper_t* stepper - the stepper to set the motor parameters
 * @returns void
 */
void setStepper(Stepper_t* stepper, Stepper_t* stepper1) {
	stepper->i2cBus.i2c = stepper1->i2cBus.i2c;
	stepper->i2cAddress.value = stepper1->i2cAddress.value;
	stepper->iRun.value = stepper1->iRun.value;
	stepper->iHold.value = stepper1->iHold.value;
	stepper->vMin.value = stepper1->vMin.value;
	stepper->vMax.value = stepper1->vMax.value;
	stepper->stepMode.value = stepper1->stepMode.value;
	stepper->rotationDirection.value = stepper1->rotationDirection.value;
}

/**
 * get the motor parameters of the stepper
 * @param Stepper_t* stepper - the stepper to get the motor parameters
 * @returns Stepper_t* - the stepper with the motor parameters
 */
Stepper_t* getStepper(Stepper_t* stepper) {
	return stepper;
}


// define prototype structure for the stepper
const Stepper_t stepper = {
	.i2cBus = { 
		// do not define default .value here, because the user has to
		.name = i2cBusName,
		.description = i2cBusDescription,
		.set = setI2CBus,
		.get = getI2CBus,
	},
	.i2cAddress = {
		// do not define default .value here, because the user has to
		.name = i2cAddressName,
		.description = i2cAddressDescription,
		.set = setI2CAddress,
		.get = getI2CAddress,
	},
	.iRun = {
		// do not define default .value here, because the user has to
		.name = iRunName,
		.description = iRunDescription,
		.set = setIRun,
		.get = getIRun,
	},
	.iHold = {
		// do not define default .value here, because the user has to
		.name = iHoldName,
		.description = iHoldDescription,
		.set = setIHold,
		.get = getIHold,
	},
	.vMin = {
		// do not define default .value here, because the user has to
		.name = vMinName,
		.description = vMinDescription,
		.set = setVMin,
		.get = getVMin,
	},
	.vMax = {
		// do not define default .value here, because the user has to
		.name = vMaxName,
		.description = vMaxDescription,
		.set = setVMax,
		.get = getVMax,
	},
	.stepMode = {
		// do not define default .value here, because the user has to
		.name = stepModeName,
		.description = stepModeDescription,
		.set = setStepMode,
		.get = getStepMode,
	},
	.rotationDirection = {
		// do not define default .value here, because the user has to
		.name = rotationDirectionName,
		.description = rotationDirectionDescription,
		.set = setRotDir,
		.get = getRotDir,
	},
	.acceleration = {
		// do not define default .value here, because the user has to
		.name = accelerationName,
		.description = accelerationDescription,
		.set = setAccel,
		.get = getAccel,
	},
	.accelerationShape = {
		.value = 0x0,
		.name = accelerationShapeName,
		.description = accelerationShapeDescription,
		.set = setAccelShape,
		.get = getAccelShape,
	},
	.position = {
		// do not define default .value here, because the user has to
		.name = positionName,
		.description = positionDescription,
	},
	.securePosition = {
		.value = 0b0000001100000000, // = 0x300
		// do not define default .value here, because the user has to
		.name = securePositionName,
		.description = securePositionDescription,
		.set = setSecPos,
		.get = getSecPos,
	},
	.pwmFrequency = {
		.value = 0x0,
		.name = pwmFrequencyName,
		.description = pwmFrequencyDescription,
		.set = setPWMFrequency,
		// no get method because the value is not available
	},
	.pwmJitter = {
		.value = 0x0,
		.name = pwmJitterName,
		.description = pwmJitterDescription,
		.set = setPWMJitter,
		.get = getPWMJitter,
	},
	.relativeMotionThreshold = {
		.value = 0x0, // = relative motion detection disable
		.name = relativeMotionThresholdName,
		.description = relativeMotionThresholdDescription,
		.set = setRelativeMotionThreshold,
		.get = getRelativeMotionThreshold,
	},
	.get = getStepper,
	.set = setStepper,
};

/** Example main used with two Stepper motors
 *
 *
 #define i2cAddr_motL 0x61
 #define i2cAddr_motR 0x60
 int main() {
 	struct Stepper StepL, StepR;
 	const uint8_t iHold = 5;

	// StepL.init(... 						iRun,	iHold, 	vMin,  	vMax, 	stepMode, rotDir, acceleration, securePosition)
 	StepperInit(&StepL, i2c, i2cAddr_motL, 	10, 	1,  	2, 		8, 		3, 			1, 		2,			 0);
	StepperInit(&StepR, i2c, i2cAddr_motR, 	10, 	1,  	2, 		8, 		3, 			1, 		2,			 0);

	stepper.iHold.set(&StepL, iHold);

   	int16_t PosL = 20;
    StepperSetPos(StepL, PosL);

    stepper.copy(&StepL, &StepR);
    printf("Description: %s \n", StepL.i2cBus.description);
	printf("Description: %s \n", StepL.i2cAddress.description);

 	// Assign a value to the string using the strcpy function
 	strcpy(StepL.i2cBus.description, "Some text");
	strcpy(StepL.i2cAddress.description, "Some other text");
 	return 0;
 }
 */
