/**
 *      step.h
 * 
 *      @file step.c provides the methods to control a stepper via the `AMIS-30624`
 *      @author: Stefan Heinrich, Dennis Lotz
 *      Created on: Dez. 05, 2023
 */

#ifndef STEP_H_
#define STEP_H_

#ifdef __cplusplus
extern "C" {
#endif


////******************** Funktionen zum StepMotor
//         Falls diese Funktionen ohne das Hauptprogramm verwendent werden (Aufruf ohne StepTask)
//         muss daran gedacht werden die StatusRegister zu aktualisieren


// Aufruf der Motorinitialisierung
//void stepMotorInit(void);


//Funktionen zum Aktualisieren des StatusRegister
extern void getFullStatus1(uint8_t addr);
extern void getFullStatus2(uint8_t addr);

extern void hardStop(uint8_t addr);
extern void softStop(uint8_t addr);
extern void resetToDefault(uint8_t addr);
extern void resetPosition (uint8_t addr);
extern void gotoSecurePosition(uint8_t addr);

// void runInit(uint8_t mot_addr, uint8_t vMax, uint8_t vMin, uint32_t pos1, uint32_t pos2);
// void initMotorParam(uint8_t addr, uint8_t rotdir);
extern void stepMotorInit(uint8_t mot_addr, uint8_t rotdir);

extern void setPosition(uint8_t addr, int data);
extern void setStepMode(uint8_t addr, uint8_t StepMode);
extern void setAccShape(uint8_t addr, uint8_t Databyte);
extern void setSecurePosition(uint8_t addr, uint32_t Dataword);
extern void setAcceleration(uint8_t addr, uint8_t Databyte);
extern void setShaft(uint8_t addr, uint8_t Databyte);
extern void setVmin (uint8_t addr, uint8_t vMin);
extern void setVmax (uint8_t addr, uint8_t vMax);
extern void setIhold(uint8_t addr,uint8_t Databyte);

extern uint8_t getStepMode(uint8_t addr);
extern uint8_t getAccShape(uint8_t addr);
extern uint8_t getAcceleration(uint8_t addr);
extern uint8_t getShaft(uint8_t addr);
extern uint8_t getVmin (uint8_t addr);

extern int16_t getActualPosition(uint8_t addr);
extern int16_t getTargetPosition(uint8_t addr);

extern uint32_t getSecurePosition(uint8_t addr);
extern uint32_t getRpm(uint8_t addr);



#ifdef __cplusplus
}
#endif



#endif /* STEP_H_ */
