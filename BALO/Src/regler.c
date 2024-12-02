
/**
 ******************************************************************************
 * @file           : regler.c
 * @author         : Prof Flaemig
 * @brief          : Regler
 ******************************************************************************
 * @attention
 * Functions for Closed Loop Control (PID) and filter signals (LowPass)
 * Copyright (c) 2024 Prof T Flämig.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/



#include <stdint.h>
#include <stdbool.h>
#include <regler.h>

// define prototype structure for the PIDContr
const PIDContr_t PID = {
	.KP =0,
	.KI = 0,
	.KD = 0,
	.ISUM = 0,
	.TA = 0,
	.InpOld = 0,
	.get = getPID,
	.set = setPID,
	.init = initPID,
	.run = runPID,
};


/**
 * get the PID Coeffizent
 * @param PIDContr_t* PID parameters
 * @returns PIDCOntr_t* - the stepper with the motor parameters
 */

PIDContr_t* getPID(PIDContr_t* PIDContr)
{
	return PIDContr;
};

void setPID(PIDContr_t* Source, PIDContr_t* Desti)
{
	Desti->KP = Source->KP;
	Desti->KI = Source->KI;
	if (Desti->KI == 0)	{Desti-> ISUM = 0;	}
	Desti->KD = Source->KD;
}



void initPID(PIDContr_t* PIDParam, float KP, float KI, float KD, float TA)
{
	PIDParam->ISUM = 0;
	PIDParam->KP = KP;
	PIDParam->KI = KI;
	PIDParam->KD = KD;
	PIDParam->TA = TA;
	PIDParam->InpOld =0;
};


float runPID(PIDContr_t* PID, float Diff)
{
	PID->ISUM += Diff;
	float result = (PID->KP * Diff) + (PID->KI * PID->ISUM *PID->TA) + (PID->KD / PID->TA)*(Diff - PID->InpOld);
	PID->InpOld = Diff;
	return result;
};



/** Example main used with PID_Regler
 *
 *
 float PID_PosOut, PID_VeloOut;
 int main() {
 	struct PIDRegler PID_Pos, PID_Velo;
 	const uint8_t iHold = 5;

	// PID_Pos.init(... 	KP, KI, KD)
 	initPID(&PID_Pos, 0.5,    0, 0.2);
	initPID(&PID_Velo, 0.1, 0.2, 0.1);

	PID_PosOut= runPID(&PID_Pos, ePos);

   
    StepperSetPos(StepL, PID_PosOut);
	
	
    //  copy ot Parameter KP,KI,KD from PID_Pos to PID_new
	struct PIDRegler PID_new;
    PID.copy(&PID_Pos, &PID_new);


**/



// Tiefpassfilterung der drei Richtungsvektoren xyz
void LowPassFilt(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp)
{
	static long _sto_xyz[3];
	uint8_t i;
	for (i=1;i<=3;i++)
	{
  	 	_sto_xyz[i] += (long) raw_data[i] - (filt_data[i] = _sto_xyz[i]/_tp);
	}
}

/*
struct LPFilter
{

};
*/
