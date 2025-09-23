
/**
 ******************************************************************************
 * @file	regler.c
 * @author	Prof Flaemig <https://github.com/tflaemig68/>
 * @brief	Regler V2.1
 * @date	Aug 2025
 ******************************************************************************
 * @attention Functions for Closed Loop Control (PID) and Signal Filter using Recursive Lowpass Function
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 *
 ******************************************************************************
 */


#include <stdint.h>
#include <stdbool.h>
#include <regler.h>



/**
 * set the pointer to the parameter PID
 * @param PIDContr - pointer of PID with parameters
 * @returns PIDContr - pointer
 */

PIDContr_t* PIDget(PIDContr_t* PIDContr)
{
	return PIDContr;
};

/**
 * clear the PID internal values
 * @param PIDParam - pointer to PID
 *
 */
void PIDclear(PIDContr_t* PIDParam)
{
	PIDParam->ISUM = 0;		//!< internal value, set zero for the sum e.g. integral value
	PIDParam->InpOld =0;	//!< internal value, last input
};

/**
 * copy the PID coefficents form source to destination PID
 * @param Source pointer to Source
 * @param Desti  pointer to destination PID
 *
 */
void PIDset(PIDContr_t* Source, PIDContr_t* Desti)
{
	Desti->KP = Source->KP;
	Desti->KI = Source->KI;
	if (Desti->KI == 0)	{Desti-> ISUM = 0;	} //!< clear internal sum if KI Value 0
	Desti->KD = Source->KD;
}

/**
 * initialize the PID by setting the coefficents
 * @param PIDParam - pointer to PID
 * @param KP - proportional coefficient
 * @param KI - integral coefficient
 * @param KD - differential coefficient
 * @param TA - sampling time, should be updated if task time changes
 *
 */
void PIDinit(PIDContr_t* PIDParam, float KP, float KI, float KD, float TA)
{
	PIDParam->KP = KP;		// proportional coefficient
	PIDParam->KI = KI;		// integral coefficient
	PIDParam->KD = KD;		// differential coefficient
	PIDParam->TA = TA;		// sampling time, should be updated if task time changes
	PIDclear(PIDParam);
};
/**
 * run the PID coefficents form source to destination PID
 * @param PID - pointer to PID
 * @param Diff - input difference = target minus current value
 *
 */

float PIDrun(PIDContr_t* PID, float Diff)
{
	if (PID->KI == 0)
	{
		PID->ISUM = 0;
	}
	else
	{
		PID->ISUM += Diff;
	}
	float result = (PID->KP * Diff) + (PID->KI * PID->ISUM *PID->TA) + (PID->KD / PID->TA)*(Diff - PID->InpOld);
	PID->InpOld = Diff;
	return result;
};

/**
 * @brief define prototype structure for the PIDContr
 */
const PIDContr_t PID = {
	.KP =0,
	.KI = 0,
	.KD = 0,
	.ISUM = 0,
	.TA = 0,
	.InpOld = 0,
	.get = PIDget,
	.set = PIDset,
	.init = PIDinit,
	.run = PIDrun,
};




// define prototype structure for the MeanVal
const MeanVal_t MV = {
	.sto_mw =0,
	.weight = 0,
};


float MeanVALrun(MeanVal_t* mVal, float Inp)
{
	float mwData;
	mwData = Inp* mVal->weight + (1-mVal->weight)*mVal->sto_mw;
	mVal->sto_mw =mwData;
	return mwData;
}

/**
 * @brief Clear MeanValue Filter and set the weight
 * @param mVal - pointer
 * @param weight - value 0 no influnce by input
 *  0 =   no weight of input only old values were kept
 *  0.5 =  half mean and half new value weights
 *  1  = 	max. weight of new value input
*/
void MeanVALclr(MeanVal_t* mVal, float weight)
{
	mVal->sto_mw = (float) 0;
	mVal->weight = weight;
}

/**
* @brief LowPass Filter for 3D Axis Sensordata
*/
void LowPassFilt(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp)
{
	static long _sto_xyz[3];
	uint8_t i;
	for (i=1;i<=3;i++)
	{
  	 	_sto_xyz[i] += (long) raw_data[i] - (filt_data[i] = _sto_xyz[i]/_tp);
	}
}


