#ifndef REGLER_H_
#define REGLER_H_
/**
 ******************************************************************************
 * @file	regler.h
 * @author	Prof Flaemig <https://github.com/tflaemig68/>
 * @brief	Regler V2.1
 * @date	Aug 2025
 ******************************************************************************
 * @attention Functions for Closed Loop Control (PID) and Signal Filter using Recursive Lowpass Function
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 *
 ******************************************************************************
 */


// ******************** Predeclare PID_Control Type ********************
typedef struct PIDContr PIDContr_t;

/**
* @brief PIDContr Structure of external and internal values for a common PID Controller
* based on the fixed cycle time calculation, triggered by PID->run command
*/
struct PIDContr
{
	float KP;				//!< Proportional Coefficient
	float KI;				//!< Integral Coeff.
	float KD;				//!< Differential Coeff
	float ISUM;				//!< Integral Sum
	float TA;				//!< Cycle Time sec
	float InpOld;			//!< last Input Value
	PIDContr_t* (*get)(PIDContr_t*);		//!< get PID Referenze
	void (*set)(PIDContr_t*, PIDContr_t*);	//!< Copy PID Parameter
	void (*init)(PIDContr_t*, float, float, float, float);  //!< Initialize PID with Parameters KP KI KD TA
	float (*run)(PIDContr_t*, float);   //!< run PID Control with in: difference end out: result
};


/**
* @brief Low Pass Filter using the weighted average calculation
* based on the fixed cycle time calculation, triggered by PID->run command
*/
typedef struct MeanVal MeanVal_t;
struct MeanVal
{
	float sto_mw;				//!> internal stored mean value
	float weight;				//!> weight of the new measured input
};

extern float MeanValrun(MeanVal_t* mVal, float Inp);

extern void MeanVALclr(MeanVal_t* mVal, float wight);

extern PIDContr_t* PIDget(PIDContr_t* PIDContr);
extern void PIDset(PIDContr_t* Source, PIDContr_t* Desti);
extern void PIDinit(PIDContr_t* PIDParam, float KP, float KI, float KD, float TA);
extern float PIDrun(PIDContr_t* PID, float Diff);
extern void PIDclear(PIDContr_t* PIDParam);
extern const PIDContr_t PID;

/**
* @brief LowPass Filter for 3D Axis Sensordata
*/
extern void LowPassFilt(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp);

#endif /* REGLER_H_ */
