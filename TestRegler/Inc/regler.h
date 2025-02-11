#ifndef REGLER_H_
#define REGLER_H_



// ******************** Predeclare PID_Control Type ********************
typedef struct PIDContr PIDContr_t;

struct PIDContr
{
	float KP;				// Proportional Coefficient eg [steps/phi]
	float KI;				// Integral Coeff.
	float KD;				// Differential Coeff
	float ISUM;				// Integral Sum
	float TA;				// Cycle Time sec
	float InpOld;			// last Input Value
	PIDContr_t* (*get)(PIDContr_t*);		// get PID Referenze
	void (*set)(PIDContr_t*, PIDContr_t*);	// Copy PID Parameter
	void (*init)(PIDContr_t*, float, float, float, float);  // Initialize PID with Parameters KP KI KD TA
	float (*run)(PIDContr_t*, float);   // run PID Control with in: difference end out: result
};

extern PIDContr_t* getPID(PIDContr_t* PIDContr);
extern void setPID(PIDContr_t* Source, PIDContr_t* Desti);
extern void initPID(PIDContr_t* PIDParam, float KP, float KI, float KD, float TA);
extern float runPID(PIDContr_t* PID, float Diff);





// LowPass Filter for 3D Axis Sensordata
extern void LowPassFilt(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp);

#endif /* REGLER_H_ */
