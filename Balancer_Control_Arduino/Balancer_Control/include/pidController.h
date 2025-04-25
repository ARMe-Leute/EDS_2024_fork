/**
 * @file pidController.h
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Deklaration der Klasse PIDController
 * @version 0.1
 * @date 2025-04-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #ifndef PIDCONTROLLER
 #define PIDCONTROLLER

 class PIDController
 {
    private:
        float KP;       ///< Proportional Coefficient
        float KI;       ///< Integral     Coefficient
        float KD;       ///< Differential Coefficient
        float ISum;     ///< Integral Sum fpr PID
        float posISum;  ///< Integral Sum to get from Velocity to position
        float TA;       ///< Cycle Time sec
        float inpOld;   ///< Last Input Value

    public:
        PIDController();
        PIDController(float kp, float ki, float kd, float ta);
        void init(float kp, float ki, float kd, float ta);
        float pidControl(float diff);
        void reset();
 };

 #endif