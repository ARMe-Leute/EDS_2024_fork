/**
 * @file pidController.cpp
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Implementation der Klasse PIDController zur Regelung von physikalischen Größen.
 * @version 0.1
 * @date 2025-04-06
 * @copyright Copyright (c) 2025
 */

#include "pidController.h"

/**
 * @brief Standardkonstruktor der PIDController-Klasse.
 */
PIDController::PIDController() {}

/**
 * @brief Konstruktor mit Initialisierungsparametern.
 *
 * Erzeugt ein neues PIDController-Objekt und ruft die Initialisierungsfunktion auf.
 *
 * @param kp Proportionalanteil
 * @param ki Integralanteil
 * @param kd Differentialanteil
 * @param ta Abtastzeit in Sekunden
 */
PIDController::PIDController(float kp, float ki, float kd, float ta)
{
   init(kp, ki, kd, ta);
}

/**
 * @brief Initialisiert den PID-Controller mit gegebenen Parametern.
 *
 * @param kp Proportionalanteil
 * @param ki Integralanteil
 * @param kd Differentialanteil
 * @param ta Abtastzeit in Sekunden
 */
void PIDController::init(float kp, float ki, float kd, float ta)
{
   KP = kp;
   KI = ki;
   KD = kd;
   TA = ta;

   ISum = 0;
   inpOld = 0;
}

/**
 * @brief Führt die PID-Regelung durch.
 *
 * Berechnet den Regelausgang auf Basis der Abweichung (Differenz zwischen Soll- und Istwert).
 *
 * @param diff Regelabweichung (Soll - Ist)
 * @return float Regelausgang (Stellgröße)
 */
float PIDController::pidControl(float diff)
{
   if (KI == 0)
   {
      ISum = 0;
   }
   else
   {
      ISum += diff;
   }

   float result = (KP * diff) + (KI * ISum * TA) + (KD / TA) * (ISum - inpOld);
   inpOld = ISum;
   return result;
}

/**
 * @brief Setzt die internen Zustände des PID-Reglers zurück.
 *
 * z.B. nach Störfall oder einem Systemneustart.
 */
void PIDController::reset()
{
   ISum = 0;
   inpOld = 0;
}