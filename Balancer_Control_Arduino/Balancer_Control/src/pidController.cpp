/**
 * @file pidController.cpp
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Implementation der Klasse PIDController
 * @version 0.1
 * @date 2025-04-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "pidController.h"

/**
 * @brief Default Constructor, notwendig
 *
 */
PIDController::PIDController() {}

/**
 * @brief Erzeugt ein neues Objekt der Klasse PID-Controller
 *
 * Ruft die Funktion init auf.
 *
 * @param kp Proportional Koeffizient
 * @param ki Integral Koeffizient
 * @param kd Differential Koeffizient
 * @param ta Zyklus-Zeit in Sekunden
 */
PIDController::PIDController(float kp, float ki, float kd, float ta)
{
   init(kp, ki, kd, ta);
}

/**
 * @brief Werte des PID Controllers initialisieren
 *
 * @param kp Proportional Koeffizient
 * @param ki Integral Koeffizient
 * @param kd Differential Koeffizient
 * @param ta Zyklus Zeit in Sekunden
 */
void PIDController::init(float kp, float ki, float kd, float ta)
{
   KP = kp;
   KI = ki;
   KD = kd;
   TA = ta;

   ISum = 0;
   posISum = 0;
   inpOld = 0;
}

/**
 * @brief PID-Regler
 *
 * Hier findet die Mathematische Berechnung des PID-Reglers statt.
 *
 * @param diff Differenz zwischen Soll- und Istwert der Motorposition
 * @return float
 */
float PIDController::pidControl(float diff)
{
   // TODO: Geschwindigkeit zu Position differenzieren
   posISum += diff;
   if (KI == 0)
   {
      ISum = 0;
   }
   else
   {
      ISum += posISum;
   }
   float result = (KP * ISum) + (KI * ISum * TA) + (KD / TA) * (ISum - inpOld);
   inpOld = ISum;
   return result;
}

/**
 * @brief Reset des PID
 * 
 * Setzt die Werte des PID Controllers auf 0 zurück, beispielsweise nach Umkippen.
 * 
 */
void PIDController::reset()
{
   ISum = 0;
    posISum = 0;
    inpOld = 0;
}