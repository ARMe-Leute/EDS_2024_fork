/**
 * @file pidController.h
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Deklaration der Klasse PIDController zur Umsetzung eines klassischen PID-Reglers.
 * @version 0.1
 * @date 2025-04-06
 * @copyright Copyright (c) 2025
 */

#ifndef PIDCONTROLLER
#define PIDCONTROLLER

/**
 * @class PIDController
 * @brief Implementierung eines klassischen PID-Reglers.
 *
 * Diese Klasse stellt Methoden zur Initialisierung, Berechnung und Rücksetzung
 * eines PID-Reglers zur Verfügung. Der Regler verarbeitet Regelabweichungen und
 * liefert eine Stellgröße basierend auf den einstellbaren Koeffizienten.
 */
class PIDController
{
private:
    float KP;      ///< Proportionalanteil
    float KI;      ///< Integralanteil
    float KD;      ///< Differentialanteil
    float ISum;    ///< Aufsummierter Wert für den Integralanteil
    float TA;      ///< Abtastzeit in Sekunden
    float inpOld;  ///< Vorheriger Regelwert zur Berechnung des Differentialanteils

public:
    PIDController();
    PIDController(float kp, float ki, float kd, float ta);
    void init(float kp, float ki, float kd, float ta);
    float pidControl(float diff);
    void reset();
};

#endif