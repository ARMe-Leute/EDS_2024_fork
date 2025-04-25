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
    /**
     * @brief Standardkonstruktor.
     */
    PIDController();

    /**
     * @brief Konstruktor mit Initialisierungsparametern.
     *
     * @param kp Proportionalanteil
     * @param ki Integralanteil
     * @param kd Differentialanteil
     * @param ta Abtastzeit in Sekunden
     */
    PIDController(float kp, float ki, float kd, float ta);

    /**
     * @brief Initialisiert den PID-Regler.
     *
     * @param kp Proportionalanteil
     * @param ki Integralanteil
     * @param kd Differentialanteil
     * @param ta Abtastzeit in Sekunden
     */
    void init(float kp, float ki, float kd, float ta);

    /**
     * @brief Führt die PID-Regelung mit gegebenem Fehlerwert aus.
     *
     * @param diff Regelabweichung (Soll - Ist)
     * @return float Berechnete Stellgröße
     */
    float pidControl(float diff);

    /**
     * @brief Setzt die internen Zustände des PID-Reglers zurück.
     */
    void reset();
};

#endif