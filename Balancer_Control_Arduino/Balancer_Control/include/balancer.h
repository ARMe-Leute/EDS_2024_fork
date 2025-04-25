/**
 * @file balancer.h
 * @author Johannes Müller, Dominik Berensspöhler
 * @brief Deklaration der Klasse Balancer zur Steuerung eines selbstbalancierenden Fahrzeugs.
 * @version 0.1
 * @date 2025-04-06
 * @copyright Copyright (c) 2025
 */

#ifndef BALANCER
#define BALANCER

#include "pidController.h"
#include "gyroSensor.h"

/**
 * @brief Enum zur Darstellung der drei möglichen Hallphasen.
 */
typedef enum
{
   u, ///< Phase U
   v, ///< Phase V
   w  ///< Phase W
} hallPhases;

/**
 * @brief Enum zur Darstellung der Bewegungsrichtung.
 */
typedef enum
{
   fwd,    ///< Vorwärts
   bwd,    ///< Rückwärts
   unknown ///< Richtung nicht erkennbar
} direction;

/**
 * @brief Die Klasse Balancer implementiert die Regelung eines selbstbalancierenden Fahrzeugs.
 */
class Balancer
{
public:
   /**
    * @brief Konstruktor.
    */
   Balancer();

   /**
    * @brief Alternativer Konstruktor zur Initialisierung mit externen Objekten.
    *
    * @param _mpu GyroSensor-Objekt
    * @param _pid_pos PID-Regler für Position
    * @param _pid_v PID-Regler für Geschwindigkeit
    * @param _pid_phi PID-Regler für Pitch
    */
   Balancer(GyroSensor _mpu, PIDController _pid_pos, PIDController _pid_v, PIDController _pid_phi);

   /**
    * @brief Initialisiert interne Objekte und Parameter.
    *
    * @param _mpu GyroSensor-Objekt
    * @param _pid_pos PID-Regler für Position
    * @param _pid_v PID-Regler für Geschwindigkeit
    * @param _pid_phi PID-Regler für Pitch
    */
   void init(GyroSensor _mpu, PIDController _pid_pos, PIDController _pid_v, PIDController _pid_phi);

   /**
    * @brief Berechnet die aktuelle Geschwindigkeit.
    *
    * @param pulsecount Anzahl gezählter Pulse
    */
   void getVelocity(int pulsecount);

   /**
    * @brief Liest den aktuellen Pitch-Winkel über den Gyrosensor aus.
    */
   void getPitch();

   /**
    * @brief Bestimmt die Bewegungsrichtung anhand der letzten Hall-Signale.
    */
   void getDirection();

   /**
    * @brief Führt die Regelung durch und steuert den Motor entsprechend an.
    */
   void motorOutput();

   /**
    * @brief ISR-Wrapper für Hall-Phase U.
    */
   static void recogniseHallPulseU();

   /**
    * @brief ISR-Wrapper für Hall-Phase V.
    */
   static void recogniseHallPulseV();

   /**
    * @brief ISR-Wrapper für Hall-Phase W.
    */
   static void recogniseHallPulseW();

   /**
    * @brief Statischer Zeiger auf aktuelle Balancer-Instanz.
    */
   static Balancer *instance;

   /**
    * @brief Flag, das anzeigt, ob ein neues Richtungsevent erkannt wurde.
    */
   volatile bool newDirectionEvent = false;

   float currentVelocity;      ///< Aktuelle Geschwindigkeit
   direction currentDirection; ///< Aktuelle Bewegungsrichtung

   float flV;             ///< Zwischenergebnis der inneren Regelung (Pitch zu Sollgeschwindigkeit)
   float flPWM;           ///< Endwert der Regelung für Motorsteuerung
   GyroSensor gyroSensor; ///< Objekt zur Erfassung des Pitch-Winkels
   float currentPitch;                     ///< Aktueller Pitch-Wert

private:
   /**
    * @brief Interne Methode zum Speichern erkannter Hallpulse.
    *
    * @param phase Erkannte Phase (U, V oder W)
    */
   void recogniseHallPulse(hallPhases phase);

   /**
    * @brief Wendet den berechneten PWM-Wert am Motor an.
    *
    * @param val PWM-Wert im Bereich [-PWMMAX, PWMMAX]
    */
   void applyMotor(float val);

   int16_t sensorBuffer[6]; ///< Sensor-Zwischenspeicher (optional)

   PIDController pid_pos; ///< PID-Regler für Position
   PIDController pid_v;   ///< PID-Regler für Geschwindigkeit
   PIDController pid_phi; ///< PID-Regler für Neigungswinkel (Pitch)

   
   float velocityFactor;                   ///< Umrechnungsfaktor von Pulsen zu Geschwindigkeit
   volatile hallPhases directionBfr[3];    ///< Ringpuffer zur Richtungserkennung
   volatile int directionBfrIncrement = 0; ///< Index für Ringpuffer
};

#endif // BALANCER