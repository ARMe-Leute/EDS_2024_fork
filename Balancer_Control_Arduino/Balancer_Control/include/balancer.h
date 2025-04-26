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
   Balancer();
   Balancer(GyroSensor _mpu, PIDController _pid_pos, PIDController _pid_v, PIDController _pid_phi);
   void init(GyroSensor _mpu, PIDController _pid_pos, PIDController _pid_v, PIDController _pid_phi);
   void getVelocity(int pulsecount);
   void getPitch();
   void getDirection();
   void motorOutput();
   static void recogniseHallPulseU();
   static void recogniseHallPulseV();
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
   float currentPitch;    ///< Aktueller Pitch-Wert

private:
   void recogniseHallPulse(hallPhases phase);
   void applyMotor(float val);

   PIDController pid_pos; ///< PID-Regler für Position (nicht verwendet)
   PIDController pid_v;   ///< PID-Regler für Geschwindigkeit
   PIDController pid_phi; ///< PID-Regler für Neigungswinkel (Pitch)

   float velocityFactor;                   ///< Umrechnungsfaktor von Pulsen zu Geschwindigkeit
   volatile hallPhases directionBfr[3];    ///< Ringpuffer zur Richtungserkennung
   volatile int directionBfrIncrement = 0; ///< Index für Ringpuffer
};

#endif // BALANCER