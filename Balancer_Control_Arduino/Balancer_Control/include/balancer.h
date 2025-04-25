/**
 * @file balancer.h
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Deklaration der Klasse Balancer
 * @version 0.1
 * @date 2025-04-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef BALANCER
#define BALANCER

#include "pidController.h"
#include "gyroSensor.h"

typedef enum
{
   u,
   v,
   w
} hallPhases;

typedef enum
{
   fwd,
   bwd,
   unknown
} direction;

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

   // Statische ISR-Wrapper
   static void recogniseHallPulseU();
   static void recogniseHallPulseV();
   static void recogniseHallPulseW();

   // Zeiger auf aktuelle Instanz (Singleton)
   static Balancer *instance;

   volatile bool newDirectionEvent = false;

   float currentVelocity; ///< Aktuelle Geschwindigkeit
   direction currentDirection; ///< Aktuelle Richtung

private:
   void recogniseHallPulse(hallPhases phase);
   void applyMotor(float val);

   int16_t sensorBuffer[6];
   GyroSensor gyroSensor;    // TODO: MPU einbinden
   PIDController pid_pos; ///< PID-Regler Position
   PIDController pid_v;   ///< PID-Regler Geschwindigkeit
   PIDController pid_phi; ///< PID-Regler Pitch-Winkel

   float flV;
   float flPWM;                         ///< Ergebnis des Reglers, mit dem der Motor angesteuert wird
   float currentPitch;                  ///< Aktueller Pitch-Wert
   float velocityFactor;                ///< Umrechnungsfaktor für die Geschwindigkeitsberechnung
   volatile hallPhases directionBfr[3]; ///< Array für Richtungserkennung
   volatile int directionBfrIncrement = 0;
};

#endif // BALANCER