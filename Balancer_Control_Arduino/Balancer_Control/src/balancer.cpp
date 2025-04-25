/**
 * @file balancer.cpp
 * @author Johannes Müller, Dominik Berensspöhler
 * @brief Implementierung der Balancer-Klasse zur Regelung eines selbstbalancierenden Fahrzeugs.
 * @version 0.1
 * @date 2025-04-06
 * @copyright Copyright (c) 2025
 */

#include <Arduino.h>
#include "defines.h"
#include "balancer.h"

/**
 * @brief Statische Instanz der Balancer-Klasse für Interruptzugriffe.
 */
Balancer *Balancer::instance = nullptr;

/**
 * @brief Konstruktor der Balancer-Klasse.
 *
 * Initialisiert Gyroskop und PID-Regler. Beendet das Programm bei Fehler.
 */
Balancer::Balancer()
{
   if (!gyroSensor.init())
   {
      digitalWrite(CTRLLEDPIN, HIGH);
      while (1)
      {
      }
   }
   pid_pos.init(0, 0, 0, 1.0f / PULSECOUNTTIME);
   pid_v.init(0.8, 1.2, 0, 1.0f / PULSECOUNTTIME);
   pid_phi.init(5, 0, 0, 1.0f / PULSECOUNTTIME);
   currentVelocity = 0;
   currentPitch = 0;
   currentDirection = fwd;
   flPWM = 0.0;
   velocityFactor = 3.33333333 / (PULSECOUNTTIME * PULSECOUNTTIME);
}

/**
 * @brief Initialisiert interne Zustände der Balancer-Klasse.
 *
 * @param _mpu GyroSensor-Objekt
 * @param _pid_pos PID-Regler für Position
 * @param _pid_v PID-Regler für Geschwindigkeit
 * @param _pid_phi PID-Regler für Neigungswinkel
 */
void Balancer::init(GyroSensor _mpu, PIDController _pid_pos, PIDController _pid_v, PIDController _pid_phi)
{
   gyroSensor = _mpu;
   pid_pos = _pid_pos;
   pid_v = _pid_v;
   pid_phi = _pid_phi;
   currentVelocity = 0;
   currentPitch = 0;
   currentDirection = fwd;
   flPWM = 0.0;
   velocityFactor = 3.33333333 / (PULSECOUNTTIME * PULSECOUNTTIME);
   Serial.println("Initialisierung abgeschlossen");
}

/**
 * @brief Berechnet die aktuelle Geschwindigkeit anhand gezählter Pulse.
 *
 * @param pulsecount Anzahl der Impulse im Messintervall
 */
void Balancer::getVelocity(int pulsecount)
{
   currentVelocity = pulsecount * velocityFactor;
   if (currentDirection == bwd)
   {
      currentVelocity = -currentVelocity;
   }
}

/**
 * @brief ISR-Handler für Hall-Signal Phase U.
 */
void Balancer::recogniseHallPulseU()
{
   if (instance)
      instance->recogniseHallPulse(u);
}

/**
 * @brief ISR-Handler für Hall-Signal Phase V.
 */
void Balancer::recogniseHallPulseV()
{
   if (instance)
      instance->recogniseHallPulse(v);
}

/**
 * @brief ISR-Handler für Hall-Signal Phase W.
 */
void Balancer::recogniseHallPulseW()
{
   if (instance)
      instance->recogniseHallPulse(w);
}

/**
 * @brief Speichert erkannte Hallphase im Ringpuffer.
 *
 * @param phase Erkannte Phase (U, V oder W)
 */
void Balancer::recogniseHallPulse(hallPhases phase)
{
   directionBfr[directionBfrIncrement] = phase;
   directionBfrIncrement = (directionBfrIncrement + 1) % 3;
   newDirectionEvent = true;
}

/**
 * @brief Ermittelt Drehrichtung anhand der letzten drei Hall-Signale.
 */
void Balancer::getDirection()
{
   int i0 = directionBfrIncrement % 3;
   int i1 = (directionBfrIncrement + 1) % 3;
   int i2 = (directionBfrIncrement + 2) % 3;

   hallPhases a = directionBfr[i0];
   hallPhases b = directionBfr[i1];
   hallPhases c = directionBfr[i2];

   if ((a == u && b == v && c == w) ||
       (a == v && b == w && c == u) ||
       (a == w && b == u && c == v))
   {
      currentDirection = fwd;
   }

   else if ((a == u && b == w && c == v) ||
       (a == v && b == u && c == w) ||
       (a == w && b == v && c == u))
   {
      currentDirection = bwd;
   }
   else
   {
      currentDirection = unknown;
   }
}

/**
 * @brief Aktualisiert den aktuellen Neigungswinkel über den Gyrosensor.
 */
void Balancer::getPitch()
{
   gyroSensor.getAcceleration();
   gyroSensor.getGyro();
   currentPitch = gyroSensor.getWeightedPitch(PULSECOUNTTIME * 0.001);
}

/**
 * @brief Führt die Regelung aus und gibt die PWM an den Motor weiter.
 *
 * Stoppt bei zu starker Neigung automatisch und setzt Regler zurück.
 */
void Balancer::motorOutput()
{
   if (abs(currentPitch) > 23.0)
   {
      flPWM = 0.0;
      applyMotor(flPWM);
      digitalWrite(ENBLPIN, HIGH);
      pid_phi.reset();
      pid_v.reset();
      return;
   }
   else
   {
      digitalWrite(ENBLPIN, LOW);
      flV = pid_phi.pidControl(-currentPitch);
      flPWM = pid_v.pidControl(flV - currentVelocity);
      applyMotor(flPWM);
      return;
   }
}

/**
 * @brief Wendet den berechneten PWM-Wert auf den Motor an.
 *
 * @param val PWM-Wert im Bereich [-PWMMAX, PWMMAX]
 */
void Balancer::applyMotor(float val)
{
   val = constrain(val, -PWMMAX, PWMMAX);

   if (val == 0.0)
   {
      analogWrite(PWMPIN, 0);
   }
   else if (val < 0)
   {
      digitalWrite(DIRPIN, HIGH);
      analogWrite(PWMPIN, (uint8_t) -val);
   }
   else if (val > 0)
   {
      digitalWrite(DIRPIN, LOW);
      analogWrite(PWMPIN, (uint8_t) val);
   }
}