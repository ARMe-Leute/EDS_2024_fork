/**
 * @file balancer.cpp
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Deklaration der Balancer Klasse
 * @version 0.1
 * @date 2025-04-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <Arduino.h>
#include "defines.h"
#include "balancer.h"

/**
 * @brief Erzeugt ein neues Objekt der Klasse Balancer
 *
 * Erzeugt neue Objekte der Klassen MPU6050 und PIDController.
 * Diese bekommt die Balancer Klasse als Attribute.
 *
 */

/**
 * @brief
 *
 */
Balancer *Balancer::instance = nullptr;

Balancer::Balancer()
{
   GyroSensor _mpu;
   if (!_mpu.init())
   {
      digitalWrite(CTRLLEDPIN, HIGH);
      while (1) {}
   }
   PIDController _pid_pos(0, 0, 0, 1 / PULSECOUNTTIME);
   PIDController _pid_v(0.8, 1.2, 0, 1 / PULSECOUNTTIME);
   PIDController _pid_phi(5, 0, 0, 1 / PULSECOUNTTIME);
   init(_mpu, _pid_pos, _pid_v, _pid_phi);
}

/**
 * @brief Erzeugt ein neues Objekt der Klasse Balancer mit vorinitialisertem
 * MPU6050-Sensor und PID-Regler.
 *
 * @param _mpu MPU6050 Sensor
 * @param _pid PID Regler
 */
Balancer::Balancer(GyroSensor _mpu, PIDController _pid_pos, PIDController _pid_v, PIDController _pid_phi)
{
   init(_mpu, _pid_pos, _pid_v, _pid_phi);
}

/**
 * @brief 
 * 
 * @param _mpu 
 * @param _pid_pos 
 * @param _pid_v 
 * @param _pid_phi 
 */
void Balancer::init(GyroSensor _mpu, PIDController _pid_pos, PIDController _pid_v, PIDController _pid_phi)
{
   gyroSensor = _mpu;
   pid_pos = _pid_pos;
   pid_v = _pid_v;
   pid_phi = _pid_phi;
   currentVelocity = 0;
   currentPitch = 0;
   flPWM = 0.0;
   velocityFactor = 3.33333333 / (PULSECOUNTTIME * PULSECOUNTTIME);
}

/**
 * @brief Berechnung der aktuellen Geschwindigkeit
 *
 * Rechnet die gezählten Impulse des Geschwindigkeitseingangs in einem festen Intervall in die Drehfrequenz des Motors
 *
 * @param pulsecount Gezählte Pulse im festgelegten Intervall
 */
void Balancer::getVelocity(int pulsecount)
{
   currentVelocity = pulsecount * velocityFactor;
}

/**
 * @brief Registriert den Hall Puls der Phase U
 *
 */
void Balancer::recogniseHallPulseU()
{
   if (instance)
      instance->recogniseHallPulse(u);
}

/**
 * @brief Registriert den Hall Puls der Phase V
 *
 */
void Balancer::recogniseHallPulseV()
{
   if (instance)
      instance->recogniseHallPulse(v);
}

/**
 * @brief Registriert den Hall Puls der Phase W
 *
 */
void Balancer::recogniseHallPulseW()
{
   if (instance)
      instance->recogniseHallPulse(w);
}

/**
 * @brief Speichert die erkannte Phase im Ringspeicher für weitere Verwendung.
 *
 * @param phase Erkannter Puls der Phase U, V oder W.
 */
void Balancer::recogniseHallPulse(hallPhases phase)
{
   directionBfr[directionBfrIncrement] = phase;
   directionBfrIncrement = (directionBfrIncrement + 1) % 3;
   newDirectionEvent = true;
}

/**
 * @brief Bestimmt die Drehrichtung des Motors anhand der letzten drei Hall-Pulsfolgen.
 *
 * Vergleicht die gespeicherte Reihenfolge der Hallphasen mit bekannten Mustern
 * für Vorwärts- oder Rückwärtsbewegung.
 *
 * @return direction fwd bei Vorwärtsdrehung, bwd bei Rückwärtsdrehung
 */
void Balancer::getDirection()
{
   // Ringpuffer-Auswertung: letzten drei Einträge in richtiger Reihenfolge lesen
   int i0 = directionBfrIncrement % 3;
   int i1 = (directionBfrIncrement + 1) % 3;
   int i2 = (directionBfrIncrement + 2) % 3;

   hallPhases a = directionBfr[i0];
   hallPhases b = directionBfr[i1];
   hallPhases c = directionBfr[i2];

   // Mögliche Vorwärtsfolgen (u → v → w, v → w → u, w → u → v)
   if ((a == u && b == v && c == w) ||
       (a == v && b == w && c == u) ||
       (a == w && b == u && c == v))
   {
      currentDirection = fwd;
   }

   // Mögliche Rückwärtsfolgen (u → w → v, v → u → w, w → v → u)
   if ((a == u && b == w && c == v) ||
       (a == v && b == u && c == w) ||
       (a == w && b == v && c == u))
   {
      currentDirection = bwd;
   }

   // Default Case
   currentDirection = unknown;
}

void Balancer::getPitch()
{
   currentPitch = gyroSensor.getWeightedPitch(PULSECOUNTTIME * 0.001);
}

void Balancer::motorOutput()
{
   if (abs(currentPitch) > 23.0) // maximaler Aufrichtwinkel 
   {
      flPWM = 0.0;
      applyMotor(flPWM);
      return;
   }
   else
   {
      flV = pid_phi.pidControl(-currentPitch);
      flPWM = pid_v.pidControl(flV - currentVelocity);
      applyMotor(flPWM);
      return;
   }
}

void Balancer::applyMotor(float val)
{
   val = constrain(val, -PWMMAX, PWMMAX);

   if (val == 0.0)
   {
      analogWrite(PWMPIN, 0);
   }
   else if (val < 0)
   {
      digitalWrite(DIRPIN, LOW);
      analogWrite(PWMPIN, -val);
   }
   else if (val > 0)
   {
      digitalWrite(DIRPIN,HIGH);
      analogWrite(PWMPIN, val);
   }
}