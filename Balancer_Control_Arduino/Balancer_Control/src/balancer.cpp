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
 Balancer::Balancer()
 {
    MPU6050 _mpu;
    PIDController _pid(0, 0, 0, 1/PULSECOUNTTIME);
    init(_mpu, _pid);
 }

 /**
  * @brief Erzeugt ein neues Objekt der Klasse Balancer mit vorinitialisertem
  * MPU6050-Sensor und PID-Regler.
  * 
  * @param _mpu MPU6050 Sensor
  * @param _pid PID Regler
  */
 Balancer::Balancer(MPU6050 _mpu, PIDController _pid)
 {
    init(_mpu, _pid);
 }

 void Balancer::init(MPU6050 _mpu, PIDController _pid)
 {
    gyroSensor = _mpu;
    pid = _pid;
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

