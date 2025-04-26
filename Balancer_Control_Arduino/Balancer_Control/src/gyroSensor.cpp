/**
 * @file mpu6050.cpp
 * @author Johannes Müller, Dominik Berenspoehler
 * @brief Implementation der Klasse GyroSensor zur Ansteuerung eines MPU6050-Sensors.
 * @version 0.1
 * @date 2025-04-06
 * @copyright Copyright (c) 2025
 */

#include "gyroSensor.h"

/**
 * @brief Konstruktor der GyroSensor-Klasse.
 */
GyroSensor::GyroSensor() {}

/**
 * @brief Initialisiert den MPU6050-Sensor.
 *
 * Startet die I2C-Kommunikation, initialisiert das MPU-Objekt und prüft die Verbindung.
 *
 * @return true bei erfolgreicher Verbindung, sonst false.
 */
bool GyroSensor::init()
{
   Wire.begin();
   mpu.initialize();
   bool statusOK = mpu.testConnection();

   return statusOK;
}

/**
 * @brief Liest die Beschleunigungswerte aus dem Sensor aus.
 *
 * Die Werte werden in das interne Array `accelerationValues` geschrieben.
 * Anschließend wird der Pitch-Winkel basierend auf der Beschleunigung berechnet.
 */
void GyroSensor::getAcceleration()
{
   mpu.getAcceleration(&accelerationValues[x], &accelerationValues[y], &accelerationValues[z]);
   getPitchfromAccel();
}

/**
 * @brief Liest die Gyroskop-Werte aus dem Sensor aus.
 *
 * Die Werte werden in das interne Array `gyroscopeValues` geschrieben.
 */
void GyroSensor::getGyro()
{
   mpu.getRotation(&gyroscopeValues[x], &gyroscopeValues[y], &gyroscopeValues[z]);
}

/**
 * @brief Berechnet den Pitch-Winkel anhand der aktuellen Beschleunigungswerte.
 *
 * Aufgrund der Sensorlage ergibt sich der Pitch um die Y-Achse des MPU6050.
 *
 * @return Der berechnete Pitch-Winkel in Grad.
 */
float GyroSensor::getPitchfromAccel()
{
   float ax = accelerationValues[x] * accelFactor;
   float ay = accelerationValues[y] * accelFactor;
   float az = accelerationValues[z] * accelFactor;

   pitch = atan2(ay, sqrt(ax * ax + az * az)) * angleFactor;

   return pitch;
}

/**
 * @brief Liefert den gefilterten Pitch-Winkel unter Verwendung eines Complementary Filters.
 *
 * Kombiniert die Werte des Beschleunigungssensors mit den Gyroskop-Daten zur robusteren Pitch-Schätzung.
 *
 * @param dt Zeitdifferenz in Sekunden seit der letzten Messung.
 * @return Der gefilterte Pitch-Winkel in Grad.
 */
float GyroSensor::getWeightedPitch(float dt)
{
   float ax = accelerationValues[x] * accelFactor;
   float ay = accelerationValues[y] * accelFactor;
   float az = accelerationValues[z] * accelFactor;
   float gy = gyroscopeValues[y] * gyroFactor;

   float accelPitch = atan2(az, sqrt(ax * ax + ay * ay)) * angleFactor;

   pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accelPitch;

   return pitch;
}