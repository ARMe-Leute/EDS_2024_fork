/**
 * @file mpu6050.h
 * @author Johannes Müller, Dominik Berenspoehler
 * @brief Deklaration der Klasse GyroSensor zur Verwendung des MPU6050.
 * @version 0.1
 * @date 2025-04-06
 * @copyright Copyright (c) 2025
 */

#ifndef MPU6050SENSOR
#define MPU6050SENSOR

#include "defines.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

/**
 * @brief Indizes für die Achsen x, y, z.
 */
enum sensorValueIndex
{
  x, ///< Index für X-Achse
  y, ///< Index für Y-Achse
  z  ///< Index für Z-Achse
};

class GyroSensor
{
private:
  MPU6050 mpu;                          ///< Objekt zur Kommunikation mit dem MPU6050
  int16_t accelerationValues[3];        ///< Array für Rohwerte der Beschleunigung
  int16_t gyroscopeValues[3];           ///< Array für Rohwerte des Gyroskops
  float alpha = 0.98;                   ///< Filtergewichtung (Gyro vs. Accel)
  float accelFactor = 0.00006103515625; ///< Umrechnungsfaktor für Beschleunigung (LSB → g)
  float gyroFactor = 0.007633587786;    ///< Umrechnungsfaktor für Gyroskop (LSB → °/s)
  float angleFactor = 57.29577951;      ///< Umrechnung von Bogenmaß in Grad

public:
  GyroSensor();
  bool init();
  void getAcceleration();
  void getGyro();
  float getPitchfromAccel();
  float getWeightedPitch(float dt);

  float pitch; ///< Letzter berechneter Pitch-Wert
  float yaw;   ///< Platzhalter für Yaw-Wert
  float roll;  ///< Platzhalter für Roll-Wert
};

#endif