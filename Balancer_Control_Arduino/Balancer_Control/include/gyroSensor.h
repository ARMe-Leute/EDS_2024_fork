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

/**
 * @brief Klasse zur Verarbeitung von MPU6050-Daten.
 *
 * Diese Klasse kapselt die Initialisierung und das Auslesen von Beschleunigungs-
 * und Gyroskopdaten des MPU6050-Sensors. Zudem bietet sie Methoden zur Berechnung
 * des Pitch-Winkels.
 */
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
  /**
   * @brief Konstruktor der GyroSensor-Klasse.
   */
  GyroSensor();

  /**
   * @brief Initialisiert den MPU6050-Sensor.
   * @return true bei erfolgreicher Initialisierung, sonst false.
   */
  bool init();

  /**
   * @brief Liest die aktuellen Beschleunigungswerte vom Sensor.
   */
  void getAcceleration();

  /**
   * @brief Liest die aktuellen Gyroskopwerte vom Sensor.
   */
  void getGyro();

  /**
   * @brief Berechnet den Pitch-Winkel auf Basis der Beschleunigungswerte.
   * @return Der berechnete Pitch-Winkel in Grad.
   */
  float getPitchfromAccel();

  /**
   * @brief Berechnet einen gefilterten Pitch-Wert mit Hilfe eines Complementary Filters.
   * @param dt Zeitdifferenz in Sekunden seit der letzten Abfrage.
   * @return Der gefilterte Pitch-Wert in Grad.
   */
  float getWeightedPitch(float dt);

  float pitch; ///< Letzter berechneter Pitch-Wert
  float yaw;   ///< Platzhalter für Yaw-Wert
  float roll;  ///< Platzhalter für Roll-Wert
};

#endif