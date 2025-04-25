/**
 * @file mpu6050.cpp
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Implementation der Klasse MPU6050
 * @version 0.1
 * @date 2025-04-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "gyroSensor.h"

 //TODO: Code für MPU mit der Wire.h library zum Laufen bekommen
 GyroSensor::GyroSensor() {}

 bool GyroSensor::init()
 {
    Wire.begin();
    mpu.initialize();
    bool statusOK = mpu.testConnection();
    
    return statusOK;
 }

 void GyroSensor::getAcceleration()
 {
    mpu.getAcceleration(&accelerationValues[x], &accelerationValues[y], &accelerationValues[z]);
    getPitchfromAccel();
 }

 void GyroSensor::getGyro()
 {
    mpu.getRotation(&gyroscopeValues[x], &gyroscopeValues[y], &gyroscopeValues[z]);
 }

/**
 * @brief Berechnung des Pitch-Winkels
 * 
 * Aufgrund der Montageposition dreht der Pitch um die Z-Achse des MPU Sensors.
 * 
 */
 float GyroSensor::getPitchfromAccel()
 {
    float ax = accelerationValues[x] * accelFactor;
    float ay = accelerationValues[y] * accelFactor;
    float az = accelerationValues[z] * accelFactor;

    pitch = atan2(ax, sqrt(ay*ay + az*az)) * angleFactor;

    return pitch;
 }

 float GyroSensor::getWeightedPitch(float dt)
 {
    float ax = accelerationValues[x] * accelFactor;
    float ay = accelerationValues[y] * accelFactor;
    float az = accelerationValues[z] * accelFactor;
    float gz = gyroscopeValues[z] * gyroFactor;

    float accelPitch = atan2(ax, sqrt(ay*ay + az*az)) * angleFactor;

    pitch = alpha * (pitch + gz * dt) + (1 - alpha) * accelPitch;

    return pitch;
 }