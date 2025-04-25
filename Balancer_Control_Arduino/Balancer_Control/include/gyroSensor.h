/**
 * @file mpu6050.h
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Deklaration der Klasse MPU6050
 * @version 0.1
 * @date 2025-04-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #ifndef MPU6050SENSOR
 #define MPU6050SENSOR

 #include "defines.h"
 #include "Wire.h"
 #include "I2Cdev.h"
 #include "MPU6050.h"

 enum sensorValueIndex
 {
   x,
   y,
   z
 };

 class GyroSensor
 {
    private:
      MPU6050 mpu;
      int16_t accelerationValues[3];
      int16_t gyroscopeValues[3];
      float alpha = 0.98;
      float pitch;
      float yaw;
      float roll;
      float accelFactor = 0.00006103515625;
      float gyroFactor = 0.007633587786;
      float angleFactor = 57.29577951;

    public:
      GyroSensor();
      bool init();
      void getAcceleration();
      void getGyro();
      float getPitchfromAccel();
      float getWeightedPitch(float dt);
 };

 #endif