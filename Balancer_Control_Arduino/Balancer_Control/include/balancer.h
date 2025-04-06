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
 #include "mpu6050.h"

 enum sensorOutput 
 {
    accelX,
    accelY,
    accelZ,
    gyroX,
    gyroY,
    gyroZ
 };

 class Balancer
 {
    public:
        Balancer();
        Balancer(MPU6050 _mpu, PIDController _pid);
        void init(MPU6050 mpu, PIDController pid);
        void getVelocity(int pulsecount);
        void getPitch();
        void motorOutput();

    private:
        int16_t sensorBuffer[6];
        MPU6050 gyroSensor;                 //TODO: MPU einbinden
        PIDController pid;                  
        float currentVelocity;              ///< Aktuelle Geschwindigkeit
        float flPWM;                        ///< Ergebnis des Reglers, mit dem der Motor angesteuert wird
        float currentPitch;                 ///< Aktueller Pitch-Wert
        float velocityFactor;               ///< Umrechnungsfaktor für die Geschwindigkeitsberechnung
 };

 #endif // BALANCER