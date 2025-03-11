#include <Wire.h>
#include <MPU6050.h>
#include <avr/wdt.h>

MPU6050 mpu;

// Pins für Motorsteuerung
const int enblPin = 2;  // Rot
const int pwmPin = 3;   // Blau
const int dirPin = 4;   // Grün
const int brkPin = 5;   // Weiß
const int almPin = 6;   // Gelb

const int ctrlLED = 13; // On-Board-LED
const int tastPin = A0; // für Kalibrier-Taster

struct balancer {
  float   currentVelocity; 
  float   flPWM; // PWM Wert ergebnis aus dem dann Motor angesteuert wird
  MPU6050 mpu1;
  float   currentPitch; // von MPU Daten berechnen
}

typedef struct {
  float KP;
  float KI;
  float KD;
  float TA;
} pid_t

float xyzFactor = 0.000006103515625;
float angleFactor = 57.29577951;

int     pwmValue;
int     pwmMax;
int     direction;

void setup() {
  // Pins definieren
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  pinMode(brkPin, OUTPUT);
  pinMode(almPin, INPUT);

  // Kommunikation starten
  //Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  //MPU Verbindung überprüfen
  if (mpu.testConnection()) {
    //Serial.println("MPU 6050 verbunden");
  }
  else {
    //Serial.println("Verbindung zum MPU6050 fehlgeschlagen!");
    digitalWrite(ctrlLED, HIGH);
  }

  // Statische Offsetwerte des Sensors setzen
  mpu.setXAccelOffset(-1911);
  mpu.setYAccelOffset(1305);
  mpu.setZAccelOffset(3777);
  mpu.setXGyroOffset(87);
  mpu.setYGyroOffset(27);
  mpu.setZGyroOffset(31);

  // PWM-Frequenz für Timer 2 (Pin 3) für Driver auf ~1kHz setzen
  TCCR2B = TCCR2B & 0b11111000 | 0x03;

  // Bremse deaktivieren und Driver aktivieren
  digitalWrite(brkPin, HIGH);
  digitalWrite(enblPin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}
