#include <Wire.h>
#include <MPU6050.h>
#include <FreqCount.h>
#include <avr/wdt.h>

MPU6050 mpu;

// Pins für Motorsteuerung
const int enblPin = 2;    // Rot
const int pwmPin  = 6;    // Blau -- ACHTUNG - UMKLEMMEN
const int dirPin  = 4;    // Grün
const int brkPin  = 7;    // Weiß -- ACHTUNG - UMKLEMMEN
const int almPin  = 3;    // Gelb -- ACHTUNG - UMKLEMMEN
const int veloPin = 5;    // Color to be defined, Frequenzeingang

const int ctrlLED = 13;   // On-Board-LED
const int tastPin = A0;   // für Kalibrier-Taster

typedef struct {
  float   currentVelocity; // Auslesen aus Controller
  float   flPWM; // PWM Wert ergebnis aus dem dann Motor angesteuert wird
  MPU6050 mpu1;
  float   currentPitch; // von MPU Daten berechnen
} balancer_t;

typedef struct {
  float KP;       // Proportional Coefficient
  float KI;       // Integral     Coefficient
  float KD;       // Differential Coefficient
  float ISum;     // Integral Sum fpr PID
  float posISum;  // Integral Sum to get from Velocity to position
  float TA;       // Cycle Time sec
  float inpOld;   // Last Input Value
} pid_t;

int16_t sensorBuffer[6];
enum sensorOutput {
  accelX,
  accelY,
  accelZ,
  gyroX,
  gyroY,
  gyroZ
}; 

float xyzFactor = 0.000006103515625;
float angleFactor = 57.29577951;
float velocityFactor = 3.33333333; // Aus Datenblatt: 60 / (6 * 3 Polpaare)

int     pwmValue;
int     pwmMax;
int     direction;
unsigned long freq;

void setup() 
{
  // Pins initialisieren
  pinMode(pwmPin,   OUTPUT);
  pinMode(dirPin,   OUTPUT);
  pinMode(enblPin,  OUTPUT);
  pinMode(brkPin,   OUTPUT);
  pinMode(almPin,   INPUT);
  pinMode(veloPin,  INPUT);

  // Kommunikation starten
  //Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  FreqCount.begin(100);

  //MPU Verbindung überprüfen
  if (mpu.testConnection()) 
  {
    //Serial.println("MPU 6050 verbunden");
  }
  else 
  {
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
  TCCR2B = TCCR2B & 0b11111000 | 0x03; // ACHTUNG: anpassen auf neuen PWM Pin!

  // Bremse deaktivieren und Driver aktivieren
  digitalWrite(brkPin, HIGH);
  digitalWrite(enblPin, LOW);
}

void loop() 
{
  // put your main code here, to run repeatedly:

}

void getCurrentVelocity(balancer_t bala) 
{
  if(FreqCount.available) 
  {
    freq = FreqCount.read()*10; // Zählintervall = 100ms,Umrechnung in Hz
    bala.currentVelocity = freq * velocityFactor;
  }
}

void getPitch(balancer_t bala) 
{
  bala.mpu1.getMotion6(&sensorBuffer[accelX],
  &sensorBuffer[accelY],
  &sensorBuffer[accelZ],
  &sensorBuffer[gyroX],
  &sensorBuffer[gyroY],
  &sensorBuffer[gyroZ]); // read accel and gyro values
  // TODO: Mit Funktion füllen
}

float pidControl(pid_t pid, float diff) 
{
  pid.posISum += diff;
  if (pid.KI == 0) 
  {
    pid.ISum = 0;
  }
  else 
  {
    pid.ISum += pid.posISum;
  }
  float result = (pid.KP * pid.posISum) + (pid.KI * pid.ISum * pid.TA) + (pid.KD / pid.TA) * (pid.posISum - pid.inpOld);
  pid.inpOld = pid.posISum;
  return result;
}
