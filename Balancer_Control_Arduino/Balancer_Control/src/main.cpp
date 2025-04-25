#include <Arduino.h>
#include "defines.h"
#include "balancer.h"

Balancer *balaPtr;
int pulseCount;
volatile unsigned long lastMillis = 0;
float deltaTime = PULSECOUNTTIME * 0.001;

/**
 * @brief Interrupt Service Routine
 *
 * Wird eine steigende Flanke am Geschwindigkeits-Eingang erkannt, wird der Pulszähler um 1 erhöht.
 *
 */
void pulseISR()
{
  pulseCount++;
}


void setup()
{
  Serial.begin(9600);
  balaPtr = new Balancer();
  Balancer::instance = balaPtr;

  int inputs[5]{ALMPIN, VELOPIN, PINU, PINV, PINW};
  int outputs[5]{ENBLPIN, DIRPIN, PWMPIN, BRKPIN, CTRLLEDPIN};

  for (int i = 0; i < 2; i++)
  {
    pinMode(inputs[i], INPUT);
  }
  for (int i = 0; i < 8; i++)
  {
    pinMode(outputs[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(VELOPIN), pulseISR, RISING);

  attachInterrupt(digitalPinToInterrupt(PINU), Balancer::recogniseHallPulseU, RISING);
  attachInterrupt(digitalPinToInterrupt(PINV), Balancer::recogniseHallPulseV, RISING);
  attachInterrupt(digitalPinToInterrupt(PINW), Balancer::recogniseHallPulseW, RISING);

  digitalWrite(BRKPIN, HIGH); // Motorbremse deaktivieren
  digitalWrite(ENBLPIN, LOW);
}

void loop() 
{
  // 1) Zeitbasis prüfen
  unsigned long now = millis();
  if (now - lastMillis >= PULSECOUNTTIME) {
      lastMillis = now;

      // 2) Geschwindigkeit ermitteln (pulseCount wird im ISR inkrementiert)
      Balancer::instance->getVelocity(pulseCount);
      pulseCount = 0;  // Zähler zurücksetzen

      // 3) Richtungserkennung wenn neues Hall-Event
      if (Balancer::instance->newDirectionEvent) {
          Balancer::instance->getDirection();          // setzt currentDirection
          Balancer::instance->newDirectionEvent = false;
      }

      // 4) Pitch neu berechnen
      Balancer::instance->getPitch();

      // 5) Regelung ausführen und Motor ansteuern
      Balancer::instance->motorOutput();
  }

  // 6) Debug-Ausgaben
  //Serial.print("Geschwindigkeit:\t");
  //Serial.println(Balancer::instance->currentVelocity);
  // Serial.print("Winkel:\t");
  // Serial.println(Balancer::instance->currentPitch);
  // Serial.print("flV\t");
  // Serial.println(Balancer::instance->flV);
  Serial.print("flPWM\t");
  Serial.println(constrain(Balancer::instance->flPWM, -PWMMAX, PWMMAX));
  /*
  Serial.print("Drehrichtung:\t");
  switch (Balancer::instance->currentDirection) {
    case fwd:    Serial.println("Vorwärts");  break;
    case bwd:    Serial.println("Rückwärts"); break;
    default:     Serial.println("Unknow");    break;
  }
  */
}