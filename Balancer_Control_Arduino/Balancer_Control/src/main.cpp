#include <Arduino.h>
#include "defines.h"
#include "balancer.h"

Balancer bala; // dein Balancer-Objekt
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
  bala = Balancer();
  Balancer::instance = &bala;

  int inputs[ALMPIN, VELOPIN];
  int outputs[ENBLPIN, DIRPIN, PWMPIN, BRKPIN, CTRLLEDPIN, PINU, PINV, PINW];

  for (int i = 0; i < sizeof(inputs); i++)
  {
    pinMode(inputs[i], INPUT);
  }
  for (int i = 0; i < sizeof(outputs); i++)
  {
    pinMode(outputs[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(VELOPIN), pulseISR, RISING);

  attachInterrupt(digitalPinToInterrupt(PINU), Balancer::recogniseHallPulseU, RISING);
  attachInterrupt(digitalPinToInterrupt(PINV), Balancer::recogniseHallPulseV, RISING);
  attachInterrupt(digitalPinToInterrupt(PINW), Balancer::recogniseHallPulseW, RISING);
}

void loop()
{
  if (millis() - lastMillis >= PULSECOUNTTIME)
  {
    bala.getVelocity(pulseCount);
    bala.getPitch();
    pulseCount = 0;
    lastMillis = millis();
  }

  Serial.print("Geschwindigkeit: ");
  Serial.println(bala.currentVelocity);

  if (Balancer::instance->newDirectionEvent)
  {
    direction dir = Balancer::instance->getDirection();
    Serial.print("Drehrichtung: ");
    Serial.println(dir == fwd ? "Vorwärts" : "Rückwärts");

    // Event zurücksetzen, nachdem es verarbeitet wurde
    Balancer::instance->newDirectionEvent = false;
  }
}