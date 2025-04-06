#include <Arduino.h>
#include "defines.h"
#include "balancer.h"

Balancer bala;
int pulseCount;
volatile unsigned long lastMillis = 0;

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
  int inputs[ALMPIN, VELOPIN];
  int outputs[ENBLPIN, DIRPIN, PWMPIN, BRKPIN, CTRLLEDPIN];

  for (int i = 0; i < sizeof(inputs); i++)
  {
    pinMode(inputs[i], INPUT);
  }
  for (int i = 0; i < sizeof(outputs); i++)
  {
    pinMode(outputs[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(VELOPIN), pulseISR, RISING);
}

void loop() {
  if (millis() - lastMillis >= PULSECOUNTTIME)
  {
    bala.getVelocity(pulseCount);
    pulseCount = 0;
    lastMillis = millis();
  }
}