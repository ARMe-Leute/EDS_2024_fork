/**
 * @file main.cpp
 * @author Johannes Müller
 * @brief Hauptprogramm zur Steuerung des Balancers.
 * @version 0.1
 * @date 2025-04-26
 * 
 * Initialisiert Hardware und Timer, setzt die ISR für Geschwindigkeit und Hall-Sensoren,
 * und steuert zyklisch den Balancer über eine PID-Regelung.
 */

 #include <Arduino.h>
 #include "defines.h"
 #include "balancer.h"
 
 /** Zeiger auf die Balancer-Instanz */
 Balancer *balaPtr;
 
 /** Pulszähler für Geschwindigkeitsmessung */
 int pulseCount;
 
 /** Zeitstempel der letzten Verarbeitung */
 volatile unsigned long lastMillis = 0;
 
 /** Zykluszeit in Sekunden */
 float deltaTime = PULSECOUNTTIME * 0.001;
 
 /**
  * @brief ISR für Geschwindigkeitspulse.
  * 
  * Erhöht den pulseCount bei jedem registrierten Geschwindigkeitspuls.
  */
 void pulseISR()
 {
   pulseCount++;
 }
 
 /**
  * @brief Initialisierung der Hardware-Komponenten.
  * 
  * Setzt Eingänge, Ausgänge, Interrupts und initialisiert den Balancer.
  */
 void setup()
 {
   Serial.begin(9600);
   balaPtr = new Balancer();
   Balancer::instance = balaPtr;
 
   int inputs[5]{ALMPIN, VELOPIN, PINU, PINV, PINW};
   int outputs[5]{ENBLPIN, DIRPIN, PWMPIN, BRKPIN, CTRLLEDPIN};
 
   for (int i = 0; i < 5; i++)
   {
     pinMode(inputs[i], INPUT);
   }
   for (int i = 0; i < 5; i++)
   {
     pinMode(outputs[i], OUTPUT);
   }
 
   attachInterrupt(digitalPinToInterrupt(VELOPIN), pulseISR, RISING);
   attachInterrupt(digitalPinToInterrupt(PINU), Balancer::recogniseHallPulseU, RISING);
   attachInterrupt(digitalPinToInterrupt(PINV), Balancer::recogniseHallPulseV, RISING);
   attachInterrupt(digitalPinToInterrupt(PINW), Balancer::recogniseHallPulseW, RISING);
 
   digitalWrite(BRKPIN, HIGH); // Motorbremse deaktivieren
   digitalWrite(ENBLPIN, LOW); // Motorcontroller freischalten
 }
 
 /**
  * @brief Hauptprogrammschleife.
  * 
  * Führt die zyklische Aktualisierung der Sensordaten, der Geschwindigkeit,
  * der Richtungserkennung und der Motorregelung aus.
  */
 void loop() 
 {
   // Zeitbasis prüfen
   unsigned long now = millis();
   if (now - lastMillis >= PULSECOUNTTIME) {
       lastMillis = now;
 
       // Geschwindigkeit ermitteln (pulseCount wird im ISR inkrementiert)
       Balancer::instance->getVelocity(pulseCount);
       pulseCount = 0;  // Zähler zurücksetzen
 
       // Richtungserkennung wenn neues Hall-Event
       if (Balancer::instance->newDirectionEvent) {
           Balancer::instance->getDirection();          // setzt currentDirection
           Balancer::instance->newDirectionEvent = false;
       }
 
       // Pitch neu berechnen
       Balancer::instance->getPitch();
 
       // Regelung ausführen und Motor ansteuern
       Balancer::instance->motorOutput();
   }
 
   // Debug-Ausgaben
   // Serial.print("Geschwindigkeit:\t");
   // Serial.println(Balancer::instance->currentVelocity);
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