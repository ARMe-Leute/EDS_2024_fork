README: Time-of-Flight (TOF) Sensor Library (angepasst fuer den Sensor VL53L0X)

Inhaltsverzeichnis
1. Ueberblick
2. Neuerungen
3. Sonstiges
4. MCAL Fehlerbehandlung

Ueberblick: 
Im zweiten Teil des Projekts TOF des 6. Semesters wurden neue Funktionen implementiert. Des weiteren wurde die 
Uebersichtlichkeit verbessert und Verbesserungen am Code vorgenommen. Allgemein wurden die Ansaetze des 5. Semesters uebernommen und fortgefuert.  

Neuerungen: 
1. Die genauen Aenderungen enthalten ein neues Registersystem in dem alle Adressen einsortiert sind. 
2. Ausserdem sind Standardbefehle fuer die Initialisierung in einem Enum gespeichert. 
3. Die Adressen von moeglicherweise mehreren TOF Sensoren lassen sich abaendern. 
4. Die Funktionsbeschreibungen in der SensorTOF.h wurden ueberarbeitet. 
5. Funktion bool TOF_getMeasurement(TOFSensor_t* TOFSENS, uint16_t *range) wurde ueberarbeit (Speicherung und Verarbeitung von Daten)
6. Funktionsbeschreibung in der SensorTOF.c wurden ergaenzt. 


Funktionen:
Die folgenden Funktionen wurden in die Bibliothek aufgenommen:

1. **TOF_set_address**  
   Aendert die I2C-Adresse des TOF-Sensors.

2. **bool TOF_start_up_task(TOFSensor_t* TOFSENS)**  
   Beginnt eine Messung fue dir Folgefunktion TOF_read_distance_task, fuer das "Briefkastensystem". 

3. **bool TOF_read_distance_task(TOFSensor_t* TOFSENS)**  
   Ueberprueft ob ein neues Messergebnis verfuegbar ist und liest dieses ggf. aus. Sofern keine neue Messung vorliegt wird ein Zyklus gewartet. 


Sonstiges: 
Im Rahmen der Bearbeitung wurden zu Testzwecken die Dateien visualisation.c und visualisation.h geaendert. 
Das Testprogramm wurde in sofern abgeaendert, dass das neue Briefkastensystem im normaln Messmodus getestet werden kann. 
Dieser Teil ist jedoch auskommentiert (Z. 276 ff). 

MCAL Fehlerbehandlung:
Wenn das Testprogramm des TOF Sensors gestartet wird und ein Sensor gefunden wird, haengt sich das Programm bei initlialiserung auf. 

Ueberfunktion: TOF_init(TOFSENS)
main.c   Zeile: 471 

Bei detaillierter Analyse laesst sich das Problem auf das Unterprogramm eingrenzen. 

Funktion: TOF_init_device(TOFSENS)
SensorTOF.c    Zeile. 897

Wenn man weiter debuggt, laesst sich das Problem noch genaue lokalisieren.  

Funktion: TOF_perform_ref_calibration(TOFSENS)
SensorTOF.c    Zeile. 786

In der Funktion treten dann unterschieldiche Kommunikationsfehler auf. 

Funktion: bool TOF_perform_ref_calibration(TOFSensor_t* TOFSENS)
SensorTOF.c    Zeile. 707

Funktion: TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_VHV)  funktioniert nach aktuellem Stand. 

Funktion: TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_PHASE) bleibt dann in der Schleife innerhalb haengen. 

Genaue Funktionsbezeichnung: bool TOF_perform_single_ref_calibration(TOFSensor_t* TOFSENS, TOF_calibration_type_t calib_type)
SensorTOF.c    Zeile. 620

Schleife in der sich die Kommunikation dann schlussendlich aufhaengt:

    /* Wait for interrupt */

    uint8_t interrupt_status = 0;
    do {
        success = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (success == I2C_OK && ((interrupt_status & 0x07) == 0));
    if (success != I2C_OK)
    {
        return false;
    }

SensorTOF.c    Zeile. 653 bis 662

Der Befehl i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status) 
funktioniert soweit, jedoch kann kein Ergebnis aus dem Register ausgelesen werden, was zum abschließen dieser Schleife fuehren wuerde. 

Wie wurde getestet ? 
TOF angeschlossen an den PC, im Debug Modus gestartet.
Resume bis main.c Z.471 ab hier step by step mit Step into, jede einzelne Funktion durch probiert. 
! Fehler tritt auch auf wenn man das Programm "normal" auf den Controller hochlaedt (mit blinkender LED getestet). 
! Fehler tritt auch auf wenn verschiedene Kommunikationsgeschwindigkeiten verwendet werden. 

Wenn der TOF nicht angeschlossen wurde tritt kein Fehler auf, da er nicht initialisiert werden muss. 
 


Autor:
Philipp Roehlke 
Andreas Ladner

