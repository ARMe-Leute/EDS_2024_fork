README: Time-of-Flight (TOF) Sensor Library (angepasst fuer den Sensor VL53L0X)

Inhaltsverzeichnis
1. Ueberblick
2. Neuerungen
3. Sonstiges
<<<<<<< HEAD

Ueberblick: 
Als Grundlage der Library wurde die bereits 2023 erstellte Version verwendet. Dabei wurden die bestehenden Funktionen ueberarbeitet und angepasst. 
Des Weiteren wurde ein Testprogramm entwickelt.

Neuerungen: 
Strukturen: 
Hier sind die Funktionen der neu implementierten Sensorstrukturen aufgefuehrt. 
Mithilfe dieser Strukturen koennen mehrere Sensoren ueber verschiedene I2C-Ports angesprochen und ausgelesen werden. 
Die Funktion initializeTOFSensor ermoeglicht es, einen neu angelegten Sensor zu initialisieren. Dabei werden verschiedene Parameter und Einstellungen festgelegt. 
Diese Funktion wird nur einmal bei der Erstellung des Sensors aufgerufen. Mit der Funktion configureTOFSensor koennen waehrend der Benutzung des Sensors einzelne Parameter, 
wie beispielsweise der Messmodus, geaendert werden.

1. **initializeTOFSensor**  
   Mit dieser Funktion kann ein neu angelegter Sensor initialisiert werden. Hierbei werden verschiedenen Parameter / Einstellungen festgelegt. 
   Diese Funktion wird nur einmal bei der Erstellung aufgerufen. 

2. **configureTOFSensor**  
   Waehrend der Benutzung der Sensoren koennen einzelne Parameter wie zum Beispiel der Messmodus geaendert werden. Dies passiert ueber die Funktion configureTOFSensor.
=======
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
>>>>>>> d6a2710a976d16c97df9f3580c9ee7531cbe5b1a


Funktionen:
Die folgenden Funktionen wurden in die Bibliothek aufgenommen:

1. **TOF_set_address**  
   Aendert die I2C-Adresse des TOF-Sensors.

<<<<<<< HEAD
2. **TOF_read_distance_timed**  
   Fuehrt eine Distanzmessung durch, nachdem eine angegebene Zeit abgewartet wurde.
   (! Noch nicht implementiert)

3. **TOF_set_ranging_profile**  
   Stellt den Messmodus des Sensors ein, zum Beispiel fuer hohe Geschwindigkeit oder Genauigkeit.

4. **TOF_set_vcsel_pulse_period**  
   Konfiguriert die VCSEL-Pulsperiode fuer Pre- oder Final-Range-Messphasen.

5. **TOF_set_signal_rate_limit**  
   Setzt die Signalratenbegrenzung, um die Messgenauigkeit und Reichweite anzupassen.

6. **TOF_get_sequence_step_timeouts**  
   Liest die Timeout-Werte fuer einzelne Messsequenzen aus.

7. **TOF_get_sequence_step_enables**  
   Liest aus, welche Messschritte (z. B. PRE_RANGE oder FINAL_RANGE) aktiviert sind.

8. **TOF_get_vcsel_pulse_period**  
   Ruft die VCSEL-Pulsperiode fuer Pre- oder Final-Range-Modi ab.

9. **TOF_set_measurement_timing_budget**  
   Stellt das Zeitbudget fuer eine Messung ein, das die Gesamtzeit pro Messzyklus definiert.

10. **encode_timeOut**  
    Kodiert einen Timeout-Wert in ein Registerformat, das der Sensor verwendet.

11. **decode_timeout**  
    Dekodiert einen Timeout-Wert aus dem Registerformat in die tatsaechliche Zeit in MCLKs.

12. **timeoutMclksToMicroseconds**  
    Wandelt einen Timeout-Wert von MCLKs (Makro-Clocks) in Mikrosekunden um.

Sonstiges: 
Im Rahmen der Bearbeitung wurden zu Testzwecken die Dateien visualisation.c und visualisation.h geaendert. 
Dies war nicht Teil der Aufgabe, aber es ist relevant, dies zu erwaehnen. Das erstellte Testprogramm konnte nicht vollstaendig fertiggestellt werden, 
jedoch wurden die einzelnen Funktionen getestet und sind funktionsfaehig. Bei der Erstellung der Kommentare in der Datei SensorTOF.h wurde Kuenstliche Intelligenz verwendet. 
Die Ergebnisse wurden auf ihre Korrektheit ueberprueft. In der abgegebenen ZIP-Datei befinden sich ausserdem alle verwendeten Bibliotheken sowie das erstellte Testprogramm.
=======
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
 

>>>>>>> d6a2710a976d16c97df9f3580c9ee7531cbe5b1a

Autor:
Philipp Roehlke 
Andreas Ladner

