README: Time-of-Flight (TOF) Sensor Library (angepasst fuer den Sensor VL53L0X)

Inhaltsverzeichnis
1. Ueberblick
2. Neuerungen
3. Sonstiges

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


Funktionen:
Die folgenden Funktionen wurden in die Bibliothek aufgenommen:

1. **TOF_set_address**  
   Aendert die I2C-Adresse des TOF-Sensors.

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

Autor:
Philipp Roehlke 
Andreas Ladner

