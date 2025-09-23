Dieses Projektverzeichnis enthält die im Rahmen der Vorlesung Entwurf Digitaler Systeme erstellte Funktionsbibliothek zur Nutzung eines MPU6050-Sensors mit dem bereitgestellten ARM-Controller.

Die Funktionsbibliothek besteht aus den Dateien MPU6050.h und MPU6050.c. Detaillierte Beschreibungen der Funktionen sind in der Datei MPU6050.c enthalten.

Die Datei main.c enthält ein Programm zur Überprüfung der Funktionalität des Gyrosensors mit dem ARM-Controller. Dabei werden die Winkel der X- und Y-Achse des Sensors relativ zur Z-Achse zyklisch ausgelesen und auf dem Piggybag-Display mithilfe der vorimplementierten Oszi-Funktion der BALO-Bibliothek dargestellt. Überschreitet der Winkel 10°, ändert die LED des Drehdruck-Schalters ihre Farbe von Grün zu Cyan. In der obersten Zeile des Displays wird die vom Sensor gemessene Temperatur in °C angezeigt.

Die Funktionsbeschreibungen und Kommentare wurden mithilfe künstlicher Intelligenz erstellt, auf ihre Richtigkeit überprüft und bei Bedarf entsprechend korrigiert.