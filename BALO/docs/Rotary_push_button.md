
# Abgabe Project ARM-Controller Gruppe: Dreh-Druck-Taster

<b>Autoren:</b> Linus Blanke, Christoph Lederbogen

Die Abgabe beinhaltet zwei STM-Projekte:
1. Balancer
2. Drehgeber

Die Implementierung wurde in C und C++ vorgenommen. <br>
Zum Starten in *STM32-CUBE-IDE* genügt es, die ```.project```-Datei im Abgabe-Verzeichnis zu öffnen.

## Balancer
Balancer beinhaltet die implementierten Funktionalitäten für den Dreh-Drucktaster. Diese befinden sich in den Dateien: ```Balancer\Inc\RotaryPushButton.h``` und ```Balancer\Src\RotaryPushButton.c``` sowie die Dateien: ```Balancer\Inc\RotaryPushButtonCpp.hpp``` und ```Balancer\Src\RotaryPushButtonCpp.cpp```. Diese Dateien sind nach dem MISRA-Standard aufgebaut und kommentiert.

## Drehgeber
Das Projekt Drehgeber beinhaltet eine ausführbare ```Drehgeber\Src\main.c```, welche ein kurzes Beispiel zur Verwendung des Dreh-Drucktasters mit dem C-Code beinhaltet. <br>
Desweiteren beinhaltet das Projekt die C++-Datei ```Drehgeber\currently_unused\main.cpp```. Diese Datei ist das gleiche Beispiel in C++ implementiert.

## Switch von C zu C++
Um das Beispiel in C++ zu starten muss folgendes beachtet werden:
1. *Debug*-Ordner mit dem Verzeichnis ```Drehgeber\Debug``` löschen.
2. Die ```main.c``` von ```Drehgeber\Src``` nach ```Drehgeber\currently_unused``` verschieben.
3. Die ```main.cpp``` von ```Drehgeber\currently_unused``` nach ```Drehgeber\Src``` verschieben.
4. in *STM32CubeIDE* Rechtsklick auf das Projekt *Drehgeber* und im Kontextmenü *Convert to C++* auswählen.

Um von C++ zu C zu wechseln, einfach alle Schritte rückwärts machen.

## Achtung!
in der MCAL ist ein kleiner Fehler, der das ausführen in C++ nicht möglich macht. Dieser muss vor dem Starten behoben werden! <br>
In der Datei ```MCAL_F40x\Inc\mcalRCC.h``` muss die Zeile *117* von:
```c
#ifdef cplusplus
```
zu:
```c
#ifdef __cplusplus
```
geändert werden.

## Eigenleistung
Die Ordner ```Balancer``` und ```Drehgeber``` beinhalten den von den genannten Autoren entwickelten Code. Die Bibliotheken ```CMSIS``` und ```MCAL_F40x``` sind nicht von den oben genannten Autoren entwickelt, auf sie wird zugegriffen und da sich ein Fehler in der ```MCAL_F40x``` befindet werden diese in dieser Abgabe mitgeliefert, damit ist der Fehler korrigiert und getestet.