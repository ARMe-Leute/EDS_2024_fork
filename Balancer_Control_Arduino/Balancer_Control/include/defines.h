/**
 * @file defines.h
 * @author Johannes Mueller, Dominik Berenspoehler
 * @brief Definition der Pins am Arduino zur klareren Lesbarkeit
 * @version 0.1
 * @date 2025-04-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#define ENBLPIN 2     ///< Motor freischalten, Rotes Kabel
#define ALMPIN 3      ///< Fehler Ausgang des BLDC-Controllers, Gelbes Kabel
#define DIRPIN 4      ///< Richtung umschalten, Grünes Kabel
#define VELOPIN 5     ///< Frequenzeingang Geschwindigkeit des Motors
#define PWMPIN 6      ///< PWM Signal für den BLDC-Controller, Blaues Kabel
#define BRKPIN 7      ///< Bremse des Motors, Weißes Kabel
#define PINU 8        ///< Eingang Hall-Signal U
#define PINV 9        ///< Eingang Hall-Signal V
#define PINW 10       ///< Eingang Hall-Signal W
#define CTRLLEDPIN 13 ///< On-Board LED für Debug-Zwecke

#define PULSECOUNTTIME 25 ///< ms Intervall, in dem die Pulse gezählt werden
#define PWMMAX 20         ///< Maximaler PWM Wert