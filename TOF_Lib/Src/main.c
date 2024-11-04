/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Andreas Ladner & Philipp Roehlke
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
//#include <mcalI2C.h>
/**
// General functions
extern I2C_RETURN_CODE_t i2cSelectI2C(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cDeselectI2C(I2C_TypeDef *i2c);

extern I2C_RETURN_CODE_t i2cSetClkSpd(I2C_TypeDef *i2c, I2C_CLOCKSPEED_t spd);
extern I2C_RETURN_CODE_t i2cEnableDevice(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cDisableDevice(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cSetPeripheralClockFreq(I2C_TypeDef *i2c, uint8_t pclk);
extern uint32_t          i2cGetPeripheralClockFrequ(I2C_TypeDef *i2c);
extern I2C_RETURN_CODE_t i2cSetDutyCycle(I2C_TypeDef *i2c, I2C_DUTY_CYCLE_t duty);
extern I2C_RETURN_CODE_t i2cSetRiseTime(I2C_TypeDef *i2c, uint8_t riseTime);

// Send functions
extern I2C_RETURN_CODE_t i2cSendByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t data);
extern I2C_RETURN_CODE_t i2cSendByteToSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t data);
extern I2C_RETURN_CODE_t i2cBurstWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data, uint8_t numBytes);

// Read functions
extern I2C_RETURN_CODE_t i2cReadByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data);
extern I2C_RETURN_CODE_t i2cReadByteFromSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data);
extern I2C_RETURN_CODE_t i2cBurstRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data, uint8_t num);
extern I2C_RETURN_CODE_t i2cBurstRegRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t num);

extern I2C_RETURN_CODE_t i2cResetDevice(I2C_TypeDef *i2c);
extern uint8_t           i2cFindSlaveAddr(I2C_TypeDef *i2c, uint8_t i2cAddr);
 */


// Definitionen von Adressen für Register, die für die Konfiguration und Steuerung benötigt werden.


//Wert kann von überall geändert werden und Wert kann so nicht von Compiler weg optimiert werden
//Es wird festgelegt auf welchem BUS welche Taktfrequenz laufen soll
//HIER: 16MHz internal (High Speed Internal)
#define RCC_AHB1ENR   (*(volatile uint32_t *)0x40023830)


//PIN 15 = Output und PIN 8 alternative Funktion
#define GPIOA_MODER   (*(volatile uint32_t *)0x40020000)


// GPIOA_ODR ist das Register, das den Ausgangszustand für GPIOA steuert (z.B. LED ein- oder ausschalten).
#define GPIOA_ODR     (*(volatile uint32_t *)0x40020014)


// GPIOC_MODER ist das Register, das den Modus für jeden Pin von GPIOC konfiguriert.
//PIN 5 = alternative Funktion
#define GPIOC_MODER   (*(volatile uint32_t *)0x40020800)


// GPIOC_IDR ist das Register, das den Eingangsstatus der Pins von GPIOC liest (z.B. ob der Taster gedrückt ist).
// BIT 13 von Port C = Eingang
#define GPIOC_IDR     (*(volatile uint32_t *)0x40020810)


// Eine einfache Funktion zur Verzögerung. Sie nimmt einen Zähler als Eingabe und läuft so lange, bis der Zähler 0 erreicht.
void delay(volatile int count) {
    while (count--);
}



int main(void) {
    // **1. Takt für GPIOA und GPIOC aktivieren**
    // Bevor wir GPIO-Pins verwenden können, müssen wir sicherstellen, dass die Peripherie mit Strom versorgt wird.
    // Durch das Setzen des ersten Bits von RCC_AHB1ENR aktivieren wir den Takt für GPIOA. (HSI = ON)
    RCC_AHB1ENR |= (1U << 0); // GPIOA
    // Durch das Setzen des dritten Bits von RCC_AHB1ENR aktivieren wir den Takt für GPIOC.
    RCC_AHB1ENR |= (1U << 2); // GPIOC

    // **2. Pin A5 als Ausgang konfigurieren (LED)**
    // Pin A5 (Bit 10 und 11 im MODER-Register) wird als Ausgang konfiguriert.
    // Das Setzen von Bit 10 auf 1 schaltet den Pin in den Ausgangsmodus.
    GPIOA_MODER |= (1U << 10);

    // **3. Pin C13 als Eingang konfigurieren (Taster)**
    // Pin C13 (Bit 26 und 27 im MODER-Register) wird als Eingang konfiguriert.
    // Durch das Löschen dieser beiden Bits (setzen auf 00) wird der Pin in den Eingangsmodus versetzt.
    GPIOC_MODER &= ~(3U << 26);

    // **4. Initiale Blinkgeschwindigkeit (mittel)**
    // Eine Variable 'speed' wird verwendet, um die aktuelle Blinkgeschwindigkeit zu speichern.
    // Sie wird zu Beginn auf 1 gesetzt, was die mittlere Geschwindigkeit bedeutet.
    int speed = 1;

    // **5. Endlosschleife zum Blinken der LED**
    while (1) {
        // **6. Überprüfen, ob der Taster gedrückt wurde (Low-Aktiv)**
        // Wenn der Taster gedrückt ist, wird der entsprechende Pin (C13) auf LOW gesetzt (0).
        // Der folgende Ausdruck prüft, ob Pin C13 auf 0 steht (Taster gedrückt).
        if (!(GPIOC_IDR & (1U << 13))) {
            // **7. Taster gedrückt - Blinkgeschwindigkeit ändern**
            // Die Geschwindigkeit wird zyklisch geändert:
            // 0 = langsam, 1 = mittel, 2 = schnell
            speed = (speed + 1) % 3; // Zyklisch zwischen 0, 1 und 2 wechseln

            // **8. Kurze Entprellungspause**
            // Nach dem Erkennen eines Tastendrucks wartet der Code kurz, um Entprellungen zu verhindern.
            delay(3000000);
        }

        // **9. LED ein**
        // Das Setzen von Bit 5 in GPIOA_ODR schaltet die LED ein.
        GPIOA_ODR |= (1U << 5);

        // **10. Verzögerung basierend auf der Geschwindigkeit**
        // Der Wert von 'speed' bestimmt, wie lange die Verzögerung dauert.
        if (speed == 0) {
            delay(3000000); // Langsam (lange Verzögerung)
        } else if (speed == 1) {
            delay(1000000); // Mittel (mittlere Verzögerung)
        } else {
            delay(300000);  // Schnell (kurze Verzögerung)
        }

        // **11. LED aus**
        // Das Löschen von Bit 5 in GPIOA_ODR schaltet die LED aus.
        GPIOA_ODR &= ~(1U << 5);

        // **12. Verzögerung basierend auf der Geschwindigkeit**
        // Erneut eine Verzögerung, bevor die LED wieder eingeschaltet wird.
        if (speed == 0) {
            delay(3000000); // Langsam
        } else if (speed == 1) {
            delay(1000000); // Mittel
        } else {
            delay(300000);  // Schnell
        }
    }
}

