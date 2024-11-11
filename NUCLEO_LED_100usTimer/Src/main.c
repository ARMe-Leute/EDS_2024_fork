#include <stdint.h>

#define I2C1_BASE          0x40005400
#define I2C1_CR1           *((volatile uint32_t*)(I2C1_BASE + 0x00))
#define I2C1_CR2           *((volatile uint32_t*)(I2C1_BASE + 0x04))
#define I2C1_SR1           *((volatile uint32_t*)(I2C1_BASE + 0x14))
#define I2C1_DR            *((volatile uint32_t*)(I2C1_BASE + 0x10))
#define I2C1_SR2           *((volatile uint32_t*)(I2C1_BASE + 0x18))

#define USART2_BASE        0x40004400
#define USART2_SR          *((volatile uint32_t*)(USART2_BASE + 0x00))
#define USART2_DR          *((volatile uint32_t*)(USART2_BASE + 0x04))
#define USART2_BRR         *((volatile uint32_t*)(USART2_BASE + 0x08))
#define USART2_CR1         *((volatile uint32_t*)(USART2_BASE + 0x0C))

#define I2C_START          0x01
#define I2C_STOP           0x02
#define I2C_ACK            0x04

void UART2_Init(void) {
    USART2_CR1 = 0x2000;      // USART2 Enable
    USART2_BRR = 0x1D4C;      // Baudrate auf 9600
    USART2_CR1 |= 0x0008;     // Transmitter Enable
}

void UART2_WriteChar(char c) {
    while (!(USART2_SR & 0x0080));  // Warten, bis Datenregister leer ist
    USART2_DR = c;
}

void UART2_WriteString(const char *str) {
    while (*str) {
        UART2_WriteChar(*str++);
    }
}

void I2C1_Init(void) {
    I2C1_CR1 |= 0x8000;      // Software Reset
    I2C1_CR1 &= ~0x8000;     // Reset beenden
    I2C1_CR2 = 0x0008;       // Peripherieclock auf 8 MHz
    I2C1_CR1 |= 0x0001;      // I2C Enable
}

uint8_t I2C1_TestAddress(uint8_t address) {
    I2C1_CR1 |= I2C_START;  // Start-Bit setzen
    while (!(I2C1_SR1 & 0x0001)); // Warten auf Start-Bit
    I2C1_DR = address << 1; // Adresse senden

    // Warten auf ACK oder NACK
    while (!(I2C1_SR1 & (0x0002 | 0x0004)));

    // ACK erhalten -> Gerät antwortet
    if (I2C1_SR1 & 0x0002) {
        I2C1_CR1 |= I2C_STOP; // Stop-Bit setzen
        return 1;             // Gerät vorhanden
    }

    I2C1_CR1 |= I2C_STOP;     // Stop-Bit setzen
    return 0;                 // Keine Antwort
}

void scanI2CDevices(void) {
    uint8_t address;
    char buffer[32];

    for (address = 0x03; address <= 0x77; address++) {
        if (I2C1_TestAddress(address)) {

            UART2_WriteString(buffer);
        }
    }

}

int main(void) {
    UART2_Init();     // Initialisierung der UART für Ausgabe
    I2C1_Init();      // Initialisierung des I2C1-Peripheriegeräts

    while (1) {
        scanI2CDevices();    // Scan durch I2C-Bus
        for (volatile int i = 0; i < 500000; i++); // Kurze Pause
    }
}
