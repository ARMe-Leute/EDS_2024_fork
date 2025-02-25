#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <mcalGPIO.h>
#include <mcalUsart.h>

typedef enum
{
    DMA_CHANNEL_0,
    DMA_CHANNEL_1,
    DMA_CHANNEL_2,
    DMA_CHANNEL_3,
    DMA_CHANNEL_4,
    DMA_CHANNEL_5,
    DMA_CHANNEL_6,
    DMA_CHANNEL_7
} DMA_CHANNEL_t;

typedef enum
{
    PER_2_MEM,
    MEM_2_PER,
    MEM_2_MEM
} DMA_TRANSFER_DIR_t;

void initUsart(USART_TypeDef *usart);
void initDmac(DMA_TypeDef *dmac);
void initDmacStreamAndChannel(uint32_t src,
                              uint32_t dest,
                              uint16_t numData,
                              DMA_TRANSFER_DIR_t dir);

volatile uint8_t msg[] = "The quick brown fox jumps\nover the lazy\ndog.\r\n";
uint16_t numData = sizeof(msg);

int main(void)
{
    // Initialisierung von PA2/PA3 fuer USART2
    gpioInitPort(GPIOA);
    gpioSelectPinMode(GPIOA, PIN2, ALTFUNC);
    gpioSelectAltFunc(GPIOA, PIN2, AF7); // PA2 : USART2 Tx
    gpioSelectPinMode(GPIOA, PIN3, ALTFUNC);
    gpioSelectAltFunc(GPIOA, PIN2, AF7); // PA3 : USART2 Rx

    // Initialisierung von USART2. Aktiviert immer Receiver
    // und Transmitter.
    // Aktiviert immer den Bustakt fuer den gewaehlten UART/USART.
    usartSetCommParams(USART2, 9600, NO_PARITY,
                       LEN_8BIT, ONE_STOP);
    usartSetDmaTxMode(USART2, DMA_TRANSMIT_ON);
    usartResetIrqFlag(USART2, USART_TC_FLG);

    // Initialisierung von DMAC1: Hier nur DMA1_Stream6/Kanal 4!
    initDmac(DMA1);
    initDmacStreamAndChannel((uint32_t) msg,
                             (uint32_t) &USART2->DR,
                             numData,
                             MEM_2_PER);
    while(1)
    {
    }
}

/**
 * @brief DMAC: Bustakt aktivieren
 *
 * @param[in] *dmac : Pointer auf den DMA-Controller
 */
void initDmac(DMA_TypeDef *dmac)
{
    if (DMA1 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // DMA1: Bustakt
        // aktivieren
    }
    if (DMA2 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // DMA2: Bustakt
        // aktivieren
    }
}

/**
 * @brief Initialisierung des Streams und des Stream-Kanals.
 *
 * @param[in] *stream : Pointer auf den DMA-Stream
 * @param[in] channel : Nummer des Stream-Kanals
 *
 * @note
 * Hier werden nur die Einstellungen fuer DMA1_Stream6Kanal4 verwendet.
 * Die MCAL wird eine allgemeingueltige Form enthalten.
 */
void initDmacStreamAndChannel(uint32_t src,
                              uint32_t dest,
                              uint16_t numData,
                              DMA_TRANSFER_DIR_t dir)
{
    // Wir verwenden nur DMA1_Sream6/Kanal 4
    DMA_Stream_TypeDef *stream = DMA1_Stream6;
    DMA_CHANNEL_t chn = DMA_CHANNEL_4;

    // Deaktiviere den Stream: Sonst sind keine Einstellungen moeglich!
    stream->CR &= ~DMA_SxCR_EN_Msk;
    while(stream->CR & DMA_SxCR_EN)
    {
        // Warte, bis das Enable-Bit = "0" ist.
    }

    stream->CR = chn << DMA_SxCR_CHSEL_Pos; // Kanal 4
    stream->CR |= DMA_SxCR_TCIE; // "Transfer-complete"-Interrupt aktivieren
    stream->CR &= ~DMA_SxCR_MSIZE_Msk; // MSIZE = '00' --> Byte-Transfer
    stream->CR &= ~DMA_SxCR_PSIZE_Msk; // PSIZE = '00' --> Byte-Transfer
    stream->CR &= ~DMA_SxCR_DIR_Msk; // DIR = Reset
    stream->CR |= dir << DMA_SxCR_DIR_Pos; // DIR = MEM_2_PER
    stream->CR |= DMA_SxCR_MINC; // Automatisches Inkrement
    stream->M0AR = src; // Quelle = Speicher
    stream->PAR = dest; // Ziel = USART2->DR
    stream->NDTR = numData; // Anzahl der Bytes
    stream->CR |= DMA_SxCR_EN; // Stream/Kanal aktivieren
    NVIC_EnableIRQ(DMA1_Stream6_IRQn); // Interrupt aktivieren
}

/**
 * @brief ISR fuer DMA1/Stream6
 */
void DMA1_Stream6_IRQHandler(void)
{
    // Hier wird nur "Transfer complete" ausgewertet.
    if (DMA1->HISR & DMA_HISR_TCIF6)
    {
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6; // Reset "Transfer complete" flag
    }
}
