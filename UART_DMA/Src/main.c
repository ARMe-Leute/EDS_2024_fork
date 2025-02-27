#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <mcalGPIO.h>
#include <mcalUsart.h>
#include <mcalDMAC.h>

volatile uint8_t msg[] = "The quick brown fox jumps\nover the lazy\ndog.\r\n";
uint16_t numData = sizeof(msg);

int main(void)
{
    // Initialize GPIO for USART2 (PA2/PA3)
    gpioInitPort(GPIOA);
    gpioSelectPinMode(GPIOA, PIN2, ALTFUNC);
    gpioSelectAltFunc(GPIOA, PIN2, AF7); // PA2 : USART2 Tx
    gpioSelectPinMode(GPIOA, PIN3, ALTFUNC);
    gpioSelectAltFunc(GPIOA, PIN3, AF7); // PA3 : USART2 Rx

    // Initialize USART2 with communication parameters
    usartSetCommParams(USART2, 9600, NO_PARITY, LEN_8BIT, ONE_STOP);
    usartSetDmaTxMode(USART2, DMA_TRANSMIT_ON);
    usartResetIrqFlag(USART2, USART_TC_FLG);

    // Initialize DMA for memory to peripheral transfer
    dmacSelectDMAC(DMA1);

    // Configure DMA Stream 6, Channel 4 for USART2_TX
    dmacDisableStream(DMA1_Stream6);
    dmacAssignStreamAndChannel(DMA1_Stream6, DMA_CHN_4);
    dmacSetMemoryAddress(DMA1_Stream6, MEM_0, (uint32_t)msg);
    dmacSetPeripheralAddress(DMA1_Stream6, (uint32_t)&USART2->DR);
    dmacSetDataFlowDirection(DMA1_Stream6, MEM_2_PER);
    dmacSetNumData(DMA1_Stream6, numData);

    // Configure data format and increment modes
    dmacSetMemoryDataFormat(DMA1_Stream6, BYTE);
    dmacSetPeripheralDataFormat(DMA1_Stream6, BYTE);
    dmacSetMemoryIncrementMode(DMA1_Stream6, INCR_ENABLE);
    dmacSetPeripheralIncrementMode(DMA1_Stream6, INCR_DISABLE);

    // Set priority and enable transfer complete interrupt
    dmacSetPriorityLevel(DMA1_Stream6, PRIO_MEDIUM);
    dmacEnableInterrupt(DMA1_Stream6, TX_COMPLETE);

    // Clear any pending flags before enabling the stream
    dmacClearAllStreamIrqFlags(DMA1, DMA1_Stream6);

    // Enable the DMA stream to start transfer
    dmacEnableStream(DMA1_Stream6);

    while(1)
    {
        // Main loop
    }
}

// DMA interrupt handler
void DMA1_Stream6_IRQHandler(void)
{
    // Check if transfer complete flag is set
    if (dmacGetHighInterruptStatus(DMA1) & (1 << 21)) // Check TC flag for Stream6
    {
        // Clear the transfer complete flag
        dmacClearInterruptFlag(DMA1, DMA1_Stream6, TX_COMPLETE);

        // Additional processing after transfer completion could go here
    }
}
