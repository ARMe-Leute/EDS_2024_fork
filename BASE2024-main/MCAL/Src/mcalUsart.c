/**
 * @defgroup usart  UART/USART Functions (mcalUsart.h/.c)
 * @defgroup usart0 Deprecated UART/USART Functions
 * @ingroup  usart
 * @defgroup usart1 UART/USART Macros
 * @ingroup  usart
 * @defgroup usart2 UART/USART Standard Functions
 * @ingroup  usart
 * @defgroup usart3 UART/USART IRQ and DMA Functions
 * @ingroup  usart
 * @defgroup usart4 UART/USART Enumerations and Definitions
 * @ingroup  usart
 *
 * @file        mcalUsart.c
 * @brief       mcalUsart.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 14, 2020
 *
 * @version     0.1
 * @copyright   GNU Public License Version 3 (GPLv3)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <mcalUsart.h>
/**
 * @ingroup usart1
 */
#define USART_TX_EMPTY(usart)  ((usart)->SR & USART_SR_TXE)
/**
 * @ingroup usart1
 */
#define USART_WAIT(usart)      do { while(!USART_TX_EMPTY(usart)); } while(0)


/**
 * Function to verify the integrity of the *usart* parameter.
 */
static bool usartVerifyUsart(USART_TypeDef *usart)
{
    if ((USART1 == usart) || (USART2 == usart) || (USART6 == usart))
    {
        return true;
    }

    return false;
}

/**
 * Function to verify the integrity of the IRQ type.
 */
static bool usartVerifyIrqType(USART_IRQ_TYPES irq)
{
    if ((USART_IRQ_TXEIE  == irq) || (USART_IRQ_CTSIE == irq) || (USART_IRQ_TCIE == irq) ||
        (USART_IRQ_RXNEIE == irq) || (USART_IRQ_IDLE  == irq) || (USART_IRQ_PEIE == irq) ||
        (USART_IRQ_LBDIE  == irq) || (USART_IRQ_EIE   == irq))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the integrity of the parity.
 */
static bool usartVerifyParity(USART_PARITY par)
{
    if ((NO_PARITY == par) || (ODD_PARITY == par) || (EVEN_PARITY == par))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the integrity of the data length.
 */
static bool usartVerifyWordlen(USART_WORDLEN len)
{
    if ((LEN_8BIT == len) || (LEN_9BIT == len))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the integrity of the number of stop bits.
 */
static bool usartVerifyNumStopBits(USART_STOPBITS num)
{
    if ((HALF_BIT     == num) || (HALF_STOP == num) || (ONE_BIT  == num) || (ONE_STOP == num) ||
        (ONE_DOT_FIVE == num) || (TWO_BIT   == num) || (TWO_STOP == num))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the integrity of the oversampling setting.
 */
static bool usartVerifyOverSampling(USART_OVER ovr)
{
    if ((OVER8 == ovr) || (OVER16 == ovr))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the enable/disable mode for the receiver.
 */
static bool usartVerifyRxEnableMode(USART_RX mode)
{
    if ((RECEIVER_OFF == mode) || (RECEIVER_ON == mode))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the enable/disable mode for the transceiver.
 */
static bool usartVerifyTxEnableMode(USART_TX mode)
{
    if ((TRANSMITTER_OFF == mode) || (TRANSMITTER_ON == mode))
    {
        return true;
    }
    return false;
}

static bool usartVerifyDmaTxMode(USART_DMA_TXMODE_t tx)
{
    if ((DMA_TRANSMIT_OFF == tx) || (DMA_TRANSMIT_ON == tx))
    {
        return true;
    }
    return false;
}

static bool usartVerifyDmaRxMode(USART_DMA_RXMode_t rx)
{
    if ((DMA_RECEIVE_OFF == rx) || (DMA_RECEIVE_ON == rx))
    {
        return true;
    }
    return false;
}

/**
 * @ingroup usart2
 * Verifies the validity of the UART/USART and enables it.
 *
 * @param   *usart : Pointer to the UART/USART
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">UE</td>
 *          <td rowspan="1">13</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartEnableUsart(USART_TypeDef *usart)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }

    if (USART1 == usart)
    {
        USART1->CR1 |= USART_CR1_UE;
    }
    else if (USART2 == usart)
    {
        USART2->CR1 |= USART_CR1_UE;
    }
    else if (USART6 == usart)
    {
        USART6->CR1 |= USART_CR1_UE;
    }

    return USART_OK;
}

/**
 * @ingroup usart0
 * This function is deprecated.
 *
 * @note
 * Use **usartEnableUsart(USART_TypeDef *usart)** instead.
 *
 * @param   *usart : Pointer to the UART/USART
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">UE</td>
 *          <td rowspan="1">13</td>
 *          <td rowspan="1">Deprecated. Use usartEnableUsart() instead.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartStartUsart(USART_TypeDef *usart)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }

    if (USART1 == usart)
    {
        USART1->CR1 |= USART_CR1_UE;
    }
    else if (USART2 == usart)
    {
        USART2->CR1 |= USART_CR1_UE;
    }
    else if (USART6 == usart)
    {
        USART6->CR1 |= USART_CR1_UE;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Disables the UART/USART.
 *
 * @param   *usart : Pointer to the UART/USART
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">UE</td>
 *          <td rowspan="1">13</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartDisableUsart(USART_TypeDef *usart)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }

    if (USART1 == usart)
    {
        USART1->CR1 &= ~USART_CR1_UE_Msk;
    }
    else if (USART2 == usart)
    {
        USART2->CR1 &= ~USART_CR1_UE_Msk;
    }

    else if (USART6 == usart)
    {
        USART6->CR1 &= ~USART_CR1_UE_Msk;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Turns the bus clock of *usart on.
 *
 * @param   *usart : Pointer to the UART/USART
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>APB1ENR/APB2ENR</td>
 *          <td rowspan="1">USARTnEN/UARTnEN</td>
 *          <td rowspan="1"></td>
 *          <td rowspan="1">Please see STM RefMan.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSelectUsart(USART_TypeDef *usart)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }

    if (USART1 == usart)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    }

    if (USART2 == usart)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    }

    if (USART6 == usart)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Turns the bus clock of *usart off.
 *
 * @param   *usart : Pointer to the UART/USART
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>APB1ENR/APB2ENR</td>
 *          <td rowspan="1">USARTnEN/UARTnEN</td>
 *          <td rowspan="1"></td>
 *          <td rowspan="1">Please see STM RefMan.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartDeselectUsart(USART_TypeDef *usart)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }

    if (USART1 == usart)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN_Msk;
    }

    if (USART2 == usart)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN_Msk;
    }

    if (USART6 == usart)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN_Msk;
    }

    return USART_OK;
}

// TODO: USART Flow control not yet implemented
/**
 * Sets the hardware flow control.
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>Not yet supported</td>
 *          <td rowspan="1"></td>
 *          <td rowspan="1"></td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 *
 */
USART_RETURN_CODE_t usartSetFlowCtrlMode(USART_TypeDef *usart, USART_FLOWCTRL_t flow)
{
    return USART_OK;
}

/**
 * @ingroup usart3
 * Enables the desired USART IRQ type.
 *
 * @param  *usart   : Pointer to the UART/USART
 * @param   irqType : The IRQ that shall be enabled
 * @return  USART_RETURN_CODE_t
 *
 * @note
 * Use this function separately for each single interrupt type. It might not work
 * properly if you combine multiple interrupt types.
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1/CR3</td>
 *          <td rowspan="1">TXEIE, CTSEIE, TCIE, EIE, RXNEIE, IDLEIE, PEIE</td>
 *          <td rowspan="1"></td>
 *          <td rowspan="1">Please see STM RefMan.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartEnableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyIrqType(irqType) != true)
    {
        return USART_INVALID_IRQ_TYPE;
    }

    switch (irqType)
    {
        /* Transmit buffer empty */
        case USART_IRQ_TXEIE:
            usart->CR1 |= USART_CR1_TXEIE;
            break;

        /* Clear-to-send */
        case USART_IRQ_CTSIE:
            usart->CR3 |= USART_CR3_CTSIE;
            break;

        /* Transmission complete */
        case USART_IRQ_TCIE:
            usart->CR1 |= USART_CR1_TCIE;
            break;

        /* Error: Detects framing/overrun/noise errors */
        case USART_IRQ_EIE:
            usart->CR3 |= USART_CR3_EIE;
            break;

        /* Receive buffer not empty */
        case USART_IRQ_RXNEIE:
            usart->CR1 |= USART_CR1_RXNEIE;
            break;

        /* UART/USART idle (no activity) */
        case USART_IRQ_IDLE:
            usart->CR1 |= USART_CR1_IDLEIE;
            break;

        /* Parity error */
        case USART_IRQ_PEIE:
            usart->CR1 |= USART_CR1_PEIE;
            break;

        /* Line break */
        case USART_IRQ_LBDIE:
            break;
    }

    return USART_OK;
}

/**
 * @ingroup usart3
 * Disables the desired USART IRQ type
 *
 * @param   *usart   : Pointer to the UART/USART
 * @param    irqType : Interrupt which shall be disabled
 * @return  USART_RETURN_CODE_t
 *
 * @note
 * Use this function separately for each single interrupt type. It might not work
 * properly if you combine multiple interrupt types.
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1/CR3</td>
 *          <td rowspan="1">TXEIE, CTSEIE, TCIE, EIE, RXNEIE, IDLEIE, PEIE</td>
 *          <td rowspan="1"></td>
 *          <td rowspan="1">Please see STM RefMan.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartDisableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyIrqType(irqType) != true)
    {
        return USART_INVALID_IRQ_TYPE;
    }

    switch (irqType)
    {
        /* Transmit buffer empty */
        case USART_IRQ_TXEIE:
            usart->CR1 &= ~USART_CR1_TXEIE_Msk;
            break;

        /* Clear-to-send */
        case USART_IRQ_CTSIE:
            usart->CR3 &= ~USART_CR3_CTSIE_Msk;
            break;

        /* Transmission complete */
        case USART_IRQ_TCIE:
            usart->CR1 &= ~USART_CR1_TCIE_Msk;
            break;

        /* Error: Detects framing/overrun/noise errors */
        case USART_IRQ_EIE:
            usart->CR3 &= ~USART_CR3_EIE_Msk;
            break;

        /* Receive buffer not empty */
        case USART_IRQ_RXNEIE:
            usart->CR1 &= ~USART_CR1_RXNEIE_Msk;
            break;

        /* UART/USART idle (no activity) */
        case USART_IRQ_IDLE:
            usart->CR1 &= ~USART_CR1_IDLEIE_Msk;
            break;

        /* Parity error */
        case USART_IRQ_PEIE:
            usart->CR1 &= ~USART_CR1_PEIE_Msk;
            break;

        /* Line break */
        case USART_IRQ_LBDIE:
            usart->CR2 &= ~USART_CR2_LBDIE_Msk;
            break;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Sets the baud rate register BRR according to SYSCLK
 *
 * @param   *usart    : Pointer to USARTn
 * @param    baudrate : baudrate in bps
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">OVER8</td>
 *          <td rowspan="1">15</td>
 *          <td rowspan="1">Gets the setting of the oversampling mode.</td>
 *      </tr>
 *      <tr>
 *          <td>BRR</td>
 *          <td rowspan="1">DIV_Mantissa</td>
 *          <td rowspan="1">15...4</td>
 *          <td rowspan="1">The mantissa is internally calculated. You only have to provide the desired baud rate.</td>
 *      </tr>
 *      <tr>
 *          <td>BRR</td>
 *          <td rowspan="1">DIV_Fraction</td>
 *          <td rowspan="1">3...0</td>
 *          <td rowspan="1">The fraction is internally calculated. You only have to provide the desired baud rate.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSetBaudrate(USART_TypeDef *usart, uint32_t baudrate)
{
    uint32_t systemClock = 0u;
    uint8_t  over = 0;
    uint32_t baudRateMantissa = 0.0f;
    uint32_t baudRateFraction = 0.0f;

    SystemCoreClockUpdate();
    systemClock = rccGetSysClock();

    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }

    over = usart->CR1 & USART_CR1_OVER8;

    baudRateMantissa = (systemClock / (8 * (2 - over) * baudrate));    // Korrekt = 104 bei 9600 bps @ 16 MHz
    baudRateFraction = (systemClock - (baudRateMantissa * 8 * (2 - over) * baudrate)) / baudrate;

    usart->BRR = baudRateMantissa << 4 | baudRateFraction;

    return USART_OK;
}

/**
 * @ingroup usart2
 * Set USART communication parameters
 *
 * @param  *usart    : Pointer to the USART
 * @param   baudrate : Baudrate in bps
 * @param   parity   : Parity (None, even, odd)
 * @param   len      : Wordlength (8 or 9 bit)
 * @param   num      : Number of stop bits
 * @return  USART_RETURN_CODE_t
 *
 * @note
 * This function **always** activates both, transmitter **and** receiver.
 *
 * This is a "comfort" function. The following list shows all functions which can also be used as individual functions.<br>
 *  + usartSelectUsart()
 *  + usartSetBaudrate()
 *  + usartEnableUsart()
 *  + usartSetWordlength()
 *  + usartSetNumStopBits()
 *  + usartSetParity()
 *  + usartEnableReceiver()
 *  + usartEnableTransmitter()
 */
USART_RETURN_CODE_t usartSetCommParams(USART_TypeDef *usart,
                        uint32_t baudrate,
                        USART_PARITY parity,
                        USART_WORDLEN len,
                        USART_STOPBITS num)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyParity(parity) != true)
    {
        return USART_INVALID_PARITY;
    }
    if (usartVerifyWordlen(len) != true)
    {
        return USART_INVALID_WORDLEN;
    }
    if (usartVerifyNumStopBits(num) != true)
    {
        return USART_INVALID_NUM_STOP;
    }

    usartSelectUsart(usart);
    usartSetBaudrate(usart, baudrate);              // Set baudrate
    usartEnableUsart(usart);
    usartSetWordlength(usart, len);
    usartSetNumStopBits(usart, num);
    usartSetParity(usart, parity);
    usartEnableReceiver(usart, RECEIVER_ON);        // Always activate receiver ...
    usartEnableTransmitter(usart, TRANSMITTER_ON);  // ... and transmitter

    return USART_OK;
}

/**
 * @ingroup usart2
 * Sets the word length of the data word. Sets the M bit of the USART control register 1.
 *
 * @param   *usart : Pointer to the USART
 * @param    len   : Length of the data word
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">M</td>
 *          <td rowspan="1">12</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSetWordlength(USART_TypeDef *usart, USART_WORDLEN len)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyWordlen(len) != true)
    {
        return USART_INVALID_WORDLEN;
    }

    if (LEN_8BIT == len)
    {
        usart->CR1 &= ~USART_CR1_M;
    }
    else
    {
        usart->CR1 |= USART_CR1_M;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Sets the parity of the serial communication.
 *
 * @param   *usart  : Pointer to the USART
 * @param    parity : Parity setting
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">PCE</td>
 *          <td rowspan="1">10</td>
 *          <td rowspan="1">Parity control enable.</td>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">PS</td>
 *          <td rowspan="1">9</td>
 *          <td rowspan="1">Parity selection</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSetParity(USART_TypeDef *usart, USART_PARITY parity)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyParity(parity) != true)
    {
        return USART_INVALID_PARITY;
    }

    if (NO_PARITY == parity)
    {
        usart->CR1 &= ~USART_CR1_PCE;       // No parity
    }
    else
    {
        if (EVEN_PARITY == parity)
        {
            usart->CR1 |= USART_CR1_PCE;
            usart->CR1 &= ~USART_CR1_PS;    // Even parity
        }
        else
        {
            usart->CR1 |= USART_CR1_PCE;
            usart->CR1 |= USART_CR1_PS;     // Odd parity
        }
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Sets the number of stop bits.
 *
 * @param   *usart : Pointer to the USART
 * @param    num   : Number of stop bits
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR2</td>
 *          <td rowspan="1">STOP</td>
 *          <td rowspan="1">13...12</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSetNumStopBits(USART_TypeDef *usart, USART_STOPBITS num)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyNumStopBits(num) != true)
    {
        return USART_INVALID_NUM_STOP;
    }

    usart->CR2 &= ~USART_CR2_STOP_Msk;          // Default: 1 stop bit

    switch (num)
    {
        case HALF_BIT:
            usart->CR2 |= USART_CR2_STOP_0;     // 0.5 stop bits
            break;

        case ONE_BIT:
            usart->CR2 &= ~USART_CR2_STOP_Msk;  // 1 stop bit
            break;

        case ONE_DOT_FIVE:
            usart->CR2 |= USART_CR2_STOP_1;     // 1.5 stop bits
            break;

        case TWO_BIT:
            usart->CR2 |= USART_CR2_STOP;       // 2 stop bits

        default:
            usart->CR2 &= ~USART_CR2_STOP_Msk;  // 1 stop bit
            break;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Selects between 8x/16x over sampling. 16x is default.
 *
 * @param   *usart : Pointer to the USART
 * @param    over  : Over-sampling mode
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">OVER8</td>
 *          <td rowspan="1">15</td>
 *          <td rowspan="1">Sets the oversampling mode.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSetOversampling(USART_TypeDef *usart, USART_OVER over)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyOverSampling(over) != true)
    {
        return USART_INVALID_OVERSAMPLING_MODE;
    }

    usart->CR1 &= ~USART_CR1_OVER8_Msk;         // Default is 16x

    if (OVER8 == over)
    {
        usart->CR1 |= USART_CR1_OVER8;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Enables the USART receiver
 *
 * @param  *usart  : Pointer to the USART
 * @param   enable : Enables/disables the receiver
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">RE</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartEnableReceiver(USART_TypeDef *usart, USART_RX enable)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyRxEnableMode(enable) != true)
    {
        return USART_INVALID_RX_ENABLE_MODE;
    }

    if (RECEIVER_OFF == enable)
    {
        usart->CR1 &= ~USART_CR1_RE_Msk;
    }
    else
    {
        usart->CR1 |= USART_CR1_RE;
    }

    return USART_OK;
}

/**
 * @ingroup usart2
 * Enables the USART transmitter
 *
 * @param   *usart  : Pointer to the USART
 * @param    enable : Enables/disables the transmitter
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">TE</td>
 *          <td rowspan="1">3</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartEnableTransmitter(USART_TypeDef *usart, USART_TX enable)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyTxEnableMode(enable) != true)
    {
        return USART_INVALID_TX_ENABLE_MODE;
    }

    if (TRANSMITTER_OFF == enable)
    {
        usart->CR1 &= ~USART_CR1_TE_Msk;
    }
    else
    {
        usart->CR1 |= USART_CR1_TE;
    }

    return USART_OK;
}

/**
 * @ingroup usart3
 * This function enables all IRQ types which are provided by the *irqList.
 *
 * @param   *usart   : Pointer to the UART/USART
 * @param   *irqList : Pointer to the list which provides IRQ types
 * @param    size    : Number of interrupt types in irqList
 * @return  USART_RETURN_CODE_t
 *
 * @note
 * Be careful with this function. According to the number of interrupt types
 * provided by *irqList* it calls the function *usartEnableIrq()* multiple
 * times.
 *
 * <b>This function repeatedly calls the function usartEnableIrq() for each interrupt in the list.</b><br>
 */
USART_RETURN_CODE_t usartEnableIrqList(USART_TypeDef *usart, USART_IRQ_TYPES *irqList, uint8_t size)
{
    uint8_t         i = 0;
    USART_IRQ_TYPES irqType;

    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }

    for (i = 0; i < size; i++)
    {
        irqType = irqList[i];
        usartEnableIrq(usart, irqType);
    }

    return USART_OK;
}

#if 0
/**
 *  Pushes incoming data to the device buffer.
 */
USART_RETURN_CODE_t usartPushDataToBuffer(USART_TypeDef *device, uint16_t data)
{
//    if (device->currentPosInBuffer < device->bufferSize)
//    {
//        // Check overflow of the buffer
//        if (device->head == device->bufferSize)
//        {
//            device->head = 0;   // Restart from the beginning
//        }
//    }

    // Add data to the buffer
//    device->buffer[device->head] = data;
//    device->head++;
//    device->currentPosInBuffer++;
}


/**
 *  Reads one byte from the buffer
 */
uint16_t usartPullDataFromBuffer(USART_t *device)
{
    uint16_t byte = 0;
    if ((device->currentPosInBuffer > 0) || (device->head != device->tail))
    {
        if (device->tail == device->head)
        {
            device->tail = 0;
        }

        byte = device->buffer[device->tail];
        device->tail++;

        if (device->currentPosInBuffer)
        {
            device->currentPosInBuffer--;
        }
    }
    return byte;
}

void usartSendBuf(USART_TypeDef *usart, Ringbuffer_t *rb, uint8_t *buffer)
{
    while (*buffer)
    {
        USART_WAIT(usart);
        usart->DR = buffer[rb->readPtr];
        rb->readPtr++;
        USART_WAIT(usart);
    }
}

///**
// * Interrupt handler for USART2
// */
//void USART2_IRQHandler(void)
//{
//    USART_BUFFER_t *buf;
//    uint8_t         data = 0;
//
//    /** Receive data via USART2 **/
//    if (USART2->SR & USART_SR_RXNE)
//    {
//        USART_WAIT(USART2);
//        data = USART2->DR;                  // Reading automatically resets the RXNE flag
//        USART_WAIT(USART2);
//
//        buf = receiveBuffer;                // Assign address of the receive buffer
//
//        if (((buf->inPtr - buf->outPtr) & ~(BUFFERSIZE - 1)) == 0)
//        {
//            buf->buffer[buf->inPtr & (BUFFERSIZE - 1)] = data;
//            buf->inPtr++;
//        }
//    }
//
//    /** Transmit data via USART2 **/
//    if (USART2->SR & USART_SR_TXE)
//    {
//        USART2->SR &= ~USART_SR_TXE_Msk;
//        buf = transmitBuffer;               // Assign the address of the transmit buffer
//
//        /** If buf->inPtr != buf->outPtr --> Buffer has data **/
//        if (buf->inPtr != buf->outPtr)
//        {
//            USART2->DR = (buf->buffer[buf->outPtr & (BUFFERSIZE - 1)] & 0x01FF);
//            buf->outPtr++;
//        }
//        else    /** Transmit buffer is empty **/
//        {
//            USART2->CR1 &= ~USART_CR1_TXEIE_Msk;
//        }
//    }
//}
#endif

USART_RETURN_CODE_t usartSendByte(USART_TypeDef *usart, uint8_t byte)
{
    USART_WAIT(usart);
    usart->DR = byte & 0x01FF;
    USART_WAIT(usart);

    return USART_OK;
}

/**
 * @ingroup usart2
 * Sends the desired string via UART/USART.
 *
 * @param   *usart : Pointer to the UART/USART
 * @param   *data  : Pointer to the array which provides the content that shall be sent
 * @return  USART_RETURN_CODE_t
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR</td>
 *          <td rowspan="1">TXE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Wait until the TXE bit is set (TX buffer is empty).</td>
 *      </tr>
 *      <tr>
 *          <td>DR</td>
 *          <td rowspan="1">Data</td>
 *          <td rowspan="1">8...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSendString(USART_TypeDef *usart, char *data)
{
    while (*data)
    {
        USART_WAIT(usart);
        usart->DR = *data++ & 0x01FF;
        USART_WAIT(usart);
    }
    return USART_OK;
}

/**
 * NOT FINALLY IMPLEMENTED
 *
 * <b>Not yet implemented.</b><br>
 */
void usartInitBuffer(void)
{
//    transmitBuffer->inPtr  = 0;
//    transmitBuffer->outPtr = 0;
//
//    receiveBuffer->inPtr   = 0;
//    receiveBuffer->outPtr  = 0;
}

/**
 * @ingroup usart3
 * Resets the desired IRQ type flag
 *
 * @param   *usart   : Pointer to the UART/USART
 * @param    irqType : The IRQ that shall be cleared
 * @return  USART_RETURN_CODE_t
 *
 * @note
 * Use this function separately for each single interrupt type. It might not work
 * properly if you combine multiple interrupt types.
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>SR</td>
 *          <td rowspan="1">CTS/LBD/TC/RXNE</td>
 *          <td rowspan="1">9...5</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartResetIrqFlag(USART_TypeDef *usart, USART_IRQ_FLAG_t irqType)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyIrqType(irqType) != true)
    {
        return USART_INVALID_IRQ_TYPE;
    }

    switch (irqType)
    {
        case USART_CTS_FLG:
            usart->SR &= ~USART_SR_CTS_Msk;
            break;
        case USART_LBD_FLG:
            usart->SR &= ~USART_SR_LBD_Msk;
            break;
        case USART_TC_FLG:
            usart->SR &= ~USART_SR_TC_Msk;
            break;
        case USART_RXNE_FLG:
            usart->SR &= ~USART_SR_RXNE_Msk;
            break;

        default:
            break;
    }

    return USART_OK;
}

/**
 * @ingroup usart3
 * Sets the DMA mode for the transmitter. No error handling.
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR3</td>
 *          <td rowspan="1">DMAT</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">DMA enable transmitter.</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSetDmaTxMode(USART_TypeDef *usart, USART_DMA_TXMODE_t dmaMode)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyDmaTxMode(dmaMode) != true)
    {
        return USART_INVALID_DMA_TX_MODE;
    }

    if (DMA_TRANSMIT_OFF == dmaMode)
    {
        usart->CR3 &= ~USART_CR3_DMAT_Msk;
    }
    else
    {
        usart->CR3 |= USART_CR3_DMAT;
    }

    return USART_OK;
}

/**
 * @ingroup usart3
 * Sets the DMA mode for the receiver. No error handling.
 *
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR3</td>
 *          <td rowspan="1">DMAR</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">DMA enable receiver</td>
 *      </tr>
 * </table>
 */
USART_RETURN_CODE_t usartSetDmaRxMode(USART_TypeDef *usart, USART_DMA_RXMode_t dmaMode)
{
    if (usartVerifyUsart(usart) != true)
    {
        return USART_INVALID_USART;
    }
    if (usartVerifyDmaRxMode(dmaMode) != true)
    {
        return USART_INVALID_DMA_RX_MODE;
    }

    if (DMA_RECEIVE_OFF == dmaMode)
    {
        usart->CR3 &= ~USART_CR3_DMAR_Msk;
    }
    else
    {
        usart->CR3 |= USART_CR3_DMAR;
    }

    return USART_OK;
}
