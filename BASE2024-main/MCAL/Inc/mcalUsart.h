/**
 * @brief Implementation of UART/USART control and access of the STM32F446
 *
 * Created on      : 16.08.2020
 *     Author      : Ralf Jesse
 *      Email      : embedded@ralf-jesse.de
 * Copyright       : Ralf Jesse, 2020
 * Other copyrights: Tilen Majerle, 2014
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

#ifndef MCALUSART_H_
#define MCALUSART_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <mcalRCC.h>
#include <mcalUsart.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup usart4
 * @{
 */

/**
 * Type definitions and enumerations
 */
#define BUFFERSIZE  (128)

typedef enum
{
    USART_OK                        = 0,
    USART_INVALID_USART             = -100,
    USART_INVALID_IRQ_TYPE          = -101,
    USART_INVALID_PARITY            = -102,
    USART_INVALID_WORDLEN           = -103,
    USART_INVALID_NUM_STOP          = -104,
    USART_INVALID_OVERSAMPLING_MODE = -105,
    USART_INVALID_RX_ENABLE_MODE    = -1010,
    USART_INVALID_TX_ENABLE_MODE    = -107,
    USART_INVALID_DMA_TX_MODE       = -108,
    USART_INVALID_DMA_RX_MODE       = -109
} USART_RETURN_CODE_t;

typedef struct
{
    uint8_t inPtr;
    uint8_t outPtr;
    uint8_t buffer[BUFFERSIZE];
} USART_BUFFER_t;

typedef enum
{
    USART_ENABLE = 0,
    USART_DISABLE
} USART_MODE_t;

typedef enum
{
    USART_ON = 0,
    USART_OFF
} USART_BUSCLK;

typedef enum
{
    NO_PARITY = 0,
    EVEN_PARITY,
    ODD_PARITY
} USART_PARITY;

typedef enum
{
    LEN_8BIT = 0,
    LEN_9BIT
} USART_WORDLEN;

typedef enum
{
    HALF_BIT        = 0,
    HALF_STOP       = 0,
    ONE_BIT         = 1,
    ONE_STOP        = 1,
    ONE_DOT_FIVE    = 2,
    TWO_BIT         = 3,
    TWO_STOP        = 3
} USART_STOPBITS;

typedef enum
{
    OVER16 = 0,
    OVER8
} USART_OVER;

typedef enum
{
    RECEIVER_OFF = 0,
    RECEIVER_ON
} USART_RX;

typedef enum
{
    TRANSMITTER_OFF = 0,
    TRANSMITTER_ON
} USART_TX;

typedef enum
{
    OFF = 0,
    ON
} USART_IRQ_MODE;

typedef enum
{
    USART_IRQ_TXEIE = 0,
    USART_IRQ_CTSIE,
    USART_IRQ_TCIE,
    USART_IRQ_RXNEIE,
    USART_IRQ_IDLE,
    USART_IRQ_PEIE,
    USART_IRQ_LBDIE,
    USART_IRQ_EIE
} USART_IRQ_TYPES;

typedef enum
{
    USART_CTS_FLG,
    USART_LBD_FLG,
    USART_TC_FLG,
    USART_RXNE_FLG
} USART_IRQ_FLAG_t;

typedef enum
{
    DMA_TRANSMIT_OFF,
    DMA_TRANSMIT_ON
} USART_DMA_TXMODE_t;

typedef enum
{
    DMA_RECEIVE_OFF,
    DMA_RECEIVE_ON
} USART_DMA_RXMode_t;

typedef enum
{
    // TODO: Flow control constants not yet implemented
    NOT_YET_DEFINED
} USART_FLOWCTRL_t;

/**
 * @}
 */

/**
 * Variables
 */
//USART_BUFFER_t  *receiveBuffer;
//USART_BUFFER_t  *transmitBuffer;

/**
 * Prototypes
 */

/* General UART/USART activities */
extern USART_RETURN_CODE_t usartSelectUsart(USART_TypeDef *usart);
extern USART_RETURN_CODE_t usartDeselectUsart(USART_TypeDef *usart);
extern USART_RETURN_CODE_t usartEnableUsart(USART_TypeDef *usart);
extern USART_RETURN_CODE_t usartDisableUsart(USART_TypeDef *usart);


extern USART_RETURN_CODE_t usartPushDataToBuffer(USART_TypeDef *device, uint16_t data);

/* USART buffer */
extern void usartInitBuffer(void);
extern USART_RETURN_CODE_t usartSendByte(USART_TypeDef *usart, uint8_t byte);
extern USART_RETURN_CODE_t usartSendString(USART_TypeDef *usart, char *data);

/* Communication parameters */
extern USART_RETURN_CODE_t usartSetCommParams(USART_TypeDef *usart, uint32_t baudrate,
                               USART_PARITY parity, USART_WORDLEN len,
                               USART_STOPBITS stop);
extern USART_RETURN_CODE_t usartSetBaudrate(USART_TypeDef *usart, uint32_t baudrate);
extern USART_RETURN_CODE_t usartSetWordlength(USART_TypeDef *usart, USART_WORDLEN len);
extern USART_RETURN_CODE_t usartSetParity(USART_TypeDef *usart, USART_PARITY parity);
extern USART_RETURN_CODE_t usartSetNumStopBits(USART_TypeDef *usart, USART_STOPBITS num);
extern USART_RETURN_CODE_t usartSetOversampling(USART_TypeDef *usart, USART_OVER over);
extern USART_RETURN_CODE_t usartEnableReceiver(USART_TypeDef *usart, USART_RX enable);
extern USART_RETURN_CODE_t usartEnableTransmitter(USART_TypeDef *usart, USART_TX enable);
extern USART_RETURN_CODE_t usartSetFlowCtrlMode(USART_TypeDef *usart, USART_FLOWCTRL_t flow);

/* Sending and receiving data */
extern USART_RETURN_CODE_t usartGetByte(USART_TypeDef *usart, uint16_t byte);

/* Interrupts */
extern USART_RETURN_CODE_t usartEnableIrqList(USART_TypeDef *usart, USART_IRQ_TYPES *irqList, uint8_t size);
extern USART_RETURN_CODE_t usartEnableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType);
extern USART_RETURN_CODE_t usartDisableIrq(USART_TypeDef *usart, USART_IRQ_TYPES irqType);
extern USART_RETURN_CODE_t usartResetIrqFlag(USART_TypeDef *usart, USART_IRQ_FLAG_t irqFlag);

/* DMA */
extern USART_RETURN_CODE_t usartSetDmaTxMode(USART_TypeDef *usart, USART_DMA_TXMODE_t dmaMode);
extern USART_RETURN_CODE_t usartSetDmaRxMode(USART_TypeDef *usart, USART_DMA_RXMode_t dmaMode);

/* Deprecated */
extern USART_RETURN_CODE_t usartStartUsart(USART_TypeDef *usart);

#ifdef __cplusplus
}
#endif



#endif /* MCALUSART_H_ */
