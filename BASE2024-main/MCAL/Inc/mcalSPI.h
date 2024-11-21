/**
 * mcalSPI.h
 *
 *  Created on: 17.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALSPI_H_
#define MCALSPI_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include <mcalGPIO.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @ingroup spi3 SPI Enumerations
 * @{
 */
typedef enum
{
    SPI_OK                      =   0,
    SPI_INVALID_SPI             = -80,
    SPI_INVALID_CLOCK_DIVIDER   = -81,
    SPI_INVALID_DATA_LENGTH     = -82,
    SPI_INVALID_SW_SLV_MGMT     = -83,
    SPI_INVALID_SLAVE_SELECTION = -84,
    SPI_INVALID_SSI_LEVEL       = -85,
    SPI_INVALID_OP_MODE         = -86,
    SPI_INVALID_PHASE           = -87,
    SPI_INVALID_IDLE_POLARITY   = -88
} SPI_RETURN_CODE_t;

typedef enum
{
  MASTER,
  SLAVE
} SPI_OPMODE_t;

typedef enum
{
    TWO_LINE_UNI,
    ONE_LINE_BIDI
} SPI_BidiMode_t;
/* doppelt zu SPI_DATA_xx_BIT
typedef enum
{
    DATA_FORMAT_8  = 0,
    DATA_FORMAT_16
} SPI_DATA_FORMAT_t;
*/
typedef enum
{
    CLK_DIV_2       = 0,
    CLK_DIV_4,
    CLK_DIV_8,
    CLK_DIV_16,
    CLK_DIV_32,
    CLK_DIV_64,
    CLK_DIV_128,
    CLK_DIV_256
} SPI_CLOCK_DIV_t;

typedef enum
{
    SPI_DATA_8_BIT,
    SPI_DATA_16_BIT
} SPI_DATALEN_t;

typedef enum
{
    SSM_OFF,
    SSM_ON
} SPI_SSM_t;

typedef enum
{
    SSI_LVL_LOW,
    SSI_LVL_HIGH
} SPI_SSI_LVL_t;

typedef enum
{
    SPI_PHASE_EDGE_1,
    SPI_PHASE_EDGE_2
} SPI_PHASE_t;

typedef enum
{
    SPI_IDLE_LOW,
    SPI_IDLE_HIGH
} SPI_POLARITY_t;

typedef enum
{
    SPI_SEND_BYTE_1,
    SPI_SEND_BYTE_2
} SPI_8BIT_STATE_t;

typedef enum
{
    ERROR_IE = 1 << 5,
    RXN_EI   = 1 << 6,
    TXE_IE   = 1 << 7
} SPI_INTERRUPT_t;

typedef enum
{
    RX_DMA_EN = 1 << 0,
    TX_DMA_EN = 1 << 1
} SPI_DMA_t;


/**
 * @}
 */

// Deprecated functions

// Supported functions
extern SPI_RETURN_CODE_t spiSetDataLen(SPI_TypeDef *spi, SPI_DATALEN_t len);
extern SPI_RETURN_CODE_t spiSelectSPI(SPI_TypeDef *spi);
extern SPI_RETURN_CODE_t spiDeselectSPI(SPI_TypeDef *spi);
extern SPI_RETURN_CODE_t spiEnableSPI(SPI_TypeDef *spi);
extern SPI_RETURN_CODE_t spiDisableSPI(SPI_TypeDef *spi);
extern SPI_RETURN_CODE_t spiWriteByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t data);
extern SPI_RETURN_CODE_t spiWriteWord(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint16_t data);
extern SPI_RETURN_CODE_t spiSendData(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg, uint8_t data);

extern SPI_RETURN_CODE_t spiInitSPI (SPI_TypeDef *spi, SPI_CLOCK_DIV_t div, SPI_DATALEN_t len,
                                     SPI_SSM_t ssm, SPI_SSI_LVL_t lvl, SPI_OPMODE_t opMode,
                                     SPI_PHASE_t phase, SPI_POLARITY_t polarity);
extern SPI_RETURN_CODE_t spiSetOpMode(SPI_TypeDef *spi, SPI_OPMODE_t mode);


extern void spiEnableInterruptType(SPI_TypeDef *spi, uint8_t interruptType);
extern void spiDisableInterruptType(SPI_TypeDef *spi, uint8_t interruptType);
extern void spiClearInterruptType(SPI_TypeDef *spi, uint8_t interruptType);
extern void spiEnableDmaType(SPI_TypeDef *spi, uint8_t dmaType);
extern void spiDisableDmaType(SPI_TypeDef *spi, uint8_t dmaType);

//extern uint8_t spiReadByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin);
//extern uint8_t spiReadRegByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg);
//extern SPI_RETURN_CODE_t spiReadRegWord(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg, uint16_t *data);
extern SPI_RETURN_CODE_t spiReadRegBurst(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg, uint8_t *data, uint8_t num);


#ifdef __cplusplus
}
#endif


#endif /* MCALSPI_H_ */
