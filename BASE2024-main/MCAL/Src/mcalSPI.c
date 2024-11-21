/**
 * @defgroup spi  Serial Peripheral Interface (SPI) Functions (mcalSPI.h/.c)
 * @defgroup spi0 Deprecated SPI Functions
 * @ingroup  spi
 * @defgroup spi1 SPI Macros
 * @ingroup  spi
 * @defgroup spi2 SPI Standard Functions
 * @ingroup  spi
 * @defgroup spi3 SPI Enumerations and definitions
 * @ingroup  spi
 *
 * @file        mcalSPI.c
 * @brief       mcalSPI.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 12, 2020
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

#include <mcalGPIO.h>
#include <mcalSPI.h>


static inline void __spi_Chk_TX_empty(SPI_TypeDef *spi)
{

	uint16_t simpleDelay;
	while(!(spi->SR & SPI_SR_TXE));
	for (simpleDelay = 0; simpleDelay < 3; simpleDelay++) {	; }  // Kurze Zwangspause, da das Signal TXE etwas vor Ausgbae des Letzen Bite erfolgt. ca 500 ns

}

static inline void __spi_Chk_RX_empty(SPI_TypeDef *spi)
{
	uint16_t simpleDelay;
	while(!(spi -> SR & SPI_SR_RXNE));
	for (simpleDelay = 0; simpleDelay < 4; simpleDelay++) {	; }

}



static inline void __spi_Chk_notBSY(SPI_TypeDef *spi)
{
	while((spi->SR & SPI_SR_BSY));
}

/*
static inline void __spi_ClearRX(SPI_TypeDef *spi)
{
	uint8_t b;
	while (spi -> SR & SPI_SR_RXNE)	{b = spi->DR;}
}
*/
/**
 * Verifies the integrity of the SPI port.
 */
static bool spiVerifySPI(SPI_TypeDef *spi)
{
    if ((SPI1 == spi) || (SPI2 == spi) || (SPI3 == spi) || (SPI4 == spi))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI clock divider.
 */
static bool spiVerifyClkDivider(SPI_CLOCK_DIV_t div)
{
    if ((CLK_DIV_2  == div) || (CLK_DIV_4  == div) || (CLK_DIV_8   == div) || (CLK_DIV_16  == div) ||
        (CLK_DIV_32 == div) || (CLK_DIV_64 == div) || (CLK_DIV_128 == div) || (CLK_DIV_256 == div))
    {
        return true;
    }
    return false;
}



/**
 * Verifies the integrity of the SPI Software Slave Management.
 */
static bool spiVerifySSM(SPI_SSM_t ssm)
{
    if ((SSM_ON == ssm) || (SSM_OFF == ssm))
    {
        return true;
    }
    return false;
}

#if 0
// This function is currently not used.
/**
 * Verifies the integrity of the SSI level.
 */
static bool spiVerifySsiLvl(SPI_SSI_LVL_t lvl)
{
    if ((SSI_LVL_HIGH == lvl) || (SSI_LVL_LOW == lvl))
    {
        return true;
    }
    return false;
}
#endif

/**
 * Verifies the integrity of the SPI operational mode.
 */
static bool spiVerifyOpMode(SPI_OPMODE_t om)
{
    if ((MASTER == om) || (SLAVE == om))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI trigger phase.
 */
static bool spiVerifyPhase(SPI_PHASE_t ph)
{
    if ((SPI_PHASE_EDGE_1 == ph) || (SPI_PHASE_EDGE_2 == ph))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the integrity of the SPI idle level.
 */
static bool spiVerifyIdlePolarity(SPI_POLARITY_t pol)
{
    if ((SPI_IDLE_LOW == pol) || (SPI_IDLE_HIGH == pol))
    {
        return true;
    }
    return false;
}

/**
 * verify and set the integrity of the SPI data length.
 */
SPI_RETURN_CODE_t spiSetDataLen(SPI_TypeDef *spi, SPI_DATALEN_t len)
{
    if (SPI_DATA_8_BIT == len)
    {
    	spi->CR1 &= ~SPI_CR1_DFF_Msk;
     	return SPI_OK;
    }
    if (SPI_DATA_16_BIT == len)
    {
    	spi->CR1 |= SPI_CR1_DFF;
    	return SPI_OK;
    }
    return SPI_INVALID_DATA_LENGTH;
}






SPI_RETURN_CODE_t spiInitSPI(SPI_TypeDef *spi, SPI_CLOCK_DIV_t div, SPI_DATALEN_t len,
                             SPI_SSM_t ssm, SPI_SSI_LVL_t lvl, SPI_OPMODE_t opMode,
                             SPI_PHASE_t phase, SPI_POLARITY_t polarity)
{
    uint16_t cr1 = 0U;
    // Parameter verification
    if (spiVerifyClkDivider(div) != true)
    {
        return SPI_INVALID_CLOCK_DIVIDER;
    }
    if (spiVerifySSM(ssm) != true)
    {
        return SPI_INVALID_SW_SLV_MGMT;
    }
#if 0
    // This is currently not passed as a parameter.
    if (spiVerifySsiLvl(lvl) != true)
    {
        return SPI_INVALID_SSI_LEVEL;
    }
#endif
    if (spiVerifyOpMode(opMode) != true)
    {
        return SPI_INVALID_OP_MODE;
    }
    if (spiVerifyPhase(phase) != true)
    {
        return SPI_INVALID_PHASE;
    }
    if (spiVerifyIdlePolarity(polarity) != true)
    {
        return SPI_INVALID_IDLE_POLARITY;
    }

    /**
     *  All parameter check passed successfully!
     */

    // Setting up the baudrate (PCLK / Pre-Scaler)
    switch (div)
    {
        case CLK_DIV_2:
            cr1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;

        case CLK_DIV_4:
            cr1 |= SPI_CR1_BR_0;
            break;

        case CLK_DIV_8:
            cr1 |= SPI_CR1_BR_1;
            break;

        case CLK_DIV_16:
            cr1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;

        case CLK_DIV_32:
            cr1 |= SPI_CR1_BR_2;
            break;

        case CLK_DIV_64:
            cr1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_0);
            break;

        case CLK_DIV_128:
            cr1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1);
            break;

        case CLK_DIV_256:
            cr1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;

        default:
            cr1 |= (SPI_CR1_BR_1 | SPI_CR1_BR_0);
            break;
    }

    // Setting up the data length
    if (SPI_DATA_8_BIT == len)
    {
        cr1 &= ~SPI_CR1_DFF_Msk;
    }
    else
    {
        cr1 |= SPI_CR1_DFF;
    }

    // Set SSM and SSI bits
    if (SSM_ON == ssm)
    {
        cr1 |= SPI_CR1_SSM;

        // SSI level works only if SSM is active
        if (SSI_LVL_HIGH == lvl)
        {
            cr1 |= SPI_CR1_SSI;
        }
        else
        {
            cr1 &= ~SPI_CR1_SSI_Msk;
        }
    }
    else
    {
        cr1 &= ~SPI_CR1_SSM_Msk;
    }

    // Select between Master/Slave mode
    if (MASTER == opMode)
    {
        cr1 |= SPI_CR1_MSTR;
    }
    else
    {
        cr1 &= ~SPI_CR1_MSTR_Msk;
    }

    // Set clock phase
    if (SPI_PHASE_EDGE_1 == phase)
    {
        cr1 &= ~SPI_CR1_CPHA_Msk;
    }
    else
    {
        cr1 |= SPI_CR1_CPHA;
    }

    // Set clock polarity
    if (SPI_IDLE_LOW == polarity)
    {
        cr1 &= ~SPI_CR1_CPOL_Msk;
    }
    else
    {
        cr1 |= SPI_CR1_CPOL;
    }

    // Transfer settings to CR1 + CR2
    spi->CR1 = cr1;
    //spi->CR1 = 0x31E; //8-bit frames
    spi->CR2 = 0;                     // Simplified version. Should be modified.

    // Finally, enable SPIn
    spiEnableSPI(spi);

    return SPI_OK;
}


SPI_RETURN_CODE_t spiSelectSPI(SPI_TypeDef *spi)
{
    // All parameter check passed successfully!

    // Selects the bus clock for SPIn
    if (SPI1 == spi)
    {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    }
    else if (SPI2 == spi)
    {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    }
    else if (SPI3 == spi)
    {
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }
    else if (SPI4 == spi)
    {
        RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
    }

    return SPI_OK;
}


SPI_RETURN_CODE_t spiDeselectSPI(SPI_TypeDef *spi)
{
    // Parameter verification
    if (spiVerifySPI(spi) != true)
    {
        return SPI_INVALID_SPI;
    }

    // Selects the bus clock for SPIn
    if (SPI1 == spi)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN_Msk;
    }
    else if (SPI2 == spi)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN_Msk;
    }
    else if (SPI3 == spi)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN_Msk;
    }
    else if (SPI4 == spi)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN_Msk;
    }


    return SPI_OK;
}

SPI_RETURN_CODE_t spiEnableSPI(SPI_TypeDef *spi)
{

	// Parameter verification
    if (spiVerifySPI(spi) != true)
    {
        return SPI_INVALID_SPI;
    }

    // All parameter check passed successfully!
    spi->CR1 |= SPI_CR1_SPE;

    return SPI_OK;
}


SPI_RETURN_CODE_t spiDisableSPI(SPI_TypeDef *spi)
{
    // Parameter verification
    if (spiVerifySPI(spi) != true)
    {
        return SPI_INVALID_SPI;
    }

    // All parameter check passed successfully!
    spi->CR1 &= ~SPI_CR1_SPE_Msk;

    return SPI_OK;
}


SPI_RETURN_CODE_t spiWriteByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t data)
{
    //static uint8_t state = SPI_SEND_BYTE_1;

    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }
	__spi_Chk_TX_empty(spi);
    gpioResetPin(port, pin);              // Set CS input to low level

	spi->DR = data;                         // Send first byte to data register
	__spi_Chk_TX_empty(spi);

	__spi_Chk_notBSY(spi);
	gpioSetPin(port, pin);


/*

    // All parameters verified successfully.
    switch (state)
    {
        case SPI_SEND_BYTE_1:
        {
            gpioResetPin(port, pin);              // Set CS input to low level
            __spi_Chk_TX_empty(spi);
            spi->DR = data;                         // Send first byte to data register
            __spi_Chk_notBSY(spi);
            state = SPI_SEND_BYTE_2;
            break;                                  // Leave switch-case-check
        }
        case SPI_SEND_BYTE_2:
        {
        	__spi_Chk_TX_empty(spi);
            spi->DR = data;                         // Send first byte to data register

            __spi_Chk_notBSY(spi);
            gpioSetPin(port, pin);
            state = SPI_SEND_BYTE_1;
        }
    }
*/
    return SPI_OK;
}


SPI_RETURN_CODE_t spiWriteWord(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint16_t data)
{
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }

    // All parameter check passed successfully!

    // Wait until the TX register is empty
    while (!(spi->SR & SPI_SR_TXE))
    {
        ;
    }

    // Reset CS
    gpioResetPin(port, pin);

    // Transfer new data to the transfer register
    spi->DR = data;

    // Waits until data has been sent
    while (spi->SR & SPI_SR_BSY)
    {
        // Wait: SPI is still busy
    }

    // Set CS
    gpioSetPin(port, pin);

    return SPI_OK;
}



SPI_RETURN_CODE_t spiSendData(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg, uint8_t data)
{
    // Parameter verification
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }

    // All parameter check passed successfully!

    if (!(spi->CR1 & SPI_CR1_DFF))				// mit 8BIT Daten Lenge werden reg und data nacheinander gesendet
    {
    	__spi_Chk_TX_empty(spi);
    	gpioResetPin(port, pin);              // Set CS input to low level
		spi->DR = reg;                         // Send first byte to data register
		__spi_Chk_TX_empty(spi);
		spi->DR = data;                         // Send first byte to data register
		__spi_Chk_TX_empty(spi);
		//__spi_Chk_notBSY(spi);
		gpioSetPin(port, pin);

    	//spiWriteByte(spi, port, pin, reg);
        //spiWriteByte(spi, port, pin, data);
    }
    else
    {
        spiWriteWord(spi, port, pin, (reg << 8) | data);
    }

    return SPI_OK;
}

// Author: TFlämig


// does not work correct ->

uint8_t spiReadByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin)
{
	uint8_t  b;
	while (spi -> SR & SPI_SR_RXNE)	{b = spi->DR;}
	__spi_Chk_notBSY(spi);

	gpioResetPin(port, pin);	 // Set CS  to low level
	spi->CR1 |= SPI_CR1_RXONLY;  // reset
   // b = (uint8_t)spi->DR;
    while (spi -> SR & SPI_SR_RXNE)	{b = spi->DR;}
    //for (simpleDelay = 0; simpleDelay < 1; simpleDelay++) {	; }  // timer delay appr. 1 us, wait for sending finished
    spi->CR1 &= ~SPI_CR1_RXONLY;
    __spi_Chk_RX_empty(spi);    //wait until rx buf is not empty (RXNE flag)
    //spi->CR1 &= ~SPI_CR1_RXONLY;
	//__spi_Chk_notBSY(spi);
  	gpioSetPin(port, pin);

    return b;
}


uint8_t spiReadRegByte(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg)
{
	uint8_t simpleDelay, b;
	 // Parameter verification
	__spi_Chk_notBSY(spi);
	gpioResetPin(port, pin);              // Set CS input to low level
	spi->DR = reg;                         // Send first byte to data register
	__spi_Chk_TX_empty(spi);

	for (simpleDelay = 0; simpleDelay < 4; simpleDelay++) {	; }  // timer delay appr. 1 us, wait for sending finished
	while (spi -> SR & SPI_SR_RXNE)	{b = spi->DR;}					//clear Data Reg from spam
    spi->CR1 |= SPI_CR1_RXONLY;  			// active SPI Clock for half duplex reading
    //b = spi->DR;
    while (spi -> SR & SPI_SR_RXNE)	{b = spi->DR;}
    __spi_Chk_RX_empty(spi);    			//wait until rx buf is not empty (RXNE flag)
    spi->CR1 &= ~SPI_CR1_RXONLY;			// Fullduplex SPI Clock
    gpioSetPin(port, pin);

    return b;
}

SPI_RETURN_CODE_t spiReadRegWord(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg, uint16_t *data)
{
	uint16_t simpleDelay, helper1, helper2 = 0;

	while (spi -> SR & SPI_SR_RXNE)	{helper1 = spi->DR;}
	__spi_Chk_notBSY(spi);
	gpioResetPin(port, pin);                // Set CS input to low level
	spi->DR = reg;  						// Send first byte to data register

	__spi_Chk_TX_empty(spi);
	for (simpleDelay = 0; simpleDelay < 4; simpleDelay++) {	; }
	spi->CR1 |= SPI_CR1_RXONLY;  			// active SPI Clock for half duplex reading

	helper1 = spi->DR;
	__spi_Chk_RX_empty(spi);

	helper2 = spi->DR;
	spi->CR1 &= ~SPI_CR1_RXONLY;
	__spi_Chk_RX_empty(spi);				// Fullduplex SPI Clock

	//Big or litle endian order: here comes 1st as High Byte; 2nd as Low Byte
	*data = helper2 + (helper1<<8);
	gpioSetPin(port, pin);
    return SPI_OK;
}



// <- end does not work correct

SPI_RETURN_CODE_t spiReadRegBurst(SPI_TypeDef *spi, GPIO_TypeDef *port, PIN_NUM_t pin, uint8_t reg, uint8_t *data, uint8_t num)
{
	uint32_t value;
	uint8_t b, simpleDelay;

	while (spi -> SR & SPI_SR_RXNE)	{b = spi->DR;}

	__spi_Chk_notBSY(spi);
	gpioResetPin(port, pin);              	// Set CS input to low level

	spi->DR = reg;  							// Send first byte to data register

	 //while (!(spi->SR & SPI_SR_TXE))         // Wait until the TX register is empty
	__spi_Chk_TX_empty(spi);
	for (simpleDelay = 0; simpleDelay < 4; simpleDelay++) {	; }
    spi->CR1 |= SPI_CR1_RXONLY;  			// active SPI Clock for half duplex reading

    while (num > 0)                        // Start reading multiple values
    {
	   if (num == 1U)                      // If there is only one byte left...
	   {
		   __spi_Chk_RX_empty(spi);
		   value = (spi->DR);         			// Read data from data register
		   spi->CR1 &= ~SPI_CR1_RXONLY;		// Fullduplex SPI Clock
		    b = (uint8_t) value;
		    *data++ = b;
		   break;
	   }
	   else                                // More than one byte left
	   {
		   __spi_Chk_RX_empty(spi);			 // Wait until receive buffer is no longer empty
		   value = spi->DR;         		// Read data from data register
		   b =(uint8_t) value;
		   *data++ = b;
		   num--;
	   }
    }
    __spi_Chk_RX_empty(spi);
    gpioSetPin(port, pin);
	return SPI_OK;
}


//TODO Not yet implemented
/**
 * Not yet implemented
 */

void spiEnableInterruptType(SPI_TypeDef *spi, uint8_t interruptType)
{

}

void spiDisableInterruptType(SPI_TypeDef *spi, uint8_t interruptType)
{

}

void spiClearInterruptType(SPI_TypeDef *spi, uint8_t interruptType)
{

}

void spiEnableDmaType(SPI_TypeDef *spi, uint8_t dmaType)
{

}

void spiDisableDmaType(SPI_TypeDef *spi, uint8_t dmaType)
{

}


SPI_RETURN_CODE_t spiSetOpMode(SPI_TypeDef *spi, SPI_OPMODE_t opMode)
{
    // Select between Master/Slave mode
    if (MASTER == opMode)
    {
        spi->CR1 |= SPI_CR1_MSTR;
    }
    else
    {
        spi->CR1 &= ~SPI_CR1_MSTR_Msk;
    }

	return SPI_OK;
}
