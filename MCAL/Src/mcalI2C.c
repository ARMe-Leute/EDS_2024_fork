/**
 * @defgroup iic  Inter-integrated Circuit (I2C) Functions (mcalIIC.h/.c)
 * @defgroup iic0 Deprecated IIC Functions
 * @ingroup  iic
 * @defgroup iic1 IIC Macros
 * @ingroup  iic
 * @defgroup iic2 IIC Standard Functions
 * @ingroup  iic
 * @defgroup iic3 Advanced IIC Functions
 * @ingroup  iic
 * @defgroup iic4 IIC Enumerations and Definitions
 * @ingroup  iic
 *
 * @file        mcalI2C.c
 * @brief       mcalI2C.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 12, 2020
 * @updated		by Prof Flaemig 2025-07-10
 *
 * I2C-Address used only with 7Bit Format
 * @version     0.3
 * @copyright   GNU Public License Version 3 (GPLv3)
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License, or any later version.<br>
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.<br>
 * You should have received a copy of the GNU General Public License along with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <mcalRCC.h>
#include <mcalI2C.h>


/**
 * Some macros which are permanently used to check the status registers
 *
 * @param  i2c : Address of the I2C component
 *
 * @note
 * tout = internal timeout counter
 */


#define I2C_WAIT_BUSY(i2c)                  ( { while (i2c->SR2 & I2C_SR2_BUSY) ; } )

#define I2C_START_COMPLETED(i2c)            ( { while (!(i2c->SR1 & I2C_SR1_SB)) ; } )
#define I2C_STOPP_COMPLETED(i2c)            ( { while (!(i2c->SR1 & I2C_SR1_STOPF)) ; } )
#define I2C_ADDRESS_COMPLETED(i2c)          ( { while (!(i2c->SR1 & I2C_SR1_ADDR)) ; } )

#define I2C_DUMMY_READ_SR1(i2c)             ( { i2c->SR1; } )
#define I2C_DUMMY_READ_SR2(i2c)             ( { i2c->SR2; } )
#define I2C_CHECK_RXBUF_NOT_EMPTY(i2c)      ( { while(!(i2c->SR1 & I2C_SR1_RXNE)) ; } )

#define I2C_BYTE_TRANSFER_FINISHED(i2c)     ( { while(!(i2c->SR1 & I2C_SR1_BTF)) ; })
#define I2C_RESET_ACK(i2c)                  ( { i2c->CR1 &= ~I2C_CR1_ACK_Msk; } )
#define I2C_SET_ACK(i2c)                    ( { i2c->CR1 |= I2C_CR1_ACK; } )
#define I2C_SET_POS(i2c)                    ( { i2c->CR1 |= I2C_CR1_POS; } )
#define I2C_RESET_POS(i2c)                  ( { i2c->CR1 &= ~I2C_CR1_POS_Msk; } )



static inline void __i2c_start(I2C_TypeDef *i2c)
{
	i2c->CR1 |= I2C_CR1_START;
	while (!(i2c->SR1 & I2C_SR1_SB));
}

static inline void __i2c_stop(I2C_TypeDef *i2c)
 {
 	i2c->CR1 |= I2C_CR1_STOP;
 	while (!(i2c->SR2 & I2C_SR2_BUSY));
 }

static inline void __i2c_dummy_read_SR1_SR2(I2C_TypeDef *i2c)
{
	while (!(i2c->SR1) | !(i2c->SR2))
	{
	;
	}
}


static inline void __i2c_Chk_TX_empty(I2C_TypeDef *i2c)
{

	uint32_t simpleDelay;
	while(!(i2c->SR1 & I2C_SR1_TXE));
	for (simpleDelay = 0UL; simpleDelay < 20UL; simpleDelay++)
	{
		;
	}

}


static bool i2cVerifyI2C(I2C_TypeDef *i2c)
{
    if ((I2C1 == i2c) || (I2C2 == i2c) || (I2C3 == i2c))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the I2C duty cycle.
 */
static bool i2cVerifyDutyCycle(I2C_DUTY_CYCLE_t dc)
{
    if ((I2C_DUTY_CYCLE_2 == dc) || (IC2_DUTY_CYCLE_16_9 == dc))
    {
        return true;
    }
    return false;
}


/**
 * @ingroup iic2
 * Activates the bus clock of the I2C component
 *
 * @param  *i2c   : Pointer to the I2C component
 *
*/
I2C_RETURN_CODE_t i2cSelectI2C(I2C_TypeDef *i2c)
{
    // Activate bus clock
    if (I2C1 == i2c)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    }
    else if (I2C2 == i2c)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    }
    else if (I2C3 == i2c)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    }

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Deinitializes the desired I2C component (turns bus clock off)
 *
 */
I2C_RETURN_CODE_t i2cDeselectI2C(I2C_TypeDef *i2c)
{
    if (i2cVerifyI2C(i2c) != true)
    {
        return I2C_INVALID_TYPE;
    }

    // Deactivate bus clock
    if (I2C1 == i2c)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN_Msk;
    }
    else if (I2C2 == i2c)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN_Msk;
    }
    else if (I2C3 == i2c)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN_Msk;
    }
    return I2C_OK;
}


/**
 * Function to set the I2C clock frequency.
 */
I2C_RETURN_CODE_t i2cSetClkSpd(I2C_TypeDef *i2c, I2C_CLOCKSPEED_t spd)
{
    if ( (I2C_CLOCK_50 == spd) || (I2C_CLOCK_100 == spd) ||
    	 (I2C_CLOCK_200 == spd)|| (I2C_CLOCK_400 == spd) || (I2C_CLOCK_1Mz == spd)		)
    {
    	I2C_WAIT_BUSY(i2c);
    	i2c->CCR = 0x00;                    // Reset Clock Control Register
    	i2c->CCR &= ~I2C_CCR_CCR_Msk;    	// deactive bus clock bits
    	i2c->CCR &= ~I2C_CCR_FS_Msk;    	// deactive fastMode I2C bus clock
    	switch (spd)
    	{
    		case I2C_CLOCK_50:
			{

				i2c->CCR |= 0x00A0;				// Select 50 kHz bus clock
				break;
			}
    		case I2C_CLOCK_100:
    		{
    			i2c->CCR |= 0x0050;				// Select 100 kHz bus clock
    			break;
    		}
    		case I2C_CLOCK_200:
    			{
       				i2c->CCR |= 0x0028;			// Select 200 kHz bus clock
    				break;
    			}
    		case I2C_CLOCK_400:
    		{
    			//i2c->CCR |= I2C_CCR_FS; 	    // fast Mode
    			i2c->CCR |= 0x0014;				// Select 400 kHz bus clock
    			break;
    		}
    		case I2C_CLOCK_1Mz:
			{
				i2c->CCR |= I2C_CCR_FS; 	    // fast Mode
				i2c->CCR |= 0x005;				// Select 800 kHz bus clock
				break;
			}
    		default:
    		{
    			i2c->CCR &= ~I2C_CCR_FS_Msk;    // Select 100 kHz bus clock
    			i2c->CCR |= 0x0050;
    			break;
    		}
    	}
    	 i2c->CR1 |= I2C_CR1_PE;            // Re-renable I2C component
    	return I2C_OK;
    }
    return I2C_INVALID_CLOCK_SPEED;
}

I2C_RETURN_CODE_t i2cInitI2C(I2C_TypeDef *i2c, I2C_DUTY_CYCLE_t duty, uint8_t trise, I2C_CLOCKSPEED_t clock)
{
	uint32_t pclock;
	if (i2cVerifyDutyCycle(duty) != true)
    {
        return I2C_INVALID_DUTY_CYCLE;
    }

    i2c->CR1 = I2C_CR1_SWRST;			// reset and clear reg
    i2c->CR1 = 0x0000;                  // Reset old CR1 settings
    i2c->CR1 &= ~I2C_CR1_PE_Msk;        // Disable I2C component

    pclock = rccGetPclk1Freq();
    i2c->CR2 = pclock / 1000000;		//

    i2c->TRISE = trise;                // Set max. rise time

    i2c->OAR1 |= (0x00 << 1);			 // set own address to 00 - not really used in master mode
    i2c->OAR1 |= (1 << 14); 			// bit 14 should be kept at 1 according to the datasheet

    i2cSetClkSpd(i2c, clock);			// set I2C Clockrate

    //i2c->CR1 |= I2C_CR1_PE;            // Re-renable I2C component

    i2cFindSlaveAddr(i2c, 1);			// first run find routine for Adr 0, work arround for result failure at first search run


    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sends a byte to the I2C slave w/o internal registers immediately.
 *
 * @param  *i2c   : Pointer to the component
 * @param   saddr : Address of the I2C slave
 * @param   data  : Byte that shall be sent
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented. This function shall be used
 * in the case when the desired I2C component provides only one internal
 * register.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>

 */
I2C_RETURN_CODE_t i2cSendByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t data)
{


	I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

	 __i2c_start(i2c);		         // Send I2C START signal
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent


    i2c->DR = saddr<<1;                 // Send 7Bit slave address
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    __i2c_Chk_TX_empty(i2c);         // Wait until the transmit buffer is empty

    i2c->DR = data;                     // Send data

    I2C_BYTE_TRANSFER_FINISHED(i2c);    // Wait until BTF Flag is set
    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2

    __i2c_stop(i2c);			       // Send STOP signal
    return I2C_OK;
}

/**
 * @ingroup iic3
 * Sends a byte to a functional register of the I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave
 * @param   regAddr : Address of the slave register
 * @param   data    : Byte that shall be sent
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * The text written in <b>bold</b> letters is required to address a device-specific register.<br>
 */
I2C_RETURN_CODE_t i2cSendByteToSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t data)
{
    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    __i2c_start(i2c);					 // send start condition
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr<<1;                    // Send slave 7Bit address
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    __i2c_Chk_TX_empty(i2c);         // Wait until the transmit buffer is empty

    i2c->DR = regAddr;                  // Set the address of the slave register
    __i2c_Chk_TX_empty(i2c);         // Wait until the transmit buffer is empty

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    i2c->DR = data;                     // Send data
    I2C_BYTE_TRANSFER_FINISHED(i2c);    // Wait until BTF Flag is set

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
      __i2c_stop(i2c);					  // send stop condition


    return I2C_OK;
}

/**
 * @ingroup iic3
 * Burst write data to I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave 7Bit
 * @param   regAddr : Address of the slave register
 * @param   data    : Byte that shall be sent
 * @param   len     : Number of data elements to be sent
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented
 *
*/
I2C_RETURN_CODE_t i2cBurstWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data, uint8_t len)
{
    uint8_t numBytes = 0;

    I2C_WAIT_BUSY(i2c);                 // Check whether the I2C bus is busy
    I2C_RESET_POS(i2c);                 // May only be active in 16-bit mode
    __i2c_start(i2c);					 // send start condition

    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr<<1;                 // Send 7Bit slave address
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    while (numBytes < len)
    {
        __i2c_Chk_TX_empty(i2c);
        __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
        i2c->DR = *data++;
        I2C_BYTE_TRANSFER_FINISHED(i2c);   // Wait until BTF Flag is set

        numBytes++;
    }

    I2C_BYTE_TRANSFER_FINISHED(i2c);   // Wait until BTF Flag is set
    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    __i2c_stop(i2c);					  // send stop condition

    return I2C_OK;
}


/**
 * @ingroup iic2
 * Reads a byte from the I2C slave w/o internal registers immediately.
 *
 * @param  *i2c   : Pointer to the component
 * @param   saddr : Address of the I2C slave 7Bit
 * @param  *data  : Pointer to the variable where the data shall be stored
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>

 */
I2C_RETURN_CODE_t i2cReadByte(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data)
{
    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    __i2c_start(i2c);					 // send start condition
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    i2c->DR = (saddr<<1) | 1;           // Resend 7 Bit slave addr with bit 0 = '1'
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_RESET_ACK(i2c);                 // Disable Acknowledge
    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    I2C_CHECK_RXBUF_NOT_EMPTY(i2c);     // Wait until receive buffer is no longer empty

    *data = (uint8_t) (i2c->DR);                    // Write data in variable


    __i2c_stop(i2c);
    return I2C_OK;
}

/**
 * @ingroup iic3
 * Reads a byte from a functional register of the I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave
 * @param   regAddr : Address of the slave register
 * @param  *data    : Address where the data shall be stored
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented. This function shall be used
 * in the case when the addresses I2C component provides only one internal
 * register.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 *
 */
I2C_RETURN_CODE_t i2cReadByteFromSlaveReg(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data)
{
    I2C_WAIT_BUSY(i2c);                 // Checks whether the I2C bus is busy

    __i2c_start(i2c);					 // send start condition
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr<<1;                    // Send with bit 0 = '0'
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2

    i2c->DR = regAddr;                  // Send address of the functional register
    __i2c_Chk_TX_empty(i2c);         	// Wait until transmit buffer is empty

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    i2c->CR1 |= I2C_CR1_START;          // Generate I2C RESTART
    I2C_START_COMPLETED(i2c);           // Checks whether the START signal has been sent

    i2c->DR = (saddr<<1) | 1;           // Resend slave addr with bit 0 = '1'
    I2C_ADDRESS_COMPLETED(i2c);         // Wait for ADDR ACK

    I2C_RESET_ACK(i2c);                 // Disable Acknowledge

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
    __i2c_stop(i2c);

    I2C_CHECK_RXBUF_NOT_EMPTY(i2c);     // Wait until receive buffer is no longer empty
    *data++ = i2c->DR;                  // Write data in variable

    return I2C_OK;
}

/**
 * @ingroup iic3
 * Burst read from I2C slave.
 *
 * @param  *i2c     : Pointer to the component
 * @param   saddr   : Address of the I2C slave
 * @param   regAddr : Address of the first slave register
 * @param  *data    : Address where the data shall be stored
 * @param   num     : Number of data elements to be read
 *
 * @return I2C_RETURN_CODE_t
 *
 * @note
 * Failure handling is not yet implemented
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>

 */
I2C_RETURN_CODE_t i2cBurstRegRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t regAddr, uint8_t *data, uint8_t num)
{
	I2C_WAIT_BUSY(i2c);
    //I2C_RESET_POS(i2c);                     // Must be used only in 16-bit transfer

    __i2c_start(i2c);					 // send start condition
    I2C_START_COMPLETED(i2c);           // Wait until START signal has been sent

    i2c->DR = saddr<<1;                        // Send with bit 0 = '0'
    I2C_ADDRESS_COMPLETED(i2c);             // Wait for ADDR ACK

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2

    i2c->DR = regAddr;                      // Send address of the functional register

    __i2c_Chk_TX_empty(i2c);             // Wait until transmit buffer is empty
    I2C_BYTE_TRANSFER_FINISHED(i2c);
    __i2c_dummy_read_SR1_SR2(i2c);
    i2c->CR1 |= I2C_CR1_START;          // Generate I2C RESTART
    I2C_START_COMPLETED(i2c);           // Checks whether the START signal has been sent
    i2c->DR = (saddr<<1) | 1;               // Resend 7Bit slave addr with bit 0 = '1'
    I2C_ADDRESS_COMPLETED(i2c);             // Wait for ADDR ACK

    I2C_SET_ACK(i2c);                       // Enable Acknowledge

    while (num > 0)                          // Start reading multiple values
    {
        if (num == 1U)                      // If there is only one byte left...
        {
            I2C_RESET_ACK(i2c);             // Disable acknowledge
            __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
            I2C_CHECK_RXBUF_NOT_EMPTY(i2c); // Wait until receive buffer is no longer empty

            *data++ = i2c->DR;              // Read data from data register
            break;
        }
        else                                // More than one byte left
        {
        	__i2c_dummy_read_SR1_SR2(i2c);
        	I2C_CHECK_RXBUF_NOT_EMPTY(i2c); // Wait until receive buffer is no longer empty
            (*data++) = i2c->DR;            // Read data from data register
            num--;
        }
    }
    __i2c_dummy_read_SR1_SR2(i2c);
    //I2C_BYTE_TRANSFER_FINISHED(i2c);
    __i2c_stop(i2c);

    return I2C_OK;
}

I2C_RETURN_CODE_t i2cBurstRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data, uint8_t num)
{
	I2C_WAIT_BUSY(i2c);
    I2C_RESET_POS(i2c);                     // Must be used only in 16-bit transfer

    __i2c_start(i2c);					 // send start condition

    i2c->DR = (saddr<<1) | 1;               // Resend 7Bit slave addr with bit 0 = '1'
    I2C_ADDRESS_COMPLETED(i2c);             // Wait for ADDR ACK

    __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2

    I2C_SET_ACK(i2c);                       // Enable Acknowledge

    while (num > 0)                          // Start reading multiple values
    {
        if (num == 1U)                      // If there is only one byte left...
        {
            I2C_RESET_ACK(i2c);             // Disable acknowledge
            __i2c_dummy_read_SR1_SR2(i2c);		// Reset SR1 & SR2
            __i2c_stop(i2c);
            I2C_CHECK_RXBUF_NOT_EMPTY(i2c); // Wait until receive buffer is no longer empty
            *data++ = i2c->DR;              // Read data from data register
            break;
        }
        else                                // More than one byte left
        {
            I2C_CHECK_RXBUF_NOT_EMPTY(i2c); // Wait until receive buffer is no longer empty
            (*data++) = i2c->DR;            // Read data from data register
            num--;
        }
    }

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Enables the desired I2C peripheral component.
 *
 * @param  *i2c   : Pointer to the I2C component
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">PE</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cEnableDevice(I2C_TypeDef *i2c)
{
    i2c->CR1 |= I2C_CR1_PE;
    return I2C_OK;
}

/**
 * @ingroup iic2
 * Disables the desired I2C peripheral component.
 *
 * @param  *i2c   : Pointer to the I2C component
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CR1</td>
 *          <td rowspan="1">PE</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cDisableDevice(I2C_TypeDef *i2c)
{
    i2c->CR1 &= ~I2C_CR1_PE_Msk;
    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sets the peripheral clock frequency. Don't mix it up with the data transfer speed!
 *
 * @param  *i2c  : Pointer to the I2C component
 * @param   pclk : Clock frequency of the desired I2C component in MHz
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CR2</td>
 *          <td rowspan="1">FREQ</td>
 *          <td rowspan="1">5...0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSetPeripheralClockFreq(I2C_TypeDef *i2c, uint8_t pclk)
{
	uint32_t pclock;
	i2c->CR2 &= ~I2C_CR2_FREQ_Msk;
    pclock = rccGetPclk1Freq();
    i2c->CR2 = pclock / 1000000;
    //i2c->CR2 |= pclk;

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sets the duty cycle of the desired I2C component.
 *
 * @param  *i2c  : Pointer to the I2C component
 * @param   duty : Duty cycle. Can be IC_DUTY_CYCLE 2 or DUTY_CYCLE_16_9.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>CCR</td>
 *          <td rowspan="1">DUTY</td>
 *          <td rowspan="1">15</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSetDutyCycle(I2C_TypeDef *i2c, I2C_DUTY_CYCLE_t duty)
{
    i2c->CCR &= ~I2C_CCR_DUTY_Msk;
    i2c->CCR |= duty << I2C_CCR_DUTY_Pos;

    return I2C_OK;
}

/**
 * @ingroup iic2
 * Sets the maximum rise time in Fm/Sm naster mode.
 *
 * @param  *i2c      : Pointer to the I2C component
 * @param   riseTime : Valid values are { 0 ... 63 }.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>TRISE</td>
 *          <td rowspan="1">TRISE</td>
 *          <td rowspan="1">5 ... 0</td>
 *      </tr>
 * </table>
 */
I2C_RETURN_CODE_t i2cSetRiseTime(I2C_TypeDef *i2c, uint8_t riseTime)
{
    i2c->TRISE &= ~I2C_TRISE_TRISE_Msk;

    if (riseTime >= 0 && riseTime <= 63)
    {
        i2c->TRISE |= riseTime;
    }
    else
    {
        i2c->TRISE |= 0x11;     // This is a reliable value
    }
    return I2C_OK;
}

/**
 * @ingroup iic2
 * Searches I2C peripheral components and returns their I2C address. It returns 0 if the desired address is free.
 *
 * @param  *i2c     : Pointer to the I2C component
 * @param   i2cAddr : The I2C address which shall be tested
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>-</td>
 *          <td rowspan="1">-</td>
 *          <td rowspan="1">-</td>
 *      </tr>
 * </table>
 */
uint8_t i2cFindSlaveAddr(I2C_TypeDef *i2c, uint8_t i2cAddr)
{
    uint32_t simpleDelay;

    __i2c_start(i2c);
    /*
    i2c->CR1 |= I2C_CR1_START;
    while (!(i2c->SR1 & I2C_SR1_SB))
    {
        ;
    }
*/
    i2c->DR = i2cAddr<<1;
    __i2c_dummy_read_SR1_SR2(i2c);
    /*
    while (!(i2c->SR1) | !(i2c->SR2))
    {
        ;
    }
*/

    i2c->CR1 |= I2C_CR1_STOP;
    for (simpleDelay = 0UL; simpleDelay < 1000UL; simpleDelay++)
    {
        ;
    }

    if (i2c->SR1 & I2C_SR1_ADDR)
    {
        return i2cAddr;
    }
    else
    {
        return 0;
    }
}

I2C_RETURN_CODE_t i2cResetDevice(I2C_TypeDef *i2c)
{
    I2C_WAIT_BUSY(i2c);
    i2c->CR1 |= I2C_CR1_SWRST;

    return I2C_OK;
}
