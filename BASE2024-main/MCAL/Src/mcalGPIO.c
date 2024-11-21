/**
 * @defgroup gpio  GPIO Functions (mcalGPIO.h/.c)
 * @defgroup gpio0 Deprecated GPIO Functions
 * @ingroup  gpio
 * @defgroup gpio1 All Functions
 * @ingroup  gpio
 * @defgroup gpio2 GPIO Enumerations and Definitions
 * @ingroup  gpio
 *
 * @file        mcalGPIO.c
 * @brief       mcalGPIO.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 11, 2020
 *
 * @version     0.1
 * @copyright   Ralf Jesse
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

#include <stdint.h>
#include <stdbool.h>

#include <mcalGPIO.h>

/**
 * Function to verify the integrity of the **port** parameter.
 *
 * @param *port : Pointer to the GPIO port
 */
bool gpioVerifyPort(GPIO_TypeDef *port)
{
    if ((GPIOA == port) || (GPIOB == port) || (GPIOC == port) ||
        (GPIOD == port) || (GPIOE == port) || (GPIOH == port))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the integrity of the **pin** parameter.
 *
 * @param pin : The pin number of any of the GPIO ports
 */
bool gpioVerifyPin(PIN_NUM_t pin)
{
    if ((PIN0  == pin) || (PIN1  == pin) || (PIN2  == pin) || (PIN3  == pin) ||
        (PIN4  == pin) || (PIN5  == pin) || (PIN6  == pin) || (PIN7  == pin) ||
        (PIN8  == pin) || (PIN9  == pin) || (PIN10 == pin) || (PIN11 == pin) ||
        (PIN12 == pin) || (PIN13 == pin) || (PIN14 == pin) || (PIN15 == pin))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the integrity of the **af** parameter.
 *
 * @param af : The number of the alternative function.
 */
bool gpioVerifyAltFunc(ALT_FUNC_t af)
{
    if ((AF0  == af) || (AF1  == af) || (AF2  == af) || (AF3  == af) || (AF4  == af) ||
        (AF5  == af) || (AF6  == af) || (AF7  == af) || (AF8  == af) || (AF9  == af) ||
        (AF10 == af) || (AF11 == af) || (AF12 == af) || (AF13 == af) || (AF14 == af) ||
        (AF15 == af))
    {
        return true;
    }

    return false;
}

/**
 * Function to verify the integrity of the **pinMode** parameter.
 *
 * @param pinMode : The operational mode in which the GPIO pin shall work.
 */
bool gpioVerifyPinMode(PIN_MODE_t pinMode)
{
    if ((INPUT == pinMode) || (OUTPUT == pinMode) || (ALTFUNC == pinMode) || (ANALOG == pinMode))
    {
        return true;
    }

    return false;
}

/**
 * Function to verify the integrity of the **outType** parameter.
 *
 * @param outType : Verifies whether the passed value is PUSHPULL or OPENDRAIN.
 */
bool gpioVerifyOutputType(OUTPUT_TYPE_t outType)
{
    if ((PUSHPULL == outType) || (OPENDRAIN == outType))
    {
        return true;
    }

    return false;
}

/**
 * Function to verify the integrity of the Pullup/Pulldown parameter.
 *
 * @param mode : Verifies the Pullup/Pulldown mode
 */
bool gpioVerifyPushPullMode(PUPD_MODE_t mode)
{
    if ((NO_PULLUP_PULLDOWN == mode) || (PULLUP == mode) || (PULLDOWN == mode))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the speed parameter.
 *
 * @param speed : Verification of the speed parameter
 */
bool gpioVerifyOutputSpeed(GPIO_SPEED_t speed)
{
    if ((LOW_SPEED == speed) || (MEDIUM_SPEED == speed) || (FAST_SPEED == speed) || (HIGH_SPEED == speed))
    {
        return true;
    }

    return false;
}

/**
 * @ingroup gpio0
 * This function is used to activate the bus clock of the port. It verifies the
 * validity of the given GPIO port. If the validation succeeds the bus clock
 * is activated.
 *
 * @param
 * *port : Pointer to the GPIO port
 *
 * @return GPIO_RETURN_CODE_t
 *
 * @note
 * This function shall not be used any longer. There is an exact copy of this function named with gpioSelectPort(GPIO_TypeDef *port).
 * This new function was implemented to align it with the naming scheme of other peripheral components.
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
 *          <td>AHB1ENR</td>
 *          <td rowspan="1">GPIOA...GPIOH</td>
 *          <td rowspan="1">7...0</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioInitPort(GPIO_TypeDef *port)
{
    if (gpioVerifyPort(port) == true)
    {
        if (GPIOA == port)
        {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        }
        else if (GPIOB == port)
        {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        }
        else if (GPIOC == port)
        {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        }
        else if (GPIOD == port)
        {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        }
        else if (GPIOE == port)
        {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
        }
        else if (GPIOH == port)
        {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
        }
        return GPIO_OK;
    }
    return GPIO_INVALID_PORT;
}

/**
 * @ingroup gpio1
 * This function is used to activate the bus clock of the port. It verifies the
 * validity of the given GPIO port. If the validation succeeds the bus clock
 * is activated.
 *
 * @param
 * *port : Pointer to the GPIO port
 *
 * @return GPIO_RETURN_CODE_t
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
 *          <td>AHB1ENR</td>
 *          <td rowspan="1">GPIOA...GPIOH</td>
 *          <td rowspan="1">7...0</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioSelectPort(GPIO_TypeDef *port)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }

    if (GPIOA == port)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    else if (GPIOB == port)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    }
    else if (GPIOC == port)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }
    else if (GPIOD == port)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    }
    else if (GPIOE == port)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    }
    else if (GPIOH == port)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    }
    return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Use this function to deselect a GPIO port when it is not longer used. In fact it turns the bus clock off.
 *
 * @param
 * *port : Pointer to the GPIO port
 *
 * @return GPIO_RETURN_CODE_t
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
 *          <td>AHB1ENR</td>
 *          <td rowspan="1">GPIOA...GPIOH</td>
 *          <td rowspan="1">7...0</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioDeselectPort(GPIO_TypeDef *port)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }

    if (GPIOA == port)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN_Msk;
    }
    else if (GPIOB == port)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN_Msk;
    }
    else if (GPIOC == port)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN_Msk;
    }
    else if (GPIOD == port)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN_Msk;
    }
    else if (GPIOE == port)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN_Msk;
    }
    else if (GPIOH == port)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN_Msk;
    }
    return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Initializes the operational mode of a GPIO pin. It verifies the validity
 * of all parameters. Returns GPIO_OK if all parameters are valid.
 *
 *
 * @param  *port : Pointer to the GPIO port
 * @param   pin  : Number of the pin
 * @param   mode : Operational mode of the pin
 * @return GPIO_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>MODER</td>
 *          <td rowspan="1">MODER15...MODER0</td>
 *          <td rowspan="1">31...0</td>
 *          <td rowspan="1">Sets the pin function either to input, output, alternate function, or analog mode</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioSelectPinMode(GPIO_TypeDef *port, PIN_NUM_t pin, PIN_MODE_t mode)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }
    if (gpioVerifyPinMode(mode) != true)
    {
        return GPIO_INVALID_PINMODE;
    }

    // Set pin to default (INPUT)
	port->MODER &= ~(0b11 << (pin * 2));

	// If mode is not INPUT --> Select other setting
	if (INPUT != mode)
	{
		port->MODER |= (mode << (pin * 2));
	}

	return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Sets the GPIO pin to High level. Verifies the integrity of the port and pin.
 *
 * @param  *port : Pointer to the GPIO port
 * @param   pin  : Number of the GPIO pin
 * @return GPIO_RETURN_CODE_t
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
 *          <td>BSRR</td>
 *          <td rowspan="1">BR15...BR0</td>
 *          <td rowspan="1">31...16</td>
 *      </tr>
 *      <tr>
 *          <td>BSRR</td>
 *          <td rowspan="1">BS15...BS0</td>
 *          <td rowspan="1">15...0</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioSetPin(GPIO_TypeDef *port, PIN_NUM_t pin)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }

	port->BSRR = 1 << pin;

	return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Resets the GPIO pin to low level. Verifies the integrity of the port and pin.
 *
 * @param  *port : Pointer to the GPIO port
 * @param   pin  : Number of the GPIO pin
 * @return GPIO_RETURN_CODE_t
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
 *          <td>BSRR</td>
 *          <td rowspan="1">BR15...BR0</td>
 *          <td rowspan="1">31...16</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioResetPin(GPIO_TypeDef *port, PIN_NUM_t pin)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }

	port->BSRR = (1 << (pin + 16));

	return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Toggles the state of the GPIO pin. Verifies the integrity of the port and pin.
 *
 * @param  *port : Pointer to the GPIO port
 * @param   pin  : Number of the GPIO pin
 * @return GPIO_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>BSRR</td>
 *          <td rowspan="1">BR15...BR0 / BS15...BS0</td>
 *          <td rowspan="1">31...0</td>
 *          <td rowspan="1">Calls the functions gpioSetPin() and gpioResetPin()</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioTogglePin(GPIO_TypeDef *port, PIN_NUM_t pin)
{
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }
    if (gpioGetPinState(port, pin))
    {
       gpioResetPin(port, pin);
    }
    else
    {
        gpioSetPin(port, pin);
    }

    return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Selects the alternate function (AF) of the GPIO pin. Verifies the integrity of the port, pin
 * and the alternate function parameter.
 *
 * @note
 * AFR is a two-element array of the GPIO_TypeDef structure. Since
 * AFR[1] starts at bit 0 we must subtract '8' from the pin number.
 * Finally, the result must be multiplied by '4' since four bits
 * must be moved to the left.
 *
 * @param  *port : Pointer to the GPIO port
 * @param   pin  : Number of the GPIO pin
 * @param   af   : Number of the alternative function
 * @return GPIO_RETURN_CODE_t
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
 *          <td>AFRL</td>
 *          <td rowspan="1">AFRL7...AFRL0</td>
 *          <td rowspan="1">31...0</td>
 *      </tr>
 *      <tr>
 *          <td>AFRH</td>
 *          <td rowspan="1">AFRH15...AFRH8</td>
 *          <td rowspan="1">31...0</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioSelectAltFunc(GPIO_TypeDef *port, PIN_NUM_t pin, ALT_FUNC_t af)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }
    if (gpioVerifyAltFunc(af) != true)
    {
        return GPIO_INVALID_ALTFUNC;
    }

	// Dont't forget to select the alternative function mode before...
	gpioSelectPinMode(port, pin, ALTFUNC);

	if (pin <= PIN7)
	{
		port->AFR[0] &= ~(0x0F << (pin * 4));       // Reset previously selected AF
		port->AFR[0] |= af << (pin * 4);            // Set AFR[0] to the new AF
	}
	else
	{
		// !!! Subtract 8 from pin number since we start at bit 0 !!!
		port->AFR[1] &= ~(0x0F << ((pin - 8) * 4)); // Reset previously selected AF.
		port->AFR[1] |= af << ((pin - 8) * 4);      // Set AFR[1] to the new AF
	}

	return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Sets bits 0 to 7 of the GPIO. Bits 8 to 15 remain unchanged!
 *
 * @param  *port : Pointer to the GPIO port
 * @param   byte : Sets bits 0 to 7 according to the value of byte
 * @return GPIO_RETURN_CODE_t
 *
 * @note
 * This function is not interrupt-safe!
 */
GPIO_RETURN_CODE_t gpioSetLowerByte(GPIO_TypeDef *port, uint8_t byte)
{
    port->ODR &= 0xFF00;                            // Does not touch bits 8 to 15
    port->ODR |= byte;
    return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Sets bits 8 to 15 of the GPIO. Bits 0 to 7 remain unchanged!
 *
 * @param  *port : Pointer to the GPIO port
 * @param   byte : Sets bits 0 to 7 according to the value of byte
 * @return GPIO_RETURN_CODE_t
 *
 * @note
 * This function is not interrupt-safe!
 */
GPIO_RETURN_CODE_t gpioSetUpperByte(GPIO_TypeDef *port, uint8_t byte)
{
    port->ODR &= 0x00FF;                            // Does not touch bits 0 to 7
    port->ODR |= (byte << 8);
    return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Sets bits 0 to 15 of the GPIO. Overrides all bits!
 *
 * @param  *port : Pointer to the GPIO port
 * @param   word : Sets bits 0 to 15 according to the value of byte
 * @return GPIO_RETURN_CODE_t
 *
 * @note
 * This function is not interrupt-safe!
 */
GPIO_RETURN_CODE_t gpioSetWord(GPIO_TypeDef *port, uint16_t word)
{
    port->ODR = word;
    return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Sets the output type of the GPIO pin either as PUSHPULL or Open Drain. Verifies the
 * port, pin and the output type.
 *
 * @param  *port    : Pointer to the GPIO port
 * @param   pin     : Number of the GPIO pin
 * @param   outType : Selects between Push/Pull or Open Drain
 * @return GPIO_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>OTYPER</td>
 *          <td rowspan="1">OT15...OT0</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Sets the output type either to push-pull or to Open drain</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioSetOutputType(GPIO_TypeDef *port, PIN_NUM_t pin, OUTPUT_TYPE_t outType)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }
    if (gpioVerifyOutputType(outType) != true)
    {
        return GPIO_INVALID_OUTPUT_TYPE;
    }

    port->OTYPER &= ~(1 << pin);   // Use pin as push-pull output
    if (OPENDRAIN == outType)
    {
        port->OTYPER |= (1 << pin);
	}

	return GPIO_OK;
}

/**
 * @ingroup gpio1
 * This function selects the push/pull resistor type. Valid values are
 * *NO_PULLUP_PULLDOWN*, *PULLUP*, and *PULLDOWN*.
 *
 * @param  *port : Pointer to the GPIO port
 * @param   pin  : Number of the GPIO pin
 * @param   pupd : Different pull-up/pull-down settings
 * @return GPIO_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>PUPDR</td>
 *          <td rowspan="1">PUPDR15...PUPDR0</td>
 *          <td rowspan="1">31...0</td>
 *          <td rowspan="1">OTYPER must be set to push-pull</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioSelectPushPullMode(GPIO_TypeDef *port, PIN_NUM_t pin, PUPD_MODE_t pupd)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }
    if (gpioVerifyPushPullMode(pupd) != true)
    {
        return GPIO_INVALID_PUSH_PULL_MODE;
    }

    port->PUPDR &= ~(0x03 << (pin * 2));        // Disable PU/PD

    if (port->OTYPER == PUSHPULL)
	{
		if (PULLUP == pupd)
		{
			port->PUPDR |= (PULLUP << (pin *2));
		}
		else if (PULLDOWN == pupd)
		{
			port->PUPDR |= (PULLDOWN << (pin *2));
		}
	}

    return GPIO_OK;
}

/**
 * @ingroup gpio1
 * Sets the speed of a GPIO output pin. Verifies the validity of port, pin and speed
 * parameters.
 *
 * @param  *port  : Pointer to the GPIO port
 * @param   pin   : Number of the GPIO pin
 * @param   speed : Speed setting
 * @return nothing
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
 *          <td>OSPEEDR</td>
 *          <td rowspan="1">OSPEEDR15...OSPEEDR0</td>
 *          <td rowspan="1">31...0</td>
 *      </tr>
 * </table>
 */
GPIO_RETURN_CODE_t gpioSetOutputSpeed(GPIO_TypeDef *port, PIN_NUM_t pin, GPIO_SPEED_t speed)
{
    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if (gpioVerifyPin(pin) != true)
    {
        return GPIO_INVALID_PIN;
    }
    if (gpioVerifyOutputSpeed(speed) != true)
    {
        return GPIO_INVALID_OUTPUT_SPEED;
    }

    port->OSPEEDR &= (LOW_SPEED << (pin * 2));	// Default: Low speed

	if (MEDIUM_SPEED == speed)
	{
		port->OSPEEDR |= (MEDIUM_SPEED << (pin * 2));
	}
	else if (FAST_SPEED == speed)
	{
		port->OSPEEDR |= (FAST_SPEED << (pin * 2));
	}
	else if (HIGH_SPEED == speed)
	{
		port->OSPEEDR |= (HIGH_SPEED << (pin * 2));
	}

	return GPIO_OK;
}

/**
 * @ingroup gpio1
 * This function is used to obtain the current state of the given pin. It verifies
 * that port and pin are valid and returns a corresponding error code. The state of
 * the pin is stored in a variable which is passed as pointer to this function.
 *
 * @param  *port     : Pointer to the GPIO port
 * @param   pin      : Number of the GPIO pin
 * @return bool
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>IDR</td>
 *          <td rowspan="1">IDR15...IDR0</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Returns the state of the desired pin</td>
 *      </tr>
 * </table>
 *
 * @note
 * The function does not return a GPIO_RETURN_CODE_t any longer. Instead it immediately
 * returns the state of the desired pin.
 */
bool gpioGetPinState(GPIO_TypeDef *port, PIN_NUM_t pin)
{
    return (port->IDR & (1 << pin)) == (1<<pin);
}

/**
 * @ingroup gpio1
 * This function is used to obtain the current state of the given port. It verifies
 * that the port parameter is valid and stores the result at the address of the portVal
 * variable.
 *
 * @param  *port    : Pointer to the GPIO port
 * @return uint16_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>IDR</td>
 *          <td rowspan="1">IDR15...IDR0</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Returns the state of the desired port</td>
 *      </tr>
 * </table>
 *
 * @note
 * The function does not return a GPIO_RETURN_CODE_t any longer. Instead it immediately
 * returns the state of the desired port.
 */
uint16_t gpioGetPortVal(GPIO_TypeDef *port)
{
	return (port->IDR);
}
