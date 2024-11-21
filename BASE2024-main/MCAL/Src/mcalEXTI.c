/**
 * @defgroup exti  External Interrupts (mcalEXTI.h/.c)
 * @defgroup exti1 All EXTI Functions
 * @ingroup  exti
 * @defgroup exti2 EXTI Enumerations and Definitions
 * @ingroup  exti
 *
 * @file        mcalEXTI.c
 * @brief       mcalEXTI.c is part of the MCAL library for STM32F4xx.
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

#include <mcalEXTI.h>

/**
 * Function to verify the integrity of the **irqNum** parameter.
 *
 * @param irqNum : Number of the EXTI interrupt (declared in maclEXTI.h)
 */
bool extiVerifyIrqNum(EXTI_IRQ_NUM irqNum)
{
    if ((EXTI_PIN0  == irqNum) || (EXTI_PIN1  == irqNum) || (EXTI_PIN2  == irqNum) || (EXTI_PIN3  == irqNum) ||
        (EXTI_PIN4  == irqNum) || (EXTI_PIN5  == irqNum) || (EXTI_PIN6  == irqNum) || (EXTI_PIN7  == irqNum) ||
        (EXTI_PIN8  == irqNum) || (EXTI_PIN9  == irqNum) || (EXTI_PIN10 == irqNum) || (EXTI_PIN11 == irqNum) ||
        (EXTI_PIN12 == irqNum) || (EXTI_PIN13 == irqNum) || (EXTI_PIN14 == irqNum) || (EXTI_PIN15 == irqNum) ||
        (EXTI_VOLTAGE_DETECTION == irqNum) || (EXTI_RTC_ALARM == irqNum)  || (EXTI_USB_OTG_FS == irqNum)     ||
        (EXTI_USB_OTG_HS == irqNum)        || (EXTI_RTC_TAMPER == irqNum) || (EXTI_RTC_WAKEUP == irqNum))
    {
        return true;
    }
    return false;
}

/**
 * Function to verify the integrity of the **trigger** parameter.
 */
bool extiVerifyTrigger(EXTI_TRIGGER trigger)
{
    if ((RISING_EDGE == trigger) || (FALLING_EDGE == trigger) || (RISING_AND_FALLING == trigger))
    {
        return true;
    }
    return false;
}

/**
 * @ingroup exti1
 * Resets all EXTI settings
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
 *          <td>EXTICR4/3/2/1</td>
 *          <td rowspan="1">EXTI0...EXTI15</td>
 *          <td rowspan="1">15...0</td>
 *      </tr>
 * </table>
 */
void extiInit(void)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        SYSCFG->EXTICR[i] = 0x0000;
    }
}

/**
 * @ingroup exti1
 * Sets the selected EXTI according to the port/pin
 *
 * @param   *port : Pointer to the GPIO port
 * @param    pin  : Number of the pin which shall be used as EXTI
 * @return  EXTI_IRQ_NUM
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
 *          <td>EXTICR4/3/2/1</td>
 *          <td rowspan="1">EXTI0...EXTI15</td>
 *          <td rowspan="1">15...0</td>
 *      </tr>
 * </table>
 */
EXTI_IRQ_NUM extiConfigIrq(GPIO_TypeDef *port, PIN_NUM_t pin)
{
    uint8_t index = 0;
    uint8_t shift = 0;
    uint8_t mask  = 0;

    if (gpioVerifyPort(port) != true)
    {
        return GPIO_INVALID_PORT;
    }
    if ((gpioVerifyPin(pin)) != true)
    {
        return GPIO_INVALID_PIN;
    }

    switch ((uint8_t) pin)
    {
        case PIN0:
        case PIN1:
        case PIN2:
        case PIN3:
            shift = pin * 4;        // Every pin uses four bits
            index = 0;              // Set the array-index of SYSCFG->EXTICR[]
            break;

        case PIN4:
        case PIN5:
        case PIN6:
        case PIN7:
            shift = (pin - 4) * 4;
            index = 1;
            break;

        case PIN8:
        case PIN9:
        case PIN10:
        case PIN11:
            shift = (pin - 8) * 4;
            index = 2;
            break;

        case PIN12:
        case PIN13:
        case PIN14:
        case PIN15:
            shift = (pin - 12) * 4;
            index = 3;
            break;
    }

    if (GPIOA == port)
    {
        mask = PORT_A;              // Inversion of the port mask
    }
    else if (GPIOB == port)
    {
        mask = PORT_B;
    }
    else if (GPIOC == port)
    {
        mask = PORT_C;
    }
    else if (GPIOD == port)
    {
        mask = PORT_D;
    }
    else if (GPIOE == port)
    {
        mask = PORT_E;
    }
    else if (GPIOH == port)
    {
        mask = PORT_H;
    }

    SYSCFG->EXTICR[index] |= (mask << shift);

    return EXTI_OK;
}

/**
 * @ingroup exti1
 * Enables an EXTI interrupt
 *
 * @param   irqNum : One from 23 EXTI IRQs
 * @return EXTI_IRQ_NUM none
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
 *          <td>IMR</td>
 *          <td rowspan="1">MR22...MR0</td>
 *          <td rowspan="1">22...0</td>
 *      </tr>
 * </table>
 */
EXTI_IRQ_NUM extiEnableIrq(EXTI_IRQ_NUM irqNum)
{
    if (extiVerifyIrqNum(irqNum) != true)
    {
        return EXTI_INVALID_IRQNUM;
    }
    EXTI->IMR |= 1 << irqNum;
    return EXTI_OK;
}

/**
 * @ingroup exti1
 * Disables an EXTI interrupt
 *
 * @param   irqNum : One from 23 EXTI IRQs
 * @return EXTI_IRQ_NUM
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
 *          <td>IMR</td>
 *          <td rowspan="1">MR22...MR0</td>
 *          <td rowspan="1">22...0</td>
 *      </tr>
 * </table>
 */
EXTI_IRQ_NUM extiDisableIrq(EXTI_IRQ_NUM irqNum)
{
    if (extiVerifyIrqNum(irqNum) != true)
    {
        return EXTI_INVALID_IRQNUM;
    }
    EXTI->IMR &= ~(1 << irqNum);
    return EXTI_OK;
}

/**
 * @ingroup exti1
 * Sets the trigger mode of the selected EXTI.
 *
 * @param   irqNum  : One from 23 EXTI IRQs
 * @param   trigger : rising only/falling only/rising + falling
 * @return EXTI_IRQ_NUM
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
 *          <td>RTSR</td>
 *          <td rowspan="1">TR22...TR0</td>
 *          <td rowspan="1">22...0</td>
 *      </tr>
 *      <tr>
 *          <td>FTSR</td>
 *          <td rowspan="1">TF22...TF0</td>
 *          <td rowspan="1">22...0</td>
 *      </tr>
 * </table>
 */
EXTI_IRQ_NUM extiSetTriggerEdge(EXTI_IRQ_NUM irqNum, EXTI_TRIGGER trigger)
{
    if (extiVerifyIrqNum(irqNum) != true)
    {
        return EXTI_INVALID_IRQNUM;
    }

    if (extiVerifyTrigger(trigger) != true)
    {
        return EXTI_INVALID_TRIGGER;
    }

    if (RISING_EDGE == trigger)
    {
        EXTI->RTSR |= 1 << irqNum;      // Enable rising edge
        EXTI->FTSR &= ~(1 << irqNum);   // Disable falling edge
    }
    else if (FALLING_EDGE == trigger)
    {
        EXTI->FTSR |= 1 << irqNum;      // Enable falling edge
        EXTI->RTSR &= ~(1 << irqNum);   // Disable rising edge
    }
    else if (RISING_AND_FALLING == trigger)
    {
        EXTI->RTSR |= 1 << irqNum;      // Enable rising edge
        EXTI->FTSR |= 1 << irqNum;      // Enable falling edge
    }
    return EXTI_OK;
}

/**
 * @ingroup exti1
 * Resets the desired EXTI IRQ pending flag.
 *
 * @param   irqNum  : One from 23 EXTI IRQs
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
 *          <td>PR</td>
 *          <td rowspan="1">PR22...PR0</td>
 *          <td rowspan="1">22...0</td>
 *      </tr>
 * </table>
 */
void extiResetPendingIRQ(EXTI_IRQ_NUM irqNum)
{
    EXTI->PR |= irqNum;
}
