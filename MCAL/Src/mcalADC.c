/**
 * @defgroup adc  ADC Functions (mcalADC.h/.c)
 * @defgroup adc0 Deprecated ADC Functions
 * @ingroup  adc
 * @defgroup adc1 All ADC Functions
 * @ingroup  adc
 * @defgroup adc2 Common ADC Functions
 * @ingroup  adc
 * @defgroup adc3 ADC Analogue Watchdog Funtions
 * @ingroup  adc
 * @defgroup adc4 ADC Enumerations and Definitions
 * @ingroup  adc
 *
 * @file        mcalADC.c
 * @brief       mcalADC.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        December 18, 2020
 * edited by	Tobias Flämig
 * @version     0.3
 * @copyright   GNU Public License Version 3 (GPLv3)
 *
 * The STM32F4xx micro controllers support up to three different timer classes:
 * *Basic timers*, *General-purpose timers*, and *Advanced-control timers*. This
 * is a collection of functions which supports the usage of all timers.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * @note
 * The capture/compare modes are still under construction. Currently, it is
 * possible to assign channels to timers which do not provide that channels.
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
#include <stddef.h>

#include <stm32f4xx.h>
#include <mcalADC.h>

static bool adcVerifyResolution(ADC_RESOLUTION_t res);
static bool adcVerifyInterruptType(ADC_IRQTYPES_t irq);
static bool adcVerifyChannelNum(ADC_CHANNEL_t chn);
static bool adcVerifyNumSamplingCycles(ADC_SAMPLING_CYCLES_t cycles);
static bool adcVerifyAnalogWatchdogType(ADC_WATCHDOG_t wdt);

// Internal (static) functions
static bool adcVerifyResolution(ADC_RESOLUTION_t res)
{
    if ((ADC_RES_6BIT == res) || (ADC_RES_8BIT == res) || (ADC_RES_10BIT == res) || (ADC_RES_12BIT == res))
    {
        return true;
    }
    return false;
}

static bool adcVerifyInterruptType(ADC_IRQTYPES_t irq)
{
    if ((ADC_IRQ_OVERRUN == irq) || /*(ADC_WATCHDOG_STATUS_ON == irq) ||*/(ADC_EOC_INJECTED_GRP == irq)
        || (ADC_EOC_REGULAR_GRP == irq))
    {
        return true;
    }

    return false;
}

static bool adcVerifyChannelNum(ADC_CHANNEL_t chn)
{
    switch (chn)
    {
        case ADC_CHN_0:
        case ADC_CHN_1:
        case ADC_CHN_2:
        case ADC_CHN_3:
        case ADC_CHN_4:
        case ADC_CHN_5:
        case ADC_CHN_6:
        case ADC_CHN_7:
        case ADC_CHN_8:
        case ADC_CHN_9:
        case ADC_CHN_10:
        case ADC_CHN_11:
        case ADC_CHN_12:
        case ADC_CHN_13:
        case ADC_CHN_14:
        case ADC_CHN_15:
        case ADC_CHN_16:
        case ADC_CHN_17:
        case ADC_CHN_18:
            return true;
            break;
    }

    return false;
}

static bool adcVerifyNumSamplingCycles(ADC_SAMPLING_CYCLES_t cycles)
{
    switch (cycles)
    {
        case SAMPLE_CYCLES_3:
        case SAMPLE_CYCLES_15:
        case SAMPLE_CYCLES_28:
        case SAMPLE_CYCLES_56:
        case SAMPLE_CYCLES_84:
        case SAMPLE_CYCLES_112:
        case SAMPLE_CYCLES_144:
        case SAMPLE_CYCLES_480:
            return true;
    }

    return false;
}

static bool adcVerifyAnalogWatchdogType(ADC_WATCHDOG_t wdt)
{
    if ((NO_ADC_WATCHDDOG == wdt) || (ALL_INJECTED_CHANNELS == wdt) || (ALL_REGULAR_CHANNELS == wdt)
        || (ALL_REGULAR_AND_INJECTED_CHANNELS == wdt) || (SINGLE_INJECTED_CHANNEL == wdt)
        || (SINGLE_REGULAR_CHANNEL == wdt) || (SINGLE_REGULAR_AND_INJECTED_CHANNEL == wdt))
    {
        return true;
    }

    return false;
}

// Public functions of the MCAL library

/********************************************************************************************
 * Outdated implementation (deprecated)                                                     *
 *******************************************************************************************/

/**
 * @ingroup adc0
 * Turns the bus clock of the ADC on.
 *
 * @param  *adc    : Pointer to the ADC component
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
 *          <td>APB2ENR</td>
 *          <td rowspan="1">ADC1/2/3EN</td>
 *          <td rowspan="1">10...8</td>
 *      </tr>
 * </table>
 *
 * @note
 * Do not use this function any longer. Use adcSelectADC(ADC_TypeDef *adc) instead.
 */
ADC_RETURN_CODE_t adcInitADC(ADC_TypeDef *adc)
{
    adcSelectADC(adc);

    return ADC_OK;
}

/**
 * @ingroup adc0
 * Turns the bus clock of the ADC off.
 *
 * @param  *adc    : Pointer to the ADC component
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
 *          <td>APB2ENR</td>
 *          <td rowspan="1">ADC1/2/3EN</td>
 *          <td rowspan="1">10...8</td>
 *      </tr>
 * </table>
 *
 * @note
 * Do not use this function any longer. Use adcDeselectADC(ADC_TypeDef *adc) instead.
 */
ADC_RETURN_CODE_t adcDeinitADC(ADC_TypeDef *adc)
{
    adcDeselectADC(adc);

    return ADC_OK;
}

/********************************************************************************************
 * Start of current implementation (not deprecated)                                         *
 *******************************************************************************************/

/**
 * @ingroup adc1
 * Turns the bus clock of the ADC on.
 *
 * @param  *adc    : Pointer to the ADC component
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
 *          <td>APB2ENR</td>
 *          <td rowspan="1">ADC1/2/3EN</td>
 *          <td rowspan="1">10...8</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcSelectADC(ADC_TypeDef *adc)
{
    if (ADC1 == adc)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    }
    return ADC_OK;
}

/**
 * @ingroup adc1
 * Turns the bus clock of the ADC off.
 *
 * @param  *adc    : Pointer to the ADC component
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
 *          <td>APB2ENR</td>
 *          <td rowspan="1">ADC1/2/3EN</td>
 *          <td rowspan="1">10...8</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcDeselectADC(ADC_TypeDef *adc)
{
    if (ADC1 == adc)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    }

    return ADC_OK;
}

/**
 * @ingroup adc1
 * Enables the ADC.
 *
 * @param  *adc    : Pointer to the ADC component
 * @return ADC_RETURN_CODE_t
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
 *          <td rowspan="1">ADON</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcEnableADC(ADC_TypeDef *adc)
{
    adc->CR2 |= ADC_CR2_ADON;
    return ADC_OK;
}

/**
 * @ingroup adc1
 * Disables the ADC.
 *
 * @param  *adc    : Pointer to the ADC component
 * @return ADC_RETURN_CODE_t
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
 *          <td rowspan="1">ADON</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcDisableADC(ADC_TypeDef *adc)
{
    adc->CR2 &= ~ADC_CR2_ADON_Msk;
    return ADC_OK;
}

/**
 * @ingroup adc1
 * Provides the information that shows whether the ADC is running or not.
 *
 * @param  *adc         : Pointer to the ADC
 * @return  bool
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
 *          <td rowspan="1">ADON</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
bool adcIsAdcRunning(ADC_TypeDef *adc)
{
    return (adc->CR2 & ADC_CR2_ADON);
}

/**
 * @ingroup adc1
 * Returns the conversion result from the ADC.
 *
 * @param  *adc    : Pointer to the ADC component
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
 *          <td>DR</td>
 *          <td>Data</TD>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1">Reads conversion result from DR</td>
 *      </tr>
 * </table>
 */
uint16_t adcGetConversionResult(ADC_TypeDef *adc)
{
    return (adc->DR);
}

/**
 * @ingroup adc1
 * Sets the resolution of the ADC.
 *
 * @param  *adc        : Pointer to the ADC component
 * @param   resolution : Might be 8, 10, or 12 bit
 * @return  ADC_RETURN_CODE
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
 *          <td rowspan="1">RES</td>
 *          <td rowspan="1">25...24</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcSetResolution(ADC_TypeDef *adc, ADC_RESOLUTION_t resolution)
{
    if (adcVerifyResolution(resolution) != true)
    {
        return ADC_INVALID_RESOLUTION;
    }

    adc->CR1 &= ~ADC_CR1_RES_Msk;
    switch (resolution)
    {
        case ADC_RES_6BIT:
            adc->CR1 |= ADC_CR1_RES_0 | ADC_CR1_RES_1;
            break;

        case ADC_RES_8BIT:
            adc->CR1 |= ADC_CR1_RES_1;
            break;

        case ADC_RES_10BIT:
            adc->CR1 |= ADC_CR1_RES_0;
            break;

        case ADC_RES_12BIT:
            adc->CR1 &= ~ADC_CR1_RES_Msk;
            break;
    }

    return ADC_OK;
}

/**
 * @ingroup adc1
 * Return the resolution of the ADC.
 *
 * @param  *adc : Pointer to the ADC component
 * @param  *res : Pointer to the variable which holds the resolution
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
 *          <td rowspan="1">RES</td>
 *          <td rowspan="1">25...24</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcGetResolution(ADC_TypeDef *adc, ADC_RESOLUTION_t *res)
{
    uint8_t resBits = 0;

    resBits = ((adc->CR1 & ADC_CR1_RES_Msk) >> ADC_CR1_RES_Pos);

    switch (resBits)
    {
        case ADC_RES_12BIT:
            *res = ADC_RES_12BIT;
            break;

        case ADC_RES_10BIT:
            *res = ADC_RES_10BIT;
            break;

        case ADC_RES_8BIT:
            *res = ADC_RES_8BIT;
            break;

        case ADC_RES_6BIT:
            *res = ADC_RES_6BIT;
            break;
    }

    return ADC_OK;
}

/**
 * @ingroup adc1
 * Enables the desired ADC interrupt.
 *
 * @param  *adc : Pointer to the ADC component
 * @param   irq : Name of the interrupt to be enabled
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
 *          <td rowspan="1">Different bits of the ADC CR1 register</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcEnableInterrupt(ADC_TypeDef *adc, ADC_IRQTYPES_t irq)
{
    if (false == adcVerifyInterruptType(irq))
    {
        return ADC_INVALID_IRQ_TYPE;
    }

    switch (irq)
    {
        case ADC_IRQ_OVERRUN:
            adc->CR1 |= ADC_CR1_OVRIE;
            break;

        case ADC_WATCHDOG_IRQ_EN:
            adc->CR1 |= ADC_CR1_AWDIE;
            break;

        case ADC_EOC_INJECTED_GRP:
            adc->CR1 |= ADC_CR1_JEOCIE;
            break;

        case ADC_EOC_REGULAR_GRP:
            adc->CR1 |= ADC_CR1_EOCIE;
            break;
    }

    return ADC_OK;
}

/**
 * @ingroup adc1
 * Disables the desired ADC interrupt.
 *
 * @param  *adc : Pointer to the ADC component
 * @param   irq : Name of the interrupt to be disabled
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
 *          <td rowspan="1">Different bits of the ADC CR1 register</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcDisableInterrupt(ADC_TypeDef *adc, ADC_IRQTYPES_t irq)
{
    if (false == adcVerifyInterruptType(irq))
    {
        return ADC_INVALID_IRQ_TYPE;
    }

    switch (irq)
    {
        case ADC_IRQ_OVERRUN:
            adc->CR1 &= ~ADC_CR1_OVRIE_Msk;
            break;

        case ADC_WATCHDOG_IRQ_EN:
            adc->CR1 &= ~ADC_CR1_AWDIE_Msk;
            break;

        case ADC_EOC_INJECTED_GRP:
            adc->CR1 &= ~ADC_CR1_JEOCIE_Msk;
            break;

        case ADC_EOC_REGULAR_GRP:
            adc->CR1 &= ~ADC_CR1_EOCIE_Msk;
            break;
    }

    return ADC_OK;
}

/**
 * @ingroup adc3
 *
 * @brief Enables the analog watchdog for the desired ADC channel.
 *
 * @param  *adc : Pointer to the ADC component
 * @param   chn : Number of the single channel which shall be monitored by the analog watchdog
 *
 * @note
 * This function implicitely sets the AWDSGL bit of CR1! It can be used only if the ADC is working in the single-channel
 * mode.
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
 *          <td rowspan="1">AWDCH</td>
 *          <td rowspan="1">4...0</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcEnableWatchdogForSingleChannel(ADC_TypeDef *adc, ADC_CHANNEL_t chn)
{
    if (false == adcVerifyChannelNum(chn))
    {
        return ADC_INVALID_CHANNEL_NUM;
    }

    adc->CR1 |= ADC_CR1_AWDSGL;             // Required to set the single-channel watchdog mode.

    switch (chn)
    {
        case ADC_CHN_0:
            adc->CR1 = ~ADC_CR1_AWDCH_Msk;
            break;

        case ADC_CHN_1:
            adc->CR1 = ADC_CR1_AWDCH_0;
            break;

        case ADC_CHN_2:
            adc->CR1 = ADC_CR1_AWDCH_1;
            break;

        case ADC_CHN_3:
            adc->CR1 = (ADC_CR1_AWDCH_0 | ADC_CR1_AWDCH_1);
            break;

        case ADC_CHN_4:
            adc->CR1 = ADC_CR1_AWDCH_2;
            break;

        case ADC_CHN_5:
            adc->CR1 = (ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_0);
            break;

        case ADC_CHN_6:
            adc->CR1 = (ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1);
            break;

        case ADC_CHN_7:
            adc->CR1 = (ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0);
            break;

        case ADC_CHN_8:
            adc->CR1 = ADC_CR1_AWDCH_3;
            break;

        case ADC_CHN_9:
            adc->CR1 = (ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_0);
            break;

        case ADC_CHN_10:
            adc->CR1 = (ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_1);
            break;

        case ADC_CHN_11:
            adc->CR1 = (ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0);
            break;

        case ADC_CHN_12:
            adc->CR1 = (ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2);
            break;

        case ADC_CHN_13:
            adc->CR1 = (ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_0);
            break;

        case ADC_CHN_14:
            adc->CR1 = (ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1);
            break;

        case ADC_CHN_15:
            adc->CR1 = (ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0);
            break;

        case ADC_CHN_16:
            adc->CR1 = ADC_CR1_AWDCH_4;
            break;

        case ADC_CHN_17:
            adc->CR1 = (ADC_CR1_AWDCH_4 | ADC_CR1_AWDCH_0);
            break;

        case ADC_CHN_18:
            adc->CR1 = (ADC_CR1_AWDCH_4 | ADC_CR1_AWDCH_1);
            break;
    }

    return ADC_OK;
}

/**
 * @ingroup adc3
 *
 * @brief Disables the analog watchdog for the desired ADC channel.
 *
 * @param  *adc : Pointer to the ADC component
 * @param   chn : Number of the single channel which shall be monitored by the analog watchdog
 *
 * @note
 * This function does NOT implicitely reset the AWDSGL bit of CR1!
 *
 * This function can only be used if the ADC is working in the
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
 *          <td rowspan="1">AWDCH</td>
 *          <td rowspan="1">4...0</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcDisableWatchdogForSingleChannel(ADC_TypeDef *adc, ADC_CHANNEL_t chn)
{
    if (false == adcVerifyChannelNum(chn))
    {
        return ADC_INVALID_CHANNEL_NUM;
    }

    if (adc->CR1 & ADC_CR1_AWDEN)
    {
        switch (chn)
        {
            case ADC_CHN_0:
                adc->CR1 &= ~ADC_CR1_AWDCH_Msk;
                break;

            case ADC_CHN_1:
                adc->CR1 &= ~ADC_CR1_AWDCH_0;
                break;

            case ADC_CHN_2:
                adc->CR1 &= ~ADC_CR1_AWDCH_1;
                break;

            case ADC_CHN_3:
                adc->CR1 &= ~(ADC_CR1_AWDCH_0 | ADC_CR1_AWDCH_1);
                break;

            case ADC_CHN_4:
                adc->CR1 &= ~ADC_CR1_AWDCH_2;
                break;

            case ADC_CHN_5:
                adc->CR1 &= ~(ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_0);
                break;

            case ADC_CHN_6:
                adc->CR1 &= ~(ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1);
                break;

            case ADC_CHN_7:
                adc->CR1 &= ~(ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0);
                break;

            case ADC_CHN_8:
                adc->CR1 &= ~ADC_CR1_AWDCH_3;
                break;

            case ADC_CHN_9:
                adc->CR1 &= ~(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_0);
                break;

            case ADC_CHN_10:
                adc->CR1 &= ~(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_1);
                break;

            case ADC_CHN_11:
                adc->CR1 &= ~(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0);
                break;

            case ADC_CHN_12:
                adc->CR1 &= ~(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2);
                break;

            case ADC_CHN_13:
                adc->CR1 &= ~(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_0);
                break;

            case ADC_CHN_14:
                adc->CR1 &= ~(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1);
                break;

            case ADC_CHN_15:
                adc->CR1 &= ~(ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0);
                break;

            case ADC_CHN_16:
                adc->CR1 &= ~ADC_CR1_AWDCH_4;
                break;

            case ADC_CHN_17:
                adc->CR1 &= ~(ADC_CR1_AWDCH_4 | ADC_CR1_AWDCH_0);
                break;

            case ADC_CHN_18:
                adc->CR1 &= ~(ADC_CR1_AWDCH_4 | ADC_CR1_AWDCH_1);
                break;
        }
    }

    return ADC_OK;
}

#if 0
/**
 * @ingroup adc3
 * Enables the analog watchdog for all ADC channels
 *
 * @param  *adc : Pointer to the ADC
 */
ADC_RETURN_CODE_t adcEnableWatchdogForAllChannels(ADC_TypeDef *adc, ADC_WATCHDOG_t type)
{
    if (false == adcVerifyAnalogWatchdogType(wdt))
    {
        return ADC_INVALID_WATCHDOG_TYPE;
    }

    if (adc->CR1 & ADC_CR1_)
        adc->CR1 &= ~ADC_CR1_AWDSGL_Msk;

    return ADC_OK;
}

/**
 * @ingroup adc3
 * Disables the analog watchdog for all ADC channels
 *
 * @param  *adc : Pointer to the ADC
 */
ADC_RETURN_CODE_t adcDisableWatchdogForAllChannels(ADC_TypeDef *adc)
{
    return ADC_OK;
}
#endif

/**
 * @ingroup adc3
 * Selects the type of the analog ADC watchdog (if any)
 *
 * @param  *adc : Pointer to the ADC
 * @param   wdt : The watchdog type
 */
ADC_RETURN_CODE_t adcSelectAnalogWatchdogType(ADC_TypeDef *adc, ADC_WATCHDOG_t wdt)
{
    if (false == adcVerifyAnalogWatchdogType(wdt))
    {
        return ADC_INVALID_WATCHDOG_TYPE;
    }

    switch (wdt)
    {
        case NO_ADC_WATCHDDOG:
            adc->CR1 &= ~(ADC_CR1_AWDEN_Msk | ADC_CR1_JAWDEN_Msk);
            break;

        case ALL_INJECTED_CHANNELS:
            adc->CR1 &= ~(ADC_CR1_AWDSGL_Msk | ADC_CR1_AWDEN_Msk);
            adc->CR1 |= ADC_CR1_JAWDEN;
            break;

        case ALL_REGULAR_CHANNELS:
            adc->CR1 &= ~(ADC_CR1_AWDSGL_Msk | ADC_CR1_JAWDEN_Msk);
            adc->CR1 |= ADC_CR1_AWDEN;
            break;

        case ALL_REGULAR_AND_INJECTED_CHANNELS:
            adc->CR1 &= ~ADC_CR1_AWDSGL_Msk;
            adc->CR1 |= (ADC_CR1_AWDEN | ADC_CR1_JAWDEN);
            break;

        case SINGLE_INJECTED_CHANNEL:
            adc->CR1 &= ~ADC_CR1_AWDEN_Msk;
            adc->CR1 |= (ADC_CR1_AWDSGL | ADC_CR1_JAWDEN);
            break;

        case SINGLE_REGULAR_CHANNEL:
            adc->CR1 &= ~ADC_CR1_JAWDEN_Msk;
            adc->CR1 |= (ADC_CR1_AWDSGL | ADC_CR1_AWDEN);
            break;

        case SINGLE_REGULAR_AND_INJECTED_CHANNEL:
            adc->CR1 |= (ADC_CR1_AWDSGL | ADC_CR1_JAWDEN | ADC_CR1_AWDEN);
            break;
    }

    return ADC_OK;
}

/**
 * @ingroup adc1
 *
 * @brief Enables the individual sampling time in cycles for each single ADC channel.
 *
 * @param  *adc    : Pointer to the ADC component
 * @param   chn    : Number of the single channel which shall be monitored by the analogue watchdog
 * @param   cycles : Duration of the sampling time in cycles
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
 *          <td>SMPR1/SMPR2</td>
 *          <td rowspan="1">Depends on the desired channel</td>
 *          <td rowspan="1">All bits</td>
 *      </tr>
 * </table>
 */
ADC_RETURN_CODE_t adcSetSampleCycles(ADC_TypeDef *adc, ADC_CHANNEL_t chn, ADC_SAMPLING_CYCLES_t cycles)
{
    if (false == adcVerifyChannelNum(chn))
    {
        return ADC_INVALID_CHANNEL_NUM;
    }

    if (false == adcVerifyNumSamplingCycles(cycles))
    {
        return ADC_INVALID_NUM_SAMPLE_CYCLES;
    }

    switch (chn)
    {
        case ADC_CHN_0:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP0_Pos;
            break;

        case ADC_CHN_1:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP1_Pos;
            break;

        case ADC_CHN_2:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP2_Pos;
            break;

        case ADC_CHN_3:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP3_Pos;
            break;

        case ADC_CHN_4:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP4_Pos;
            break;

        case ADC_CHN_5:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP5_Pos;
            break;

        case ADC_CHN_6:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP6_Pos;
            break;

        case ADC_CHN_7:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP7_Pos;
            break;

        case ADC_CHN_8:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP8_Pos;
            break;

        case ADC_CHN_9:
            adc->SMPR2 = cycles << ADC_SMPR2_SMP9_Pos;
            break;

        case ADC_CHN_10:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP10_Pos;
            break;

        case ADC_CHN_11:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP11_Pos;
            break;

        case ADC_CHN_12:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP12_Pos;
            break;

        case ADC_CHN_13:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP13_Pos;
            break;

        case ADC_CHN_14:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP14_Pos;
            break;

        case ADC_CHN_15:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP15_Pos;
            break;

        case ADC_CHN_16:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP16_Pos;
            break;

        case ADC_CHN_17:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP17_Pos;
            break;

        case ADC_CHN_18:
            adc->SMPR1 = cycles << ADC_SMPR1_SMP18_Pos;
            break;
    }

    return ADC_OK;
}

/**
 * @ingroup adc1
 *
 * @brief Defines a list of channels which shall be monitored within a sequence.
 *
 * A sequence can use up to 16 channels of the desired ADC. This function returns an error code if the size of the
 * list exceeds 16 entries!
 *
 * @param  *adc    : Pointer to the ADC component
 * @param  *chnList: A pointer to an array which provides up to 16 ADC input channels
 * @param   seqLen : Length of the sequence of ADC channels
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
 *          <td>SQR1/SQR2/SQR3</td>
 *          <td rowspan="1">Assigns all channels in the given order and sets the length of the sequence</td>
 *          <td rowspan="1">Depends on the registers</td>
 *      </tr>
 * </table>
 *
 * @note
 * The sequence starts always at position 0 in the sequence list.
 */
ADC_RETURN_CODE_t adcSetChannelSequence(ADC_TypeDef *adc, ADC_CHANNEL_t *chnList, size_t seqLen)
{
     uint8_t i = 0;

     if (seqLen > 16)
     {
         return ADC_INVALID_SEQUENCE_LENGTH;
     }

     adc->SQR1 |= ((seqLen - 1) << ADC_SQR1_L_Pos);

     for (i = 0; i < seqLen; i++)
     {
         switch (i)
         {
             case 0:                                            // This is ADC Input Channel #1
                 adc->SQR3 |= (chnList[i] << ADC_SQR3_SQ1_Pos);
                 break;

             case 1:                                            // This is ADC Input Channel #2
                 adc->SQR3 |= (chnList[i] << ADC_SQR3_SQ2_Pos);
                 break;

             case 2:                                            // This is ADC Input Channel #3
                 adc->SQR3 |= (chnList[i] << ADC_SQR3_SQ3_Pos);
                 break;

             case 3:                                            // This is ADC Input Channel #4
                 adc->SQR3 |= (chnList[i] << ADC_SQR3_SQ4_Pos);
                 break;

             case 4:                                            // This is ADC Input Channel #5
                 adc->SQR3 |= (chnList[i] << ADC_SQR3_SQ5_Pos);
                 break;

             case 5:                                            // This is ADC Input Channel #6
                 adc->SQR3 |= (chnList[i] << ADC_SQR3_SQ6_Pos);
                 break;

             case 6:                                            // This is ADC Input Channel #7
                 adc->SQR2 |= (chnList[i] << ADC_SQR2_SQ7_Pos);
                 break;

             case 7:                                            // This is ADC Input Channel #8
                 adc->SQR2 |= (chnList[i] << ADC_SQR2_SQ8_Pos);
                 break;

             case 8:                                            // This is ADC Input Channel #9
                 adc->SQR2 |= (chnList[i] << ADC_SQR2_SQ9_Pos);
                 break;

             case 9:                                            // This is ADC Input Channel #10
                 adc->SQR2 |= (chnList[i] << ADC_SQR2_SQ10_Pos);
                 break;

             case 10:                                            // This is ADC Input Channel #11
                 adc->SQR2 |= (chnList[i] << ADC_SQR2_SQ11_Pos);
                 break;

             case 11:                                            // This is ADC Input Channel #12
                 adc->SQR2 |= (chnList[i] << ADC_SQR2_SQ12_Pos);
                 break;

             case 12:                                            // This is ADC Input Channel #13
                 adc->SQR1 |= (chnList[i] << ADC_SQR1_SQ13_Pos);
                 break;

             case 13:                                            // This is ADC Input Channel #14
                 adc->SQR1 |= (chnList[i] << ADC_SQR1_SQ14_Pos);
                 break;

             case 14:                                            // This is ADC Input Channel #15
                 adc->SQR1 |= (chnList[i] << ADC_SQR1_SQ15_Pos);
                 break;

             case 15:                                            // This is ADC Input Channel #16
                 adc->SQR1 |= (chnList[i] << ADC_SQR1_SQ16_Pos);
                 break;
         }
     }

     return ADC_OK;
}

/**
 * @ingroup adc1
 *
 * @brief Starts the ADC conversion (sequence).
 *
 * The ADC must be enabled with adcEnableADC(ADC_TypeDef *adc) **before** the conversion can be started. Otherwise,
 * the adcStartConversion() function has no effect.
 *
 * @param  *adc    : Pointer to the ADC component
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
 *          <td rowspan="1">SWSTART</td>
 *          <td rowspan="1">30</td>
 *      </tr>
 * </table>
 *
 * @note
 * The sequence starts always at position 0 in the sequence list.
 */
void adcStartConversion(ADC_TypeDef *adc)
{
    adc->CR2 |= ADC_CR2_SWSTART;
}

/**
 * @ingroup adc1
 *
 * @brief Returns the state of an ADC conversion.
 *
 * @param  *adc    : Pointer to the ADC component
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
 *          <td>SR</td>
 *          <td rowspan="1">EOC</td>
 *          <td rowspan="1">1</td>
 *      </tr>
 * </table>
 *
 * @note
 * The sequence starts always at position 0 in the sequence list.
 */
bool adcIsConversionFinished(ADC_TypeDef *adc)
{
    return (adc->SR & ADC_SR_EOC);
}

// EOF
