/**
 * @defgroup tim  Timer Functions (mcalTimer.h/.c)
 * @defgroup tim0 Deprecated Timer Functions
 * @ingroup  tim
 * @defgroup tim1 Common Timer Functions
 * @ingroup  tim
 * @defgroup tim2 Capture/Compare Functions
 * @ingroup  tim
 * @defgroup tim3 Common Capture/Compare Functions
 * @ingroup  tim2
 * @defgroup tim4 Input Capture Functions (IC functions)
 * @ingroup  tim2
 * @defgroup tim5 Output Compare Functions (OC functions)
 * @ingroup  tim2
 * @defgroup tim6 Interrupt-related Functions
 * @ingroup  tim
 * @defgroup tim7 DMA-related functions
 * @ingroup  tim
 * @defgroup tim8 Event Generation
 * @ingroup  tim
 * @defgroup tim9 Master/Slave Control
 * @ingroup  tim
 * @defgroup tim10 Advanced Control Timer Functions
 * @ingroup  tim
 * @defgroup tim11 Timer Enumerations and Definitions
 * @ingroup  tim
 *
 *
 * @file     mcalTimer/mcalTimer.c
 * @brief    mcalTimer.c is part of the MCAL library for STM32F4xx.
 * @author   Ralf Jesse (embedded@ralf-jesse.de)
 * @date     December 21, 2020
 * @date     $Date (last modified)
 *
 * @version     0.3
 *
 * <b>Description</b><br>
 * The STM32F4xx micro controllers support up to three different timer classes:
 * *Basic timers*, *General-purpose timers*, and *Advanced-control timers*. This
 * is a collection of functions which supports the usage of all timers.
 *
 * <b>Some functions (e.g. handling of internal and external triggers) are not yet supported.</b>
 *
 * @copyright   GNU Public License Version 3 (GPLv3)<br>
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
#include <mcalTimer/mcalTimer.h>


//--------------------------------------------------------------------------------

/**
 * @ingroup tim1
 * This function selects a timer and turns the corresponding bus clock on.
 *
 * @param   *tim : Pointer to the timer
 *
 * @note
 * Use this function instead of the deprecated *timerBusClkOn(TIM_TypeDef *tim)* function.
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
 *          <td>RCC</td>
 *          <td rowspan="1">APB1ENR/APB2ENR</td>
 *          <td rowspan="1">depend on the desired timer</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerSelectTimer(TIM_TypeDef *tim)
{
    if (TIM1 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if (TIM2 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }
    else if (TIM3 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    }
    else if (TIM4 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    else if (TIM5 == tim)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }
    else if (TIM9 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    }
    else if (TIM10 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    }
    else if (TIM11 == tim)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    }
    tim->CR1 = 0;  // Reset all bits of TIMx_CR1 to reset state
    tim->CR2 = 0;  // Reset all bits of TIMx_CR1 to reset state
}

/**
 * @ingroup tim1
 * @brief Deselects the desired timer
 *
 * This function deselects a timer and turns the corresponding bus clock off. It also clears the corresponding
 *
 * @param   *tim : Pointer to the timer
 *
 * @note
 * Use this function instead of the deprecated *timerBusClkOff(TIM_TypeDef *tim)* function.
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
 *          <td>RCC</td>
 *          <td rowspan="1">APB1ENR/APB2ENR</td>
 *          <td rowspan="1">depend on the desired timer</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerDeselectTimer(TIM_TypeDef *tim)
{
    if (TIM1 == tim)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN_Msk;
    }
    else if (TIM2 == tim)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN_Msk;
    }
    else if (TIM3 == tim)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN_Msk;
    }
    else if (TIM4 == tim)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN_Msk;
    }
    else if (TIM5 == tim)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN_Msk;
    }
    else if (TIM9 == tim)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN_Msk;
    }
    else if (TIM10 == tim)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN_Msk;
    }
    else if (TIM11 == tim)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN_Msk;
    }
    tim->CR1 = 0;  // Reset all bits of TIMx_CR1
    tim->CR2 = 0;  // Reset all bits of TIMx_CR2
}

/**
 * @ingroup tim1
 * Enables the timer preload mode.
 *
 * @param   *tim : Pointer to the timer
 * @return  TIMER_RETURN_CODE_t
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
 *          <td rowspan="1">ARPE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Sets the ARPE bit to '1'</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableAutoReloadPreload(TIM_TypeDef *tim)
{
    tim->CR1 |= TIM_CR1_ARPE;

    return TIMER_OK;
}

/**
 * @ingroup tim1
 * Disables the timer preload mode.
 *
 * @param   *tim : Pointer to the timer
 * @return  TIMER_RETURN_CODE_t
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
 *          <td rowspan="1">ARPE</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Resets the ARPE bit to '0'</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableAutoReloadPreload(TIM_TypeDef *tim)
{
    tim->CR1 &= ~TIM_CR1_ARPE_Msk;

    return TIMER_OK;
}

/**
 * @ingroup tim1
 * Sets the prescaler register (PSC) of the timer.
 *
 * @param   *tim : Pointer to the timer tim
 * @param    psc : New prescaler value. Range: 1 ... 65536
 *
 * @note
 * The hardware adds 1 to the given prescaler. When using this function
 * you can now pass the desired prescaler value instead of (desiredValue - 1).
 * This function internally subtratcs 1 from *psc*.
 *
 * Example: The prescaler shall be set to 10000.
 * Old    : Use 'timerSetPrescaler(TIMx, (10000 - 1));
 * **New**: Use 'timerSetPrescaler(TIMx, 10000);**
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
 *          <td>PSC</td>
 *          <td rowspan="1">PSC</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerSetPrescaler(TIM_TypeDef *tim, uint32_t psc)
{
    psc -= 1;
    if (psc > TIMER_MAX_PRESCALER)
    {
        psc = TIMER_MAX_PRESCALER;
    }
    tim->PSC = psc;
}

/**
 * @ingroup tim1
 * Returns the currently set prescaler value.
 *
 * @param   *tim : Pointer to the timer
 * @return  psc : The content of the PSC register
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
 *          <td>PSC</td>
 *          <td rowspan="1">PSC</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
uint32_t timerGetPrescaler(TIM_TypeDef *tim)
{
    return tim->PSC & 0xFFFFUL;
}

/**
 * @ingroup tim1
 * Sets the auto-reload register (ARR) of the timer.
 *
 * @param   *tim    : Pointer to the timer tim
 * @param    reload : New reload value for the timer
 *
 * @note Consider that only TIM2 and TIM5 are 32 bit timers.
 *       If the 'reload' value is too high for all the other
 *       timers the reload value is set to 0 and the timer
 *       will not start (ARR MUST be different from 0!).
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
 *          <td>ARR</td>
 *          <td rowspan="1">ARR</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerSetAutoReloadValue(TIM_TypeDef *tim, uint32_t reload)
{
    reload -= 1;

    // 32-bit reload values are only valid for TIM2 and TIM5.
    if ((TIM2 == tim) || (TIM5 == tim))
    {
        tim->ARR = reload;
    }
    else
    {
        // For all other timers: Check that reload does not exceed UIN16_MAX.
        if (reload > UINT16_MAX)
        {
            reload = 0;   // reload value too high --> set to 0 -->
                          // timer will not start!
        }
        tim->ARR = reload;
    }
}

/**
 * @ingroup tim1
 * Returns the current value of the Auto-reload Register (ARR).
 *
 * @param   *tim : Pointer to the timer
 * @return  arr : The content of the ARR
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
 *          <td>ARR</td>
 *          <td rowspan="1">ARR</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
uint32_t timerGetAutoReloadValue(TIM_TypeDef *tim)
{
    if ((TIM2 == tim) || (TIM5 == tim))
    {
        return tim->ARR;
    }

    return ((tim->ARR) & 0xFFFFUL);
}

/**
 * @ingroup tim1
 * Sets the starting value of the counter.
 *
 * @param  *tim : Pointer to the timer
 * @param   cnt : Start value of the counter
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
 *          <td>CNT</td>
 *          <td rowspan="1">CNT</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerSetCounter(TIM_TypeDef *tim, uint32_t cnt)
{
    if ((TIM2 == tim) || (TIM5 == tim))
    {
        tim->CNT = cnt;
    }
    else
    {
        tim->CNT = cnt & 0xFFFFUL;
    }
}

/**
 * @ingroup tim1
 * Returns the current value of the CNT register
 *
 * @param   *tim : Pointer to the timer
 * @return  cnt : The current value of the CNT register
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
 *          <td>CNT</td>
 *          <td rowspan="1">CNT</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
uint32_t timerGetCounter(TIM_TypeDef *tim)
{
    if ((TIM2 == tim) || (TIM5 == tim))
    {
        return tim->CNT;
    }

    return ((tim->CNT) & 0xFFFFUL);
}

/**
 * @ingroup tim1
 * Resets the CNT register
 *
 * @param   *tim  : Pointer to the timer
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
 *          <td>CNT</td>
 *          <td rowspan="1">CNT</td>
 *          <td rowspan="1">15...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerResetCounter(TIM_TypeDef *tim)
{
    tim->CNT = 0;
}

/**
 * @ingroup tim1
 * @brief   Starts the desired timer.
 * @details All timer settings must be finished before starting the timer.
 *
 * @param   *tim : Pointer to the timer tim
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
 *          <td rowspan="1">CEN</td>
 *          <td rowspan="1">0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerStartTimer(TIM_TypeDef *tim)
{
    tim->CR1 |= TIM_CR1_CEN;
}

/**
 * @ingroup tim1
 * @brief   Stops the desired timer.
 *
 * @param   *tim : Pointer to the timer tim
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
 *          <td rowspan="1">CEN</td>
 *          <td rowspan="1">0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
void timerStopTimer(TIM_TypeDef *tim)
{
    tim->CR1 &= ~TIM_CR1_CEN_Msk;
}

/**
 * @ingroup tim1
 * Enables the one-shot (one-pulse) mode of the timer.
 *
 * @param   *tim ; Pointer to the timer
 * @return  TIMER_RETURN_CODE_t
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
 *          <td rowspan="1">OPM</td>
 *          <td rowspan="1">3</td>
 *          <td rowspan="1">TIM10/11/13/14 don't support OPM</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableOneShot(TIM_TypeDef *tim)
{
    if (TIM10 != tim && TIM11 != tim )
    {
        tim->CR1 |= TIM_CR1_OPM;
        return TIMER_OK;
    }
    return TIMER_ONE_SHOT_NOT_SUPPORTED;
}

/**
 * @ingroup tim1
 * Disables the one-shot mode of the timer
 *
 * @param   *tim : Pointer to the timer
 * @return  TIMER_RETURN_CODE_t
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
 *          <td rowspan="1">OPM</td>
 *          <td rowspan="1">3</td>
 *          <td rowspan="1">TIM10/11/13/14 don't support OPM</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableOneShot(TIM_TypeDef *tim)
{
    if (TIM10 != tim && TIM11 != tim )
    {
        tim->CR1 &= ~TIM_CR1_OPM_Msk;
        return TIMER_OK;
    }
    return TIMER_ONE_SHOT_NOT_SUPPORTED;
}

/**
 * @ingroup tim1
 * Returns the current setting of the OPM bit.
 *
 * @param   *tim : Pointer to the timer
 * @param   *opm : Pointer to the variable that shall show the state of the OPM bit
 * @return  TIMER_RETURN_CODE_t
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
 *          <td rowspan="1">OPM</td>
 *          <td rowspan="1">3</td>
 *          <td rowspan="1">TIM10/11/13/14 don't support OPM</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerGetOneShotMode(TIM_TypeDef *tim, bool *opm)
{
    if (TIM10 != tim && TIM11 != tim)
    {
        *opm = tim->CR1 & TIM_CR1_OPM;
        return TIMER_OK;
    }
    return TIMER_ONE_SHOT_NOT_SUPPORTED;
}

/**
 * @ingroup tim1
 * Sets the counting direction of the desired timer.
 *
 * @param  *tim : Pointer to the desired timer
 * @param   dir : Might be COUNTUP or COUNTDOWN
 *
 * @note
 * Consider that only TIM1/TIM2/TIM5/TIM8 are able to count in both directions. For any other timer the counting direction is
 * ignored. Consider that this function has no effect in the center-aligned mode (CMS)!
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
 *          <td rowspan="1">DIR</td>
 *          <td rowspan="1">4</td>
 *          <td rowspan="1">Only TIM1/TIM2/TIM5/TIM8 support the selection of the counting direction.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetCountingDir(TIM_TypeDef *tim, TIMER_COUNT_DIR_t dir)
{
    if ((TIM1 == tim) || (TIM2 == tim) || (TIM5 == tim))
    {
        switch (dir)
        {
            case COUNTUP:
                tim->CR1 &= ~TIM_CR1_DIR_Msk;
                break;

            case COUNTDOWN:
                tim->CR1 |= TIM_CR1_DIR;
                break;

            default:
                tim->CR1 &= ~TIM_CR1_DIR_Msk;
                break;
        }
    }
    return TIMER_OK;
}







/* EOF */
