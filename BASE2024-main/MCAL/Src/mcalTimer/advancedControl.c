/**
 * @file    mcalTimer/advancedControl.c
 * @author  Ralf Jesse
 * @date    MAR 15, 2021
 *
 * This file provides some advanced functions which are only available for th Advanced Control Timer TIM1 and TIM8.
 * TIM8 is deleted for use of STM32F401
 */

#include <mcalTimer/mcalTimer.h>


/**
 * @ingroup tim10
 * Sets the repetition counter of TIM1/TIM8.
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Number of the channel
 * @param    value   : Number of repetitions before the compare
 *                     registers are updated
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
 *          <td>RCR</td>
 *          <td rowspan="1">REP</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Only supported by TIM1 and TIM8.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetRepetitionCounter(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, uint8_t value)
{
if (TIM1 != tim)
    {
        return TIMER_REPETITION_NOT_SUPPORTED;
    }

    tim->RCR = value;
    return TIMER_OK;
}

/**
 * @ingroup tim10
 * Sets the DTG bits (DTG = dead time generator) of TIM1 and TIM8
 *
 * @param  *tim      : Pointer to timer tim
 * @param   deadTime : Deadtime value. Must be in the range of [0 ... 255].
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
 *          <td>BDTR</td>
 *          <td rowspan="1">DTG</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Break and dead time only supported by TIM1 and TIM8. Setting of BDTR not yet supported.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSetBreakAndDeadTime(TIM_TypeDef *tim, uint8_t deadTime)
{
    if (TIM1 == tim)
    {
        tim->BDTR |= deadTime;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim10
 * Clears the BDTR register of TIM1 and/or TIM8
 *
 * @param  *tim : Pointer to timer tim
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
 *          <td>BDTR</td>
 *          <td rowspan="1">DTG</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Break and dead time only supported by TIM1 and TIM8. Setting of BDTR not yet supported.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerBdtrClearRegister(TIM_TypeDef *tim)
{
    if (TIM1 == tim)
    {
        tim->BDTR = 0;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim10
 * Modifies the *Break and dead-time register (BDTR)* and activates/deactivates the main output
 *
 * @param   *tim     : Pointer to timer
 * @param    moeMode : Activates or deactivates the main output
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
 *          <td>BDTR</td>
 *          <td rowspan="1">MOE</td>
 *          <td rowspan="1">15</td>
 *          <td rowspan="1">Break and dead time only supported by TIM1 and TIM8. Assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerBdtrEnableMainOutput(TIM_TypeDef *tim, TIMER_BDTR_MOE_t moeMode)
{
    if (TIM1 != tim)
    {
        return TIMER_BDTR_NOT_SUPPORTED;
    }

    if (MOE_OFF == moeMode)
    {
        tim->BDTR &= ~(TIM_BDTR_MOE);
    }
    else
    {
        tim->BDTR |= TIM_BDTR_MOE;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim10
 * Activates/deactivates the Break input
 *
 * @param  *tim     : Pointer to timer tim
 * @param   bkeMode : Activation of the Break input
 * @return TIMER_RETURN_CODE_t
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
 *          <td>BDTR</td>
 *          <td rowspan="1">BKE</td>
 *          <td rowspan="1">12</td>
 *          <td rowspan="1">Break and dead time only supported by TIM1 and TIM8. Assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerBdtrEnableBreakInput(TIM_TypeDef *tim, TIMER_BDTR_BKE_t bkeMode)
{
    if (TIM1 != tim)
    {
        return TIMER_BDTR_NOT_SUPPORTED;
    }

    if (BRK_ENABLE == bkeMode)
    {
        tim->BDTR |= TIM_BDTR_BKE;
    }
    else
    {
        tim->BDTR &= ~(TIM_BDTR_BKE);
    }
    return TIMER_OK;
}


/* EOF */
