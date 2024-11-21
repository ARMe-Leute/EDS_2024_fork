/**
 * @file    mcalTimer/events.c
 * @author  Ralf Jesse
 * @date    MAR 15, 2021
 *
 * This file provides functions related to event generation.
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @ingroup tim8
 * Enables generation of a timer update event
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
 *          <td rowspan="1">UDIS</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Supported by all timers</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerEnableCaptureCompareUpdateEvent(TIM_TypeDef *tim)
{
    tim->CR1 &= ~TIM_CR1_UDIS_Msk;      // UDIS must be set to '0' to enable the event generation
    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Disables generation of a timer update event
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
 *          <td rowspan="1">UDIS</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Supported by all timers</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerDisableCaptureCompareUpdateEvent(TIM_TypeDef *tim)
{
    tim->CR1 |= TIM_CR1_UDIS;
    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Generates a timer update event. This function is available for all timers.
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
 *          <td>EGR</td>
 *          <td rowspan="1">UG</td>
 *          <td rowspan="1">0</td>
 *          <td rowspan="1">Supported by all timers</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerGenerateCaptureCompareUpdateEvent(TIM_TypeDef *tim)
{
    tim->EGR |= TIM_EGR_UG;

    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Enables generation of a capture/compare event on the desired timer channel. The timer <--> channel assignment is internally
 * verified.
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : The desired timer channel
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
 *          <td>EGR</td>
 *          <td rowspan="1">CC1G/CC2G/CC3G/CC4G</td>
 *          <td rowspan="1">4...1</td>
 *          <td rowspan="1">Depending on the timer not all channels are supported</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableCaptureCompareEventGeneration(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
                tim->EGR |= TIM_EGR_CC1G;
            break;

        case TIMIO_CH2:
                tim->EGR |= TIM_EGR_CC2G;
            break;

        case TIMIO_CH3:
                tim->EGR |= TIM_EGR_CC3G;
            break;

        case TIMIO_CH4:
                tim->EGR |= TIM_EGR_CC4G;
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Disables generation of a capture/compare event on the desired timer channel. The timer <--> channel assignment is internally
 * verified.
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : The desired timer channel
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
 *          <td>EGR</td>
 *          <td rowspan="1">CC1G/CC2G/CC3G/CC4G</td>
 *          <td rowspan="1">4...1</td>
 *          <td rowspan="1">Depending on the timer not all channels are supported</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableCaptureCompareEventGeneration(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
                tim->EGR &= ~TIM_EGR_CC1G_Msk;
            break;

        case TIMIO_CH2:
                tim->EGR &= ~TIM_EGR_CC2G_Msk;
            break;

        case TIMIO_CH3:
                tim->EGR &= ~TIM_EGR_CC3G_Msk;
            break;

        case TIMIO_CH4:
                tim->EGR &= ~TIM_EGR_CC4G_Msk;
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Does not really trigger an event, but sets the corresponding interrupt flag that has to be handled separately.
 *
 * @param   *tim     : Pointer to the timer
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
 *          <td>EGR</td>
 *          <td rowspan="1">TG</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">Only supported by TIM1/TIM2/TIM3/TIM4/TIM5/TIM8</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableCaptureCompareTriggerGeneration(TIM_TypeDef *tim)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim))
    {
        return TIMER_INVALID_TIMER;
    }

    tim->EGR |= TIM_EGR_TG;

    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Does not really trigger an event, but sets the corresponding interrupt flag that has to be handled separately.
 *
 * @param   *tim     : Pointer to the timer
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
 *          <td>EGR</td>
 *          <td rowspan="1">TG</td>
 *          <td rowspan="1">6</td>
 *          <td rowspan="1">Only supported by TIM1/TIM2/TIM3/TIM4/TIM5/TIM8</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableCaptureCompareTriggerGeneration(TIM_TypeDef *tim)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim))
    {
        return TIMER_INVALID_TIMER;
    }

    tim->EGR &= ~TIM_EGR_TG_Msk;

    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Allows to update CCxE, CCxNE and OCxM bits if CCPC in control register 2 (CR2) is set.
 *
 * @param   *tim     : Pointer to the timer
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
 *          <td>EGR</td>
 *          <td rowspan="1">COMG</td>
 *          <td rowspan="1">5</td>
 *          <td rowspan="1">Only supported by TIM1 and TIM8</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableCaptureCompareUpdateCtrl(TIM_TypeDef *tim)
{
    if (TIM1 != tim)
    {
        return TIMER_INVALID_TIMER;
    }

    tim->EGR |= TIM_EGR_COMG;

    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Disables update of CCxE, CCxNE and OCxM bits even if the CCPC in control register 2 (CR2) is set.
 *
 * @param   *tim     : Pointer to the timer
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
 *          <td>EGR</td>
 *          <td rowspan="1">COMG</td>
 *          <td rowspan="1">5</td>
 *          <td rowspan="1">Only supported by TIM1 and TIM8</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableCaptureCompareUpdateCtrl(TIM_TypeDef *tim)
{
    if (TIM1 != tim)
    {
        return TIMER_INVALID_TIMER;
    }

    tim->EGR &= ~TIM_EGR_COMG_Msk;

    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Disables update of CCxE, CCxNE and OCxM bits even if the CCPC in control register 2 (CR2) is set.
 *
 * @param   *tim     : Pointer to the timer
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
 *          <td>EGR</td>
 *          <td rowspan="1">BG</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Only supported by TIM1 and TIM8</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerEnableCaptureCompareBreakGeneration(TIM_TypeDef *tim)
{
    if (TIM1 != tim)
    {
        return TIMER_INVALID_TIMER;
    }

    tim->EGR |= TIM_EGR_BG;

    return TIMER_OK;
}

/**
 * @ingroup tim8
 * Disables update of CCxE, CCxNE and OCxM bits even if the CCPC in control register 2 (CR2) is set.
 *
 * @param   *tim     : Pointer to the timer
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
 *          <td>EGR</td>
 *          <td rowspan="1">BG</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1">Only supported by TIM1 and TIM8</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerDisableCaptureCompareBreakGeneration(TIM_TypeDef *tim)
{
    if (TIM1 != tim)
    {
        return TIMER_INVALID_TIMER;
    }

    tim->EGR &= ~TIM_EGR_BG_Msk;

    return TIMER_OK;
}


/* EOF */
