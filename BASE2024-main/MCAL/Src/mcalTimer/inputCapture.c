/**
 * @file    mcalTimer/inputCapture.c
 * @author  Ralf Jesse
 * @date    MAR 17, 2021
 *
 * This file provides functions related to Input Capture.
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @ingroup tim4
 * Sets the prescaler for input trigger events
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Number of the channel
 * @param    psc     : Prescaler
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
 *          <td>CCMR1/CCMR2</td>
 *          <td rowspan="1">IC1PSC/IC2PSC/IC3PSC/IC4PSC</td>
 *          <td rowspan="1">11...10/3...2</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetInputCapturePrescaler(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, TIMER_IC_PRESCALER_t psc)
{
    if (timerVerifyChannelNum(channel) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (psc)
    {
        case CAPTURE_ALL:
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1PSC_Msk;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2PSC_Msk;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3PSC_Msk;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4PSC_Msk;
            }
            break;

        case CAPTURE_EVERY_2ND:
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 |= TIM_CCMR1_IC1PSC_0;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 |= TIM_CCMR1_IC2PSC_0;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 |= TIM_CCMR2_IC3PSC_0;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 |= TIM_CCMR2_IC4PSC_0;
            }
            break;

        case CAPTURE_EVERY_4TH:
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 |= TIM_CCMR1_IC1PSC_1;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 |= TIM_CCMR1_IC2PSC_1;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 |= TIM_CCMR2_IC3PSC_1;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 |= TIM_CCMR2_IC4PSC_1;
            }
            break;

        case CAPTURE_EVERY_8TH:
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 |= (TIM_CCMR1_IC1PSC_1 | TIM_CCMR1_IC1PSC_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 |= (TIM_CCMR1_IC2PSC_1 | TIM_CCMR1_IC2PSC_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 |= (TIM_CCMR2_IC3PSC_1 | TIM_CCMR2_IC3PSC_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 |= (TIM_CCMR2_IC4PSC_1 | TIM_CCMR2_IC4PSC_0);
            }
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim4
 * Gets the prescaler for input trigger events
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Number of the channel
 * @param   *psc     : Pointer of the variable where the current prescaler value shall be stored
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
 *          <td>CCMR1/CCMR2</td>
 *          <td rowspan="1">IC1PSC/IC2PSC/IC3PSC/IC4PSC</td>
 *          <td rowspan="1">11...10/3...2</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerGetInputCapturePrescaler(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, TIMER_IC_PRESCALER_t *psc)
{
    if (timerVerifyChannelNum(channel) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == channel)
    {
        *psc = (tim->CCMR1 >> TIM_CCMR1_IC1PSC_Pos);
    }
    else if (TIMIO_CH2 == channel)
    {
        *psc = (tim->CCMR1 >> TIM_CCMR1_IC2PSC_Pos);
    }
    else if (TIMIO_CH3 == channel)
    {
        *psc = (tim->CCMR2 >> TIM_CCMR2_IC3PSC_Pos);
    }
    else if (TIMIO_CH4 == channel)
    {
        *psc = (tim->CCMR2 >> TIM_CCMR2_IC4PSC_Pos);
    }

    return TIMER_OK;
}

/**
 * @ingroup tim4
 * Sets the timer channel to TIMIO_INPUT and selects the operational mode.
 *
 * @param   *tim     : Pointer to the timer
 * @param    chn     : Number of the channel
 * @param    mapping : Mapping of the input-capture channel
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
 *          <td>CCMR1/CCMR2</td>
 *          <td rowspan="1">CC1S...CC4S</td>
 *          <td rowspan="1">9...8/1...0</td>
 *          <td rowspan="1">Only TIM1/tim3/TIM5/TIM8 support four channels! The tim-chn assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetInputCaptureMode(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_CHANNEL_MAPPING_t mapping)
{
    if (timerVerifyChannelNum(chn) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    //    MAP_TO_OUTPUT               = 0,
    //    MAP_CHN_TO_SAME_TIMER,
    //    MAP_CHN_TO_OPPOSITE_TIMER,
    //    MAP_CHN_TO_TRC


    switch (chn)
    {
        case TIMIO_CH1:
            tim->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;  // Reset previous setting
            switch (mapping)
            {
                // CH1: Mapped to output
                case MAP_TO_OUTPUT:
                    tim->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;
                    break;

                // CH1: Mapped to IC1
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC1S_0;
                    break;

                // CH1: Mapped to IC2
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC1S_1;
                    break;

                // CH1: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR1 |= TIM_CCMR1_CC1S_1 | TIM_CCMR1_CC1S_0;
                    break;
            }

        case TIMIO_CH2:
            tim->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;  // Reset previous setting
            switch (mapping)
            {
                // CH2: Mapped to putput
                case MAP_TO_OUTPUT:
                    tim->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;
                    break;

                // CH2: Mapped to IC2
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC2S_0;
                    break;

                // CH2: Mapped to IC1
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC2S_1;
                    break;

                // CH2: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR1 |= TIM_CCMR1_CC2S_1 | TIM_CCMR1_CC2S_0;
                    break;
            }
            break;

        case TIMIO_CH3:
            tim->CCMR2 &= ~TIM_CCMR2_CC3S_Msk;  // Reset previous setting
            switch (mapping)
            {
                // CH3: Mapped to output
                case MAP_TO_OUTPUT:
                    tim->CCMR2 &= ~TIM_CCMR2_CC3S_Msk;
                    break;

                // CH3: Mapped to IC3
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC3S_0;
                    break;

                // CH3: Mapped to IC4
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC3S_1;
                    break;

                // CH3: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR2 |= TIM_CCMR2_CC3S_1 | TIM_CCMR2_CC3S_0;
                    break;
            }
            break;

        case TIMIO_CH4:
            tim->CCMR2 &= ~TIM_CCMR2_CC4S_Msk;  // Reset previous setting
            switch (mapping)
            {
                // CH4: Mapped to output
                case MAP_TO_OUTPUT:
                    tim->CCMR2 &= ~TIM_CCMR2_CC4S_Msk;
                    break;

                // CH4: Mapped to IC3
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC4S_0;
                    break;

                // CH4: Mapped to IC4
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC4S_1;
                    break;

                // CH4: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR2 |= TIM_CCMR2_CC4S_1 | TIM_CCMR2_CC4S_0;
                    break;
            }
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim4
 * When using a timer channel as an input this function sets the
 * ratio between the timer clock frequency and the sampling frequency
 * of the sampling clock of the input filters.
 *
 * @param   *tim : Pointer to the timer
 * @param    chn : Number of the input capture channel
 * @param    ckd : Ratio can be to set 1x/2x/4x of the timer clock
 * returns   TIMER_RETURN_CODE_t
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
 *          <td rowspan="1">CKD</td>
 *          <td rowspan="1">9...8</td>
 *          <td rowspan="1">Timer-Channel assignment is internally checked.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetInputCaptureSamplingClkRatio(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_RATIO_t ckd)
{
    // These timers provide one capture/compare channel
    if ((TIM10 == tim) || (TIM11 == tim))
    {
        if (chn > TIMIO_CH1)
        {
            return TIMER_INVALID_CHANNEL;
        }
    }

    // These timers provide two capture/compare channels
    if (TIM9 == tim)
    {
        if (chn > TIMIO_CH2)
        {
            return TIMER_INVALID_CHANNEL;
        }
    }

    tim->CR1 &= ~TIM_CR1_CKD_Msk;

    switch (ckd)
    {
        case CKD_1:
            tim->CR1 &= ~TIM_CR1_CKD_Msk;
            break;

        case CKD_2:
            tim->CR1 |= TIM_CR1_CKD_0;
            break;

        case CKD_4:
            tim->CR1 |= TIM_CR1_CKD_1;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim4
 * Returns the current value of the ratio between sampling frequency of the
 * timer channel and the clock frequency of the timer.
 *
 * @param   *tim : Pointer to the timer
 * @param    chn : Numer of the input capture channel
 * @param   *ckd : Pointer to the variable which holds the CKD value
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
 *          <td rowspan="1">n.n.</td>
 *          <td rowspan="1">tbd</td>
 *          <td rowspan="1">Not yet supported</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerGetInputCaptureSamplingClkRatio(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_RATIO_t *ckd)
{
    // These timers provide one capture/compare channel
    if ((TIM10 == tim) || (TIM11 == tim))
    {
        if (chn > TIMIO_CH1)
        {
            return TIMER_INVALID_CHANNEL;
        }
    }

    // These timers provide two capture/compare channels
    if (TIM9 == tim)
    {
        if (chn > TIMIO_CH2)
        {
            return TIMER_INVALID_CHANNEL;
        }
    }

    *ckd = ((tim->CR1 & TIM_CR1_CKD_1) || (tim->CR1 & TIM_CR1_CKD_0)) >> TIM_CR1_CKD_Pos;

    return TIMER_OK;
}

/**
 * @ingroup tim4
 * Assigns an external trigger input to the timer input
 *
 * @param  *tim   : Pointer to the desired timer
 * @param   input : Selects the input which finally triggers the timer
 *
 * @note
 * Consider that the selection of the trigger mode is only available for the timers TIM1/2/3/4/5/8/12. Other timers do not support
 * the trigger mode selection.
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
 *          <td>CR2</td>
 *          <td rowspan="1">TI1S</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSelectInputCaptureTriggerInput(TIM_TypeDef *tim, TRIGGER_INPUT_t input)
{
    switch (input)
    {
        case ASSIGN_TIMX_CH1:
            tim->CR2 &= ~TIM_CR2_TI1S_Msk;
            break;

        case ASSIGN_XORED_TIMX_CH1_2_3:
            tim->CR2 |= TIM_CR2_TI1S;
            break;

        default:
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim4
 * This function defines the frequency used to sample TI1 input and the length of the digital filter applied to TI1. The digital
 * filter is made of an event counter in which N consecutive events are needed to validate a transition on the output.
 *
 * FCKINT is the internal clock / FDTS = Is the clock frequency for all filters. It is derived from FCKINT
 *
 * @param  *tim     : Pointer to the desired timer
 * @param   channel : Selects the input channel of which the input signal shall be filtered
 * @param   filter  : Filter setting (frequency)
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
 *          <td>CCMR1/CCMR2</td>
 *          <td rowspan="1">IC1F/IC2F/IC3F/IC4F</td>
 *          <td rowspan="1">15...12/7...4</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetInputCaptureFilter(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, TIMER_IC_FILTER_t filter)
{
    if (timerVerifyChannelNum(channel) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (filter)
    {
        case NO_FILTER:                             // 0000
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
            }
            break;

        case FCKINT_N2:                             // 0001
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC1F_0;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC2F_0;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC3F_0;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC4F_0;
            }
            break;

        case FCKINT_N4:                             // 0010
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC1F_1;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC2F_1;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC3F_1;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC4F_1;
            }
            break;

        case FCKINT_N8:                             // 0011
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_1 | TIM_CCMR2_IC3F_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_1 | TIM_CCMR2_IC4F_0);
            }
            break;

        case FTDS_DIV_2_N6:                         // 0100
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC1F_2;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC2F_2;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC3F_2;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC4F_2;
            }
            break;

        case FTDS_DIV_2_N8:                         // 0101
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_0);
            }
            break;

        case FTDS_DIV_4_N6:                         // 0110
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_1);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_1);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_1);
            }
           break;

        case FTDS_DIV_4_N8:                         // 0111
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_IC3F_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_IC4F_0);
            }
            break;

        case FTDS_DIV_8_N6:                         // 1000
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC1F_3;
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= TIM_CCMR1_IC2F_3;
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC3F_3;
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= TIM_CCMR2_IC4F_3;
            }
            break;

        case FTDS_DIV_8_N8:                         // 1001
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_0);
            }
            break;

        case FTDS_DIV_16_N5:                        // 1010
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_1);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_1);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_1);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_1);
            }
            break;

        case FTDS_DIV_16_N8:                        // 1011
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_IC3F_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_IC4F_0);
            }
            break;

        case FTDS_DIV_16_N6:                        // 1100
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_2);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_2);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_2);
            }
            break;

        case FTDS_DIV_32_N5:                        // 1101
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_0);
            }
            break;

        case FTDS_DIV_32_N8:                        // 1110
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_1);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_1);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_1);
            }
            break;

        case FTDS_DIV_32_N6:                        // 1111
            if (TIMIO_CH1 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0);
            }
            else if (TIMIO_CH2 == channel)
            {
                tim->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
                tim->CCMR1 |= (TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0);
            }
            else if (TIMIO_CH3 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC3F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_IC3F_0);
            }
            else if (TIMIO_CH4 == channel)
            {
                tim->CCMR2 &= ~TIM_CCMR2_IC4F_Msk;
                tim->CCMR2 |= (TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_IC3F_0);
            }
            break;
    }
    return TIMER_OK;
}

/* EOF */
