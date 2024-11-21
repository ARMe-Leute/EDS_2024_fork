/**
 * @file    mcalTimer/inputCapture.c
 * @author  Ralf Jesse
 * @date    MAR 17, 2021
 *
 * Functions which can be used for both "Input Capture" and "Output Compare".
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @ingroup tim3
 * Selects whether a timer channel is used as output or as one of three input types (input mappings).
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Timer channel
 * @param    mapping : Definitions in --> TIMER_CHANNEL_MAPPING_t
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
 *          <td rowspan="1">CC1S/CC2S/CC3S/CC4S</td>
 *          <td rowspan="1">1...0/9...8/1...0/9...8</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetCaptureCompareChannelMapping(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                         TIMER_CHANNEL_MAPPING_t mapping)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
            tim->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;          // Reset to default: Timer channel is used as output

            switch (mapping)
            {
                // Channel 1: Used as output
                case MAP_TO_OUTPUT:
                    // No action needed: Timer channel is already set to output
                    break;

                    // TI1: Mapped to IC1
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC1S_0;
                    break;

                    // TI1: Mapped to IC2
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC1S_1;
                    break;

                    // TI1: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR1 |= TIM_CCMR1_CC1S_1 | TIM_CCMR1_CC1S_0;
                    break;
            }

        case TIMIO_CH2:
            tim->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;          // Reset to default: Timer channel is used as output

            switch (mapping)
            {
                // Channel 2: Used as output
                case MAP_TO_OUTPUT:
                    // No action needed: Timer channel is already set to output
                    break;

                    // TI2: Mapped to IC2
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC2S_0;
                    break;

                    // TI2: Mapped to IC1
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR1 |= TIM_CCMR1_CC2S_1;
                    break;

                    // TI2: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR1 |= TIM_CCMR1_CC2S_1 | TIM_CCMR1_CC2S_0;
                    break;
            }
            break;

        case TIMIO_CH3:
            tim->CCMR2 &= ~TIM_CCMR2_CC3S_Msk;          // Reset to default: Timer channel is used as output

            switch (mapping)
            {
                // Channel 3: Used as output
                case MAP_TO_OUTPUT:
                    // No action needed: Timer channel is already set to output
                    break;

                    // TI3: Mapped to IC3
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC3S_0;
                    break;

                    // TI3: Mapped to IC4
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC3S_1;
                    break;

                    // TI3: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR2 |= TIM_CCMR2_CC3S_1 | TIM_CCMR2_CC3S_0;
                    break;
            }
            break;

        case TIMIO_CH4:
            tim->CCMR2 &= ~TIM_CCMR2_CC4S_Msk;          // Reset to default: Timer channel is used as output

            switch (mapping)
            {
                // Channel 4: Used as output
                case MAP_TO_OUTPUT:
                    // No action needed: Timer channel is already set to output
                    break;

                    // TI4: Mapped to IC3
                case MAP_CHN_TO_SAME_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC4S_0;
                    break;

                    // TI4: Mapped to IC4
                case MAP_CHN_TO_OPPOSITE_TIMER:
                    tim->CCMR2 |= TIM_CCMR2_CC4S_1;
                    break;

                    // TI4: Mapped to TRC
                case MAP_CHN_TO_TRC:
                    tim->CCMR2 |= TIM_CCMR2_CC4S_1 | TIM_CCMR2_CC4S_0;
                    break;
            }
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Returns the channel mapping of the desired timer/channel combination. The result is stored in a variable whose address is
 * passed as a pointer.
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Timer channel
 * @param   *mapping : Pointer to a variable that stores the current channel mapping
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
 *          <td rowspan="1">CC1S/CC2S/CC3S/CC4S</td>
 *          <td rowspan="1">1...0/9...8/1...0/9...8</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerGetCaptureCompareChannelMapping(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                         TIMER_CHANNEL_MAPPING_t *mapping)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
            *mapping = ((tim->CCMR1 & TIM_CCMR1_CC1S_Msk) >> TIM_CCMR1_CC1S_Pos);
            break;

        case TIMIO_CH2:
            *mapping = ((tim->CCMR1 & TIM_CCMR1_CC2S_Msk) >> TIM_CCMR1_CC2S_Pos);
            break;

        case TIMIO_CH3:
            *mapping = ((tim->CCMR2 & TIM_CCMR2_CC3S_Msk) >> TIM_CCMR2_CC3S_Pos);
            break;

        case TIMIO_CH4:
            *mapping = ((tim->CCMR2 & TIM_CCMR2_CC4S_Msk) >> TIM_CCMR2_CC4S_Pos);
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Enables InputCapture/OutputCompare for the desired channel. I.e. the corresponding pin is enabled.
 *
 * @param   *tim   : Pointer to the timer
 * @param    chn   : Timer channel
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
 *          <td>CCER</td>
 *          <td rowspan="1">CC1E/CC2E/CC3E/CC4E</td>
 *          <td rowspan="1">0/4/8/12</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableCaptureCompareChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == chn)
    {
        tim->CCER |= TIM_CCER_CC1E;
    }
    else if (TIMIO_CH2 == chn)
    {
        tim->CCER |= TIM_CCER_CC2E;
    }
    else if (TIMIO_CH3 == chn)
    {
        tim->CCER |= TIM_CCER_CC3E;
    }
    else if (TIMIO_CH4 == chn)
    {
        tim->CCER |= TIM_CCER_CC4E;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Disables InputCapture/OutputCompare  for the desired channel. I.e. the corresponding pin is disabled.
 *
 * @param   *tim   : Pointer to the timer
 * @param    chn   : Timer channel
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
 *          <td>CCER</td>
 *          <td rowspan="1">CC1E/CC2E/CC3E/CC4E</td>
 *          <td rowspan="1">0/4/8/12</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableCaptureCompareChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == chn)
    {
        tim->CCER &= ~TIM_CCER_CC1E_Msk;
    }
    else if (TIMIO_CH2 == chn)
    {
        tim->CCER &= ~TIM_CCER_CC2E_Msk;
    }
    else if (TIMIO_CH3 == chn)
    {
        tim->CCER &= ~TIM_CCER_CC3E_Msk;
    }
    else if (TIMIO_CH4 == chn)
    {
        tim->CCER &= ~TIM_CCER_CC4E_Msk;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Enables the **complementary** InputCapture/OutputCompare mode for the desired channel.
 *
 * @param   *tim   : Pointer to the timer
 * @param    chn   : Timer channel
 * @return  TIMER_RETURN_CODE_t
 *
 * @note
 * Consider that timer channel #4 has no complementary input/output!
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
 *          <td>CCER</td>
 *          <td rowspan="1">CC1E/CC2E/CC3E</td>
 *          <td rowspan="1">2/6/10</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableCaptureCompareComplementaryChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == chn)
    {
        tim->CCER |= TIM_CCER_CC1NE;
    }
    else if (TIMIO_CH2 == chn)
    {
        tim->CCER |= TIM_CCER_CC2NE;
    }
    else if (TIMIO_CH3 == chn)
    {
        tim->CCER |= TIM_CCER_CC3NE;
    }
    else if (TIMIO_CH4 == chn)
    {
        // No action because there is not such a pin
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Disables the **complementary** InputCapture/OutputCompare mode for the desired channel.
 *
 * @param   *tim   : Pointer to the timer
 * @param    chn   : Timer channel
 * @return  TIMER_RETURN_CODE_t
 *
 * @note
 * Consider that timer channel #4 has no complementary input/output!
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
 *          <td>CCER</td>
 *          <td rowspan="1">CC1NE/CC2NE/CC3NE</td>
 *          <td rowspan="1">2/6/10</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableCaptureCompareComplementaryChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == chn)
    {
        tim->CCER &= ~TIM_CCER_CC1NE_Msk;
    }
    else if (TIMIO_CH2 == chn)
    {
        tim->CCER &= ~TIM_CCER_CC2NE_Msk;
    }
    else if (TIMIO_CH3 == chn)
    {
        tim->CCER &= ~TIM_CCER_CC3NE_Msk;
    }
    else if (TIMIO_CH4 == chn)
    {
        // No action because there is not such a pin
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Sets the default level of the InputCapture/OutputCompare pin to either ACTIVE_HIGH or ACTIVE_LOW
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Timer channel
 * @param    level   : Sets the default value either to ACTIVE_HIGH or ACTIVE_LOW
 * @return  TIMER_RETURN_CODE_t
 *
 * @note
 * Consider that timer channel #4 has no complementary input/output!
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
 *          <td>CCER</td>
 *          <td rowspan="1">CC1NE/CC2NE/CC3NE</td>
 *          <td rowspan="1">2/6/10</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetCaptureCompareOutputActiveLevel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                             TIMER_CAPCOMP_PIN_ACTIVE_LEVEL_t level)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == channel)
    {
        if (ACTIVE_HIGH == level)
        {
            tim->CCER &= ~TIM_CCER_CC1P_Msk;            // ACTIVE_HIGH --> Bit must be reset to '0'
        }
        else
        {
            tim->CCER |= TIM_CCER_CC1P;                 // ACTIVE_LOW  --> Bit must be set to '1'
        }
    }
    else if (TIMIO_CH2 == channel)
    {
        if (ACTIVE_HIGH == level)
        {
            tim->CCER &= ~TIM_CCER_CC2P_Msk;
        }
        else
        {
            tim->CCER |= TIM_CCER_CC2P;
        }
    }
    else if (TIMIO_CH3 == channel)
    {
        if (ACTIVE_HIGH == level)
        {
            tim->CCER &= ~TIM_CCER_CC3P_Msk;
        }
        else
        {
            tim->CCER |= TIM_CCER_CC3P;
        }
    }
    else if (TIMIO_CH4 == channel)
    {
        if (ACTIVE_HIGH == level)
        {
            tim->CCER &= ~TIM_CCER_CC4P_Msk;
        }
        else
        {
            tim->CCER |= TIM_CCER_CC4P;
        }
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Sets the default level of the **complementary** InputCapture/OutputCompare pin to either ACTIVE_HIGH or ACTIVE_LOW
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Timer channel
 * @param    level   : Sets the default value either to ACTIVE_HIGH or ACTIVE_LOW
 * @return  TIMER_RETURN_CODE_t
 *
 * @note
 * Consider that timer channel #4 has no complementary input/output!
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
 *          <td>CCER</td>
 *          <td rowspan="1">CC1NE/CC2NE/CC3NE</td>
 *          <td rowspan="1">2/6/10</td>
 *          <td rowspan="1">The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetCaptureCompareComplementaryPinActiveLevel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                                          TIMER_CAPCOMP_PIN_ACTIVE_LEVEL_t level)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == channel)
    {
        if (ACTIVE_HIGH == level)
        {
            tim->CCER &= ~TIM_CCER_CC1NP_Msk;            // ACTIVE_HIGH --> Bit must be reset to '0'
        }
        else
        {
            tim->CCER |= TIM_CCER_CC1NP;                 // ACTIVE_LOW  --> Bit must be set to '1'
        }
    }
    else if (TIMIO_CH2 == channel)
    {
        if (ACTIVE_HIGH == level)
        {
            tim->CCER &= ~TIM_CCER_CC2NP_Msk;
        }
        else
        {
            tim->CCER |= TIM_CCER_CC2NP;
        }
    }
    else if (TIMIO_CH3 == channel)
    {
        if (ACTIVE_HIGH == level)
        {
            tim->CCER &= ~TIM_CCER_CC3NP_Msk;
        }
        else
        {
            tim->CCER |= TIM_CCER_CC3NP;
        }
    }
    else if (TIMIO_CH4 == channel)
    {
        tim->CCER &= ~TIM_CCER_CC4P_Msk;                // There is no complementary pin
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Sets the capture-compare register (CCRx). This function can be used e.g. to define the duty cycle of PWM.
 *
 * @param   *tim        : Pointer to the timer
 * @param    channel    : Timer channel
 * @param    preloadVal : Value which determines the preload value
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
 *          <td>CCR1/CCR2/CCR3/CCR4</td>
 *          <td rowspan="1">CCR1/CCR2/CCR3/CCR4</td>
 *          <td rowspan="1">31...0 / 15...0</td>
 *          <td rowspan="1">Only TIM2 and TIM5 support 32 bit. The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetCaptureComparePreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                       uint32_t preloadVal)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == channel)
    {
        tim->CCR1 = preloadVal;
    }
    else if (TIMIO_CH2 == channel)
    {
        tim->CCR2 = preloadVal;
    }
    else if (TIMIO_CH3 == channel)
    {
        tim->CCR3 = preloadVal;
    }
    else
    {
        tim->CCR4 = preloadVal;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Retrieves the current preload value
 *
 * @param  *tim     : Pointer to the timer
 * @param   channel : Number of the capture/compare channel
 * @param  *ccrVal  : Pointer to the preload variable
 * @return TIMER_RETURN_CODE_t
 *
 * @note
 * The implementation of this function is no longer compatible to that described in the STM book.
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
 *          <td>CCR1/CCR2/CCR3/CCR4</td>
 *          <td rowspan="1">CCR1/CCR2/CCR3/CCR4</td>
 *          <td rowspan="1">31...0 / 15...0</td>
 *          <td rowspan="1">Only TIM2 and TIM5 support 32 bit. The TIM<->CHN assignment is checked internally.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerGetCaptureComparePreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, uint32_t *ccrVal)
{
    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    if (TIMIO_CH1 == channel)
    {
        *ccrVal = tim->CCR1;
    }
    else if (TIMIO_CH2 == channel)
    {
        *ccrVal = tim->CCR2;
    }
    else if (TIMIO_CH3 == channel)
    {
        *ccrVal = tim->CCR3;
    }
    else if (TIMIO_CH4 == channel)
    {
        *ccrVal = tim->CCR4;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Enables/disables capture/compare preload.
 *
 * @param  *tim     : Pointer to the desired timer
 * @param   control : TIMER_CCPC_MODE_t
 * @return  TIMER_RETURN_CODE
 *
 * @note
 * This function works only on channels which provide a complementary output.
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
 *          <td rowspan="1">CCPC</td>
 *          <td rowspan="1">0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetCaptureComparePreloadCtrlBits(TIM_TypeDef *tim, TIMER_CCPC_MODE_t control)
{
    if (TIM1 != tim)
    {
        return TIMER_INVALID_TIMER;
    }

    switch (control)
    {
        case CCPC_OFF:
            tim->CR2 &= ~TIM_CR2_CCPC_Msk;
            break;

        case CCPC_ON:
            tim->CR2 |= TIM_CR2_CCPC;
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Updates the capture/compare control bits depending onf the state of the COMG bit and/or the detection of a rising edge on the
 * input which is used for triggering the timer.
 *
 * @param  *tim    : Pointer to the desired timer
 * @param   update : TIMER_CAP_COMP_CTRL_t
 * @return  TIMER_RETURN_CODE
 *
 * @note
 * Capture/Control bits (CCPC bits) must be set to '1' before.
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
 *          <td rowspan="1">CCUS</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */TIMER_RETURN_CODE_t timerUpdateCaptureCompareCtrlBits(TIM_TypeDef *tim, TIMER_CAPCOMP_CTRL_t update)
{
    if (TIM1 != tim)
    {
        return TIMER_INVALID_TIMER;
    }

    switch (update)
    {
        case COMG_ONLY:
            tim->CR2 &= ~TIM_CR2_CCUS_Msk;
            break;

        case COMG_AND_RISING_EDGE:
            tim->CR2 |= TIM_CR2_CCUS;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Selects the outputs for the idle state.
 *
 *
 * @param  *tim    : Pointer to the desired timer
 * @param   output : TIMER_IDLE_STATE_OUTPUT_t
 * @param   state  : TIMER_IDLE_STATE_t
 * @return  TIMER_RETURN_CODE
 *
 * @note
 * Capture/Control bits (CCPC bits) must be set to '1' before.
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
 *          <td rowspan="1">CCUS</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetCaptureCompareIdleStateOutput(TIM_TypeDef *tim, TIMER_IDLE_STATE_OUTPUT_t output,
                                                          TIMER_IDLE_STATE_t state)
{
    if (TIM1 != tim)
    {
        return TIMER_INVALID_TIMER;
    }

    switch (output)
    {
        case IDLE_STATE_OUT_1:
            if (IDLE_STATE_OFF == state)
            {
                tim->CR2 &= ~TIM_CR2_OIS1_Msk;
            }
            else
            {
                tim->CR2 |= TIM_CR2_OIS1;
            }
            break;

        case IDLE_STATE_OUT_1N:
            if (IDLE_STATE_OFF == state)
            {
                tim->CR2 &= ~TIM_CR2_OIS1N_Msk;
            }
            else
            {
                tim->CR2 |= TIM_CR2_OIS1N;
            }
            break;

        case IDLE_STATE_OUT_2:
            if (IDLE_STATE_OFF == state)
            {
                tim->CR2 &= ~TIM_CR2_OIS2_Msk;
            }
            else
            {
                tim->CR2 |= TIM_CR2_OIS2;
            }
            break;

        case IDLE_STATE_OUT_2N:
            if (IDLE_STATE_OFF == state)
            {
                tim->CR2 &= ~TIM_CR2_OIS2N_Msk;
            }
            else
            {
                tim->CR2 |= TIM_CR2_OIS2N;
            }
            break;

        case IDLE_STATE_OUT_3:
            if (IDLE_STATE_OFF == state)
            {
                tim->CR2 &= ~TIM_CR2_OIS3_Msk;
            }
            else
            {
                tim->CR2 |= TIM_CR2_OIS3;
            }
            break;

        case IDLE_STATE_OUT_3N:
            if (IDLE_STATE_OFF == state)
            {
                tim->CR2 &= ~TIM_CR2_OIS3N_Msk;
            }
            else
            {
                tim->CR2 |= TIM_CR2_OIS3N;
            }
            break;

        case IDLE_STATE_OUT_4:
            if (IDLE_STATE_OFF == state)
            {
                tim->CR2 &= ~TIM_CR2_OIS4_Msk;
            }
            else
            {
                tim->CR2 |= TIM_CR2_OIS4;
            }
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim3
 * Sets the clock division rate (CKD) of the desired timer and channel.
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
TIMER_RETURN_CODE_t timerSetCaptureCompareClkRatio(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_RATIO_t ckd)
{
    // These timers provide one capture/compare channel
    if ((TIM10 == tim) || (TIM11 == tim) )
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

/* EOF */
