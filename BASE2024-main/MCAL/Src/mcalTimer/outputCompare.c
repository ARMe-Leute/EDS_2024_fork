/**
 * @file    mcalTimer/outputCompare.c
 * @author  Ralf Jesse
 * @date    MAR 17, 2021
 *
 * This file provides output-compare related functions.
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @ingroup tim5
 * Sets the timer channel to TIMIO_OUTPUT and selects the operational mode.
 *
 * @param   *tim  : Pointer to the timer
 * @param    chn  : Number of the channel
 * @param    mode : Operational mode of the output-compare channel
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
 *          <td rowspan="1">CCMR1/CCMR2</td>
 *          <td rowspan="1">15...0/31...0</td>
 *          <td rowspan="1">TIM2 and TIM5 support 32 bits. Other timers with capture/compare feature support are limited to 16 bit.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetOutputCompareMode(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_OUTPUT_COMPARE_MODE_t mode)
{
    if (timerVerifyChannelNum(chn) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (chn)
    {
        case TIMIO_CH1:
            tim->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;          // Use channel in output-compare mode
            tim->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;          // Reset all OC1M bits (--> Frozen)
            switch (mode)
            {
                case CHN_FROZEN:
                    // Set OC1M = 000
                    tim->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;  // Already reset before
                    break;
                case CHN_ACTIVE_ON_MATCH:
                    // Set OC1M = 001
                    tim->CCMR1 |= TIM_CCMR1_OC1M_0;
                    break;
                case CHN_INACTIVE_ON_MATCH:
                    // Set OC1M = 010
                    tim->CCMR1 |= TIM_CCMR1_OC1M_1;
                    break;
                case CHN_TOGGLE_ON_MATCH:
                    // Set OC1M = 011
                    tim->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
                    break;
                case CHN_FORCE_LOW:
                    // Set OC1M = 100
                    tim->CCMR1 |= TIM_CCMR1_OC1M_2;
                    break;
                case CHN_FORCE_HIGH:
                    // Set OC1M = 101
                    tim->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0;
                    break;
                case CHN_PWM_MODE_1:
                    // Set OC1M = 110
                    tim->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
                    break;
                case CHN_PWM_MODE_2:
                    // Set OC1M = 111
                    tim->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
                    break;
            }
            break;

        case TIMIO_CH2:
            tim->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;          // Use channel in output-compare mode
            tim->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;          // Reset all OC2M bits
            switch (mode)
            {
                case CHN_FROZEN:
                    // Set OC2M = 000
                    tim->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;  // Already reset before
                    break;
                case CHN_ACTIVE_ON_MATCH:
                    // Set OC2M = 001
                    tim->CCMR1 |= TIM_CCMR1_OC2M_0;
                    break;
                case CHN_INACTIVE_ON_MATCH:
                    // Set OC2M = 010
                    tim->CCMR1 |= TIM_CCMR1_OC2M_1;
                    break;
                case CHN_TOGGLE_ON_MATCH:
                    // Set OC2M = 011
                    tim->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
                    break;
                case CHN_FORCE_LOW:
                    // Set OC2M = 100
                    tim->CCMR1 |= TIM_CCMR1_OC2M_2;
                    break;
                case CHN_FORCE_HIGH:
                    // Set OC2M = 101
                    tim->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0;
                    break;
                case CHN_PWM_MODE_1:
                    // Set OC2M = 110
                    tim->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
                    break;
                case CHN_PWM_MODE_2:
                    // Set OC2M = 111
                    tim->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
                    break;
            }
            break;

        case TIMIO_CH3:
            tim->CCMR2 &= ~TIM_CCMR2_CC3S_Msk;          // Use channel in output-compare mode
            tim->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;          // Reset all OC3M bits
            switch (mode)
            {
                case CHN_FROZEN:
                    // Set OC3M = 000
                    tim->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;  // Already reset before
                    break;
                case CHN_ACTIVE_ON_MATCH:
                    // Set OC3M = 001
                    tim->CCMR2 |= TIM_CCMR2_OC3M_0;
                    break;
                case CHN_INACTIVE_ON_MATCH:
                    // Set OC3M = 010
                    tim->CCMR2 |= TIM_CCMR2_OC3M_1;
                    break;
                case CHN_TOGGLE_ON_MATCH:
                    // Set OC3M = 011
                    tim->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
                    break;
                case CHN_FORCE_LOW:
                    // Set OC3M = 100
                    tim->CCMR2 |= TIM_CCMR2_OC3M_2;
                    break;
                case CHN_FORCE_HIGH:
                    // Set OC3M = 101
                    tim->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0;
                    break;
                case CHN_PWM_MODE_1:
                    // Set OC3M = 110
                    tim->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
                    break;
                case CHN_PWM_MODE_2:
                    // Set OC3M = 111
                    tim->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
                    break;
            }
            break;

        case TIMIO_CH4:
            tim->CCMR2 &= ~TIM_CCMR2_CC4S_Msk;          // Use channel in output-compare mode
            tim->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;          // Reset all OC1M bits (--> Frozen)
            switch (mode)
            {
                case CHN_FROZEN:
                    // Set OC4M = 000
                    tim->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;  // Already reset before
                    break;
                case CHN_ACTIVE_ON_MATCH:
                    // Set OC4M = 001
                    tim->CCMR2 |= TIM_CCMR2_OC4M_0;
                    break;
                case CHN_INACTIVE_ON_MATCH:
                    // Set OC4M = 010
                    tim->CCMR2 |= TIM_CCMR2_OC4M_1;
                    break;
                case CHN_TOGGLE_ON_MATCH:
                    // Set OC4M = 011
                    tim->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
                    break;
                case CHN_FORCE_LOW:
                    // Set OC4M = 100
                    tim->CCMR2 |= TIM_CCMR2_OC4M_2;
                    break;
                case CHN_FORCE_HIGH:
                    // Set OC4M = 101
                    tim->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0;
                    break;
                case CHN_PWM_MODE_1:
                    // Set OC4M = 110
                    tim->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
                    break;
                case CHN_PWM_MODE_2:
                    // Set OC4M = 111
                    tim->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
                    break;
            }
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim5
 * Enables the preload of the Capture Compare Register n (CCRn)
 *
 * @param   *tim  : Pointer to the timer
 * @param    chn  : Number of the channel
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
 *          <td rowspan="1">OC1PE/OC2PE/OC3PE/OC4PE</td>
 *          <td rowspan="1">11/3</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableOutputComparePreload(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn)
{
    if (timerVerifyChannelNum(chn) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (chn)
    {
        case TIMIO_CH1:
            tim->CCMR1 |= TIM_CCMR1_OC1PE;
            break;

        case TIMIO_CH2:
            tim->CCMR1 |= TIM_CCMR1_OC2PE;
            break;

        case TIMIO_CH3:
            tim->CCMR2 |= TIM_CCMR2_OC3PE;
            break;

        case TIMIO_CH4:
            tim->CCMR2 |= TIM_CCMR2_OC4PE;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim5
 * Disables the preload of the Capture Compare Register n (CCRn)
 *
 * @param   *tim  : Pointer to the timer
 * @param    chn  : Number of the channel
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
 *          <td rowspan="1">OC1PE/OC2PE/OC3PE/OC4PE</td>
 *          <td rowspan="1">11/3</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableOutputComparePreload(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn)
{
    if (timerVerifyChannelNum(chn) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, chn) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (chn)
    {
        case TIMIO_CH1:
            tim->CCMR1 &= ~TIM_CCMR1_OC1PE_Msk;
            break;

        case TIMIO_CH2:
            tim->CCMR1 &= ~TIM_CCMR1_OC2PE_Msk;
            break;

        case TIMIO_CH3:
            tim->CCMR2 &= ~TIM_CCMR2_OC3PE_Msk;
            break;

        case TIMIO_CH4:
            tim->CCMR2 &= ~TIM_CCMR2_OC4PE_Msk;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim5
 * Triggering of the capture-compare output of the desired channel is speed up from 5 to 3 clock cycles (minimum value).
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Number of the channel
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
 *          <td rowspan="1">OC1FE/OC2FE/OC3FE/OC4FE</td>
 *          <td rowspan="1">10/2</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableOutputCompareFastCompare(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel)
{
    if (timerVerifyChannelNum(channel) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
            tim->CCMR1 |= TIM_CCMR1_OC1FE;
            break;

        case TIMIO_CH2:
            tim->CCMR1 |= TIM_CCMR1_OC2FE;
            break;

        case TIMIO_CH3:
            tim->CCMR2 |= TIM_CCMR2_OC3FE;
            break;

        case TIMIO_CH4:
            tim->CCMR2 |= TIM_CCMR2_OC4FE;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim5
 * Leaves the triggering of the desired capture-compare output at normal speed.
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Number of the channel
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
 *          <td rowspan="1">OC1FE/OC2FE/OC3FE/OC4FE</td>
 *          <td rowspan="1">10/2</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableOutputCompareFastCompare(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel)
{
    if (timerVerifyChannelNum(channel) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
            tim->CCMR1 &= ~TIM_CCMR1_OC1FE_Msk;
            break;

        case TIMIO_CH2:
            tim->CCMR1 &= ~TIM_CCMR1_OC2FE_Msk;
            break;

        case TIMIO_CH3:
            tim->CCMR2 &= ~TIM_CCMR2_OC3FE_Msk;
            break;

        case TIMIO_CH4:
            tim->CCMR2 &= ~TIM_CCMR2_OC4FE_Msk;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim5
 * Enables reset of the desired channel output when detecting high level on the ETRF input.
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Number of the channel
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
 *          <td rowspan="1">OC1CE/OC2CE/OC3CE/OC4CE</td>
 *          <td rowspan="1">15/1</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableOutputCompareClearOutput(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel)
{
    if (timerVerifyChannelNum(channel) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
            tim->CCMR1 |= TIM_CCMR1_OC1CE;
            break;

        case TIMIO_CH2:
            tim->CCMR1 |= TIM_CCMR1_OC2CE;
            break;

        case TIMIO_CH3:
            tim->CCMR2 |= TIM_CCMR2_OC3CE;
            break;

        case TIMIO_CH4:
            tim->CCMR2 |= TIM_CCMR2_OC4CE;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim5
 * Disables reset of the desired channel output when detecting high level on the ETRF input.
 *
 * @param   *tim     : Pointer to the timer
 * @param    channel : Number of the channel
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
 *          <td rowspan="1">OC1CE/OC2CE/OC3CE/OC4CE</td>
 *          <td rowspan="1">15/1</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableOutputCompareClearOutput(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel)
{
    if (timerVerifyChannelNum(channel) != true)
    {
        return TIMER_INVALID_CHANNEL;
    }

    if (timerVerifyCaptureCompareChannelAssignment(tim, channel) != true)
    {
        return TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT;
    }

    switch (channel)
    {
        case TIMIO_CH1:
            tim->CCMR1 &= ~TIM_CCMR1_OC1CE_Msk;
            break;

        case TIMIO_CH2:
            tim->CCMR1 &= ~TIM_CCMR1_OC2CE_Msk;
            break;

        case TIMIO_CH3:
            tim->CCMR2 &= ~TIM_CCMR2_OC3CE_Msk;
            break;

        case TIMIO_CH4:
            tim->CCMR2 &= ~TIM_CCMR2_OC4CE_Msk;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim5
 * Selects the Center-Align-Mode of TIM1/2/3/4/5/8. Other timers are not supported.
 *
 * @param  *tim   : Pointer to the desired timer
 * @param   align : Center-align mode
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
 *          <td rowspan="1">CMS</td>
 *          <td rowspan="1">6...5</td>
 *          <td rowspan="1">Only TIM1/TIM2/TIM3/TIM4/TIM5/TIM8 support this function.</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSelectOutputCompareCenterAlignMode(TIM_TypeDef *tim, TIMER_CENTER_ALIGN_MODE_t align)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim))
    {
         return TIMER_INVALID_TIMER;
    }

    tim->CR1 &= ~TIM_CR1_CMS_Msk;           // Reset center-align mode to default (Edge align)
    switch (align)
    {
        case EDGE_ALIGNMENT:
            tim->CR1 &= ~TIM_CR1_CMS_Msk;
            break;

        case CENTER_ALIGN_1:
            tim->CR1 |= TIM_CR1_CMS_0;
            tim->CR1 &= ~TIM_CR1_CMS_1;
            break;

        case CENTER_ALIGN_2:
            tim->CR1 &= ~TIM_CR1_CMS_0;
            tim->CR1 |= TIM_CR1_CMS_1;
            break;

        case CENTER_ALIGN_3:
            tim->CR1 |= (TIM_CR1_CMS_1 | TIM_CR1_CMS_0);
            break;

        default:
            tim->CR1 &= ~TIM_CR1_CMS_Msk;
            break;
    }
    return TIMER_OK;
}


/* EOF */
