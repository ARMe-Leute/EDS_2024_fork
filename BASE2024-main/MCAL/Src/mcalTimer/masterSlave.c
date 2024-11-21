/**
 * @file    mcalTimer/masterSlave.c
 * @author  Ralf Jesse
 * @date    MAR 15, 2021
 *
 * This file provides functions related to event generation.
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @ingroup tim9
 * Selects the behaviour according to a trigger event
 *
 * @param  *tim : Pointer to the desired timer
 * @param   action : TIMER_TRIGGER_ACTION_t
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
 *          <td>SMCR</td>
 *          <td rowspan="1">MSM</td>
 *          <td rowspan="1">7</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSelectMasterSlaveTriggerAction(TIM_TypeDef *tim, TIMER_TRIGGER_ACTION_t action)
{
    if (timerVerifyTimerTriggerActionType(action) == false)
    {
        return TIMER_INVALID_TRIGGER_ACTION;
    }

    switch (action)
    {
        case NORMAL:
            tim->SMCR &= ~TIM_SMCR_MSM_Msk;
            break;

        case DELAYED:
            tim->SMCR |= TIM_SMCR_MSM;
            break;

        default:
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim9
 * Selects the Master-Slave mode of the timer
 *
 * @param  *tim : Pointer to the desired timer
 * @param   msm : TIMER_MASTER_SLAVE_MODE_t
 *
 * @note
 * Consider that Slave Mode Selection (SMS) is only supported by the timers TIM1/2/3/4/5/8/9/12. All other timers even if they
 * belong to the General-Purpose Timer do not support the SMS mode.
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
 *          <td>SMCR</td>
 *          <td rowspan="1">SMS</td>
 *          <td rowspan="1">2...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSelectMasterSlaveMode(TIM_TypeDef *tim, TIMER_MSM_MODE_t msm)
{
    if (timerVerifyTimerMSM(msm) == false)
    {
        return TIMER_INVALID_MASTER_SLAVE_MODE_TYPE;
    }

    switch (msm)
    {
        case MSM_DISABLED_MODE:
            tim->SMCR &= ~TIM_SMCR_SMS_Msk;
            break;

        case MSM_ENCODER_1_MODE:
            tim->SMCR |= TIM_SMCR_SMS_0;
            break;

        case MSM_ENCODER_2_MODE:
            tim->SMCR |= TIM_SMCR_SMS_1;
            break;

        case MSM_ENCODER_3_MODE:
            tim->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);
            break;

        case MSM_RESET_MODE:
            tim->SMCR |= TIM_SMCR_SMS_2;
            break;

        case MSM_GATED_MODE:
            tim->SMCR |= (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0);
            break;

        case MSM_TRIGGER_MODE:
            tim->SMCR |= (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1);
            break;

        case MSM_EXT_CLK_MODE:
            tim->SMCR |= (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);
            break;

        default:
            tim->SMCR &= ~TIM_SMCR_SMS_Msk;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim9
 * This function selects the trigger input that shall be used to synchronize the counter.
 *
 * @param  *tim     : Pointer to the desired timer
 * @param   trigger : TIMER_TRIGGER_t
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
 *          <td>SMCR</td>
 *          <td rowspan="1">TS</td>
 *          <td rowspan="1">6...4</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSelectMasterSlaveTriggerSyncInput(TIM_TypeDef *tim, TIMER_TRIGGER_t trigger)
{
    if (timerVerifyTimerTrigger(trigger) == false)
    {
        return TIMER_INVALID_TRIGGER;
    }

    switch (trigger)
    {
        case TRIGGER_0:
            tim->SMCR &= ~TIM_SMCR_TS_Msk;
            break;

        case TRIGGER_1:
            tim->SMCR |= TIM_SMCR_TS_0;
            break;

        case TRIGGER_2:
            tim->SMCR |= TIM_SMCR_TS_1;
            break;

        case TRIGGER_3:
            tim->SMCR |= (TIM_SMCR_TS_1 | TIM_SMCR_TS_0);
            break;

        case TRIGGER_TIM1_EDGE_DETECTION:
            tim->SMCR |= TIM_SMCR_TS_2;
            break;

        case TRIGGER_TIM1_FILTERED_INPUT_1:
            tim->SMCR |= (TIM_SMCR_TS_2 | TIM_SMCR_TS_0);
            break;

        case TRIGGER_TIM1_FILTERED_INPUT_2:
            tim->SMCR |= (TIM_SMCR_TS_2 | TIM_SMCR_TS_1);
            break;

        case TRIGGER_EXTERNAL:
            tim->SMCR |= (TIM_SMCR_TS_2 | TIM_SMCR_TS_1 | TIM_SMCR_TS_0);
            break;

        default:
            tim->SMCR &= ~TIM_SMCR_TS_Msk;
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim9
 * Sets the polarity of the external trigger signal either to ACTIVE HIGH/RISING EDGE or ACTIVE LOW/FALLING EDGE.
 *
 * @param  *tim      : Pointer to the desired timer
 * @param   polarity : see TIEMR_EXT_TRIG_POL_t
 * @return  TIMER_RETURN_CODE
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
 *          <td>SMCR</td>
 *          <td rowspan="1">ETP</td>
 *          <td rowspan="1">15</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSetMasterSlaveExternalTriggerPolarity(TIM_TypeDef *tim, TIMER_EXT_TRIG_POL_t polarity)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim) )
    {
        return TIMER_INVALID_TIMER;
    }

    switch (polarity)
    {
        case ACTIVE_HIGH_OR_RISING_EDGE:
            tim->SMCR &= ~TIM_SMCR_ETP_Msk;
            break;

        case ACTIVE_LOW_OR_FALLING_EDGE:
            tim->SMCR |= TIM_SMCR_ETP;
            break;
    }
    return TIMER_OK;
}

/**
 * @ingroup tim9
 * Enables the external clock.
 *
 * @param  *tim : Pointer to the desired timer
 * @return  TIMER_RETURN_CODE
 *
 * @note
 * This function does <b>not</b> provide the full functionality of the SMCR/ECE bit since there are multiple conditions which are
 * not considered! It might be required to take care of this conditions by writing a separate function.
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
 *          <td>SMCR</td>
 *          <td rowspan="1">ECE</td>
 *          <td rowspan="1">14</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerEnableMasterSlaveExternalTriggerClock(TIM_TypeDef *tim)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim) )
    {
        return TIMER_INVALID_TIMER;
    }
    tim->SMCR |= TIM_SMCR_ECE;

    return TIMER_OK;
}

/**
 * @ingroup tim9
 * Disables the external clock.
 *
 * @param  *tim : Pointer to the desired timer
 * @return  TIMER_RETURN_CODE
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
 *          <td>SMCR</td>
 *          <td rowspan="1">ECE</td>
 *          <td rowspan="1">14</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerDisableMasterSlaveExternalTriggerClock(TIM_TypeDef *tim)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim) )
    {
        return TIMER_INVALID_TIMER;
    }
    tim->SMCR &= ~TIM_SMCR_ECE_Msk;

    return TIMER_OK;
}

/**
 * @ingroup tim9
 * Defines the prescaler for the external clock signal.
 *
 * @param  *tim    : Pointer to the desired timer
 * @param   extPsc : see TIMER_EXT_TRIG_PSC
 * @return  TIMER_RETURN_CODE
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
 *          <td>SMCR</td>
 *          <td rowspan="1">ETPS</td>
 *          <td rowspan="1">13</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSelectMasterSlaveExternalTriggerPrescaler(TIM_TypeDef *tim, TIMER_EXT_TRIG_PSC extPsc)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim) )
    {
        return TIMER_INVALID_TIMER;
    }

    tim->SMCR &= ~TIM_SMCR_ETPS_Msk;
    switch (extPsc)
    {
        case PSC_OFF:
            tim->SMCR &= ~TIM_SMCR_ETPS_Msk;
            break;

        case PSC_DIV_2:
            tim->SMCR |= TIM_SMCR_ETPS_0;
            break;

        case PSC_DIV_4:
            tim->SMCR |= TIM_SMCR_ETPS_1;
            break;

        case PSC_DIV_8:
            tim->SMCR |= (TIM_SMCR_ETPS_1 | TIM_SMCR_ETPS_0);
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim9
 * Defines the frequency <b>and</b> the number of consecutive events that must happen before a transition of the output occurs.
 *
 * @param  *tim    : Pointer to the desired timer
 * @param   filter : TIMER_EXT_TRIG_FILTER_t provides the divisor with N being the number of consecutive events.
 * @return  TIMER_RETURN_CODE
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
 *          <td>SMCR</td>
 *          <td rowspan="1">ETF</td>
 *          <td rowspan="1">11...8</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSelectMasterSlaveExternalTriggerFilter(TIM_TypeDef *tim, TIMER_EXT_TRIG_FILTER_t filter)
{
    if ((TIM1 != tim) && (TIM2 != tim) && (TIM3 != tim) && (TIM4 != tim) && (TIM5 != tim) )
    {
        return TIMER_INVALID_TIMER;
    }

    tim->SMCR &= ~TIM_SMCR_ETF_Msk;         // Reset to default settings
    switch (filter)
    {
        case EXTTRIG_FILTER_OFF:
            tim->SMCR &= ~TIM_SMCR_ETF_Msk;
            break;

        case EXTTRIG_FCK_INT_2:
            tim->SMCR |= TIM_SMCR_ETF_0;
            break;

        case EXTTRIG_FCK_INT_4:
            tim->SMCR |= TIM_SMCR_ETF_1;
            break;

        case EXTTRIG_FCK_INT_8:
            tim->SMCR |= (TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0);
            break;

        case EXTTRIG_FDTS_DIV_2_N6:
            tim->SMCR |= TIM_SMCR_ETF_2;
            break;

        case EXTTRIG_FDTS_DIV_2_N8:
            tim->SMCR |= (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_0);
            break;

        case EXTTRIG_FDTS_DIV_4_N6:
            tim->SMCR |= (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1);
            break;

        case EXTTRIG_FDTS_DIV_4_N8:
            tim->SMCR |= (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0);
            break;

        case EXTTRIG_FDTS_DIV_8_N6:
            tim->SMCR |= TIM_SMCR_ETF_3;
            break;

        case EXTTRIG_FDTS_DIV_8_N8:
            tim->SMCR |= (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_0);
            break;

        case EXTTRIG_FDTS_DIV_16_N5:
            tim->SMCR |= (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1);
            break;

        case EXTTRIG_FDTS_DIV_16_N6:
            tim->SMCR |= (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0);
            break;

        case EXTTRIG_FDTS_DIV_16_N8:
            tim->SMCR |= (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2);
            break;

        case EXTTRIG_FDTS_DIV_32_N5:
            tim->SMCR |= (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 | TIM_SMCR_ETF_0);
            break;

        case EXTTRIG_FDTS_DIV_32_N6:
            tim->SMCR |= (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1);
            break;

        case EXTTRIG_FDTS_DIV_32_N8:
            tim->SMCR |= (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0);
            break;
    }

    return TIMER_OK;
}


/* EOF */
