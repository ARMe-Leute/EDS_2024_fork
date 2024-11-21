/**
 * @file    mcalTimer/dma.c
 * @author  Ralf Jesse
 * @date    MAR 15, 2021
 * @date    $Date (last modified)
 *
 * This file provides DMA-related functions.
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @ingroup tim7
 * Enables DMA usage of timer tim.
 *
 * @param   *tim      : Pointer to the timer tim
 * @param    dmaEvent : The timer event that shall generate a DMA event
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
 *          <td>DIER</td>
 *          <td rowspan="1">UDE, CC1DE, CC2DE, CC3DE, CC4DE, TDE, COMDE</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Some timers don't support DMA</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableDMA(TIM_TypeDef *tim, TIMER_DMA_t dmaEvent)
{
    if (timerVerifyDmaAssignment(tim) != true)
    {
        return TIMER_INVALID_DMA_ASSIGNMENT;
    }

    switch (dmaEvent)
    {
        case TIM_UPDATE_DMA:
            tim->DIER |= TIM_DIER_UDE;
            break;

        case TIM_CAPCOMP_DMA_CH1:
            tim->DIER |= TIM_DIER_CC1DE;
            break;

        case TIM_CAPCOMP_DMA_CH2:
            tim->DIER |= TIM_DIER_CC2DE;
            break;

        case TIM_CAPCOMP_DMA_CH3:
            tim->DIER |= TIM_DIER_CC3DE;
            break;

        case TIM_CAPCOMP_DMA_CH4:
            tim->DIER |= TIM_DIER_CC4DE;
            break;

        case TIM_COM_DMA:
            tim->DIER |= TIM_DIER_COMDE;
            break;

        case TIM_TRG_DMA:
            tim->DIER |= TIM_DIER_TDE;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim7
 * Disables DMA usage of timer tim.
 *
 * @param   *tim      : Pointer to the timer tim
 * @param    dmaEvent : The timer event that shall not longer generate a DMA event
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
 *          <td>DIER</td>
 *          <td rowspan="1">UDE, CC1DE, CC2DE, CC3DE, CC4DE, TDE, COMDE</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Some timers don't support DMA</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableDMA(TIM_TypeDef *tim, TIMER_DMA_t dmaEvent)
{
    if (timerVerifyDmaAssignment(tim) != true)
    {
        return TIMER_INVALID_DMA_ASSIGNMENT;
    }

    switch (dmaEvent)
    {
        case TIM_UPDATE_DMA:
            tim->DIER &= ~TIM_DIER_UDE_Msk;
            break;

        case TIM_CAPCOMP_DMA_CH1:
            tim->DIER &= ~TIM_DIER_CC1DE_Msk;
            break;

        case TIM_CAPCOMP_DMA_CH2:
            tim->DIER &= ~TIM_DIER_CC2DE_Msk;
            break;

        case TIM_CAPCOMP_DMA_CH3:
            tim->DIER &= ~TIM_DIER_CC3DE_Msk;
            break;

        case TIM_CAPCOMP_DMA_CH4:
            tim->DIER &= ~TIM_DIER_CC4DE_Msk;
            break;

        case TIM_COM_DMA:
            tim->DIER &= ~TIM_DIER_COMDE_Msk;
            break;

        case TIM_TRG_DMA:
            tim->DIER &= ~TIM_DIER_TDE_Msk;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim7
 * Defines the time when the DMA request shall be triggered. A DMA request is triggered on either a capture-compare or an update
 * event.
 *
 *
 * @param  *tim  : Pointer to the desired timer
 * @param   mode : Must be in [SEND_ON_CCX_MATCH, SEND_ON_UPDT_EVENT]
 *
 * @note
 * Timer TIM10 to 14 do not support DMA. However, all other timers including the Basic Timers TIM6/7 support at least one DMA
 * mode.
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
 *          <td rowspan="1">CCDS</td>
 *          <td rowspan="1">3</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSelectDMATriggerMode(TIM_TypeDef *tim, TIMER_DMA_TRIGGER_MODE_t mode)
{
    switch (mode)
    {
        case SEND_ON_CCX_MATCH:
            tim->CR2 &= ~TIM_CR2_CCDS_Msk;
            break;

        case SEND_ON_UPDT_EVENT:
            tim->CR2 |= TIM_CR2_CCDS;
            break;

        default:
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim7
 * Selects the trigger output
 *
 * @param  *tim    : Pointer to the desired timer
 * @param   output : Defines the output which triggers the DMAC
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
 *          <td rowspan="1">MMS</td>
 *          <td rowspan="1">6...4</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t  timerSelectDmaTriggerOutput(TIM_TypeDef *tim, TRIGGER_OUTPUT_t output)
{
    switch(output)
    {
        case RESET_TRIGGER_OUTPUT:
            tim->CR2 &= ~TIM_CR2_MMS_Msk;
            break;

        case CEN_IS_TRIGGER_OUTPUT:
            tim->CR2 |= TIM_CR2_MMS_0;
            break;

        case UPDATE_IS_TRIGGER_OUTPUT:
            tim->CR2 |= TIM_CR2_MMS_1;
            break;

        case SEND_COMPARE_PULSE_ON_TRG_OUTPUT:
            tim->CR2 |= (TIM_CR2_MMS_1 | TIM_CR2_MMS_0);
            break;

        case OC1REF_IS_TRIGGER_OUTPUT:
            tim->CR2 |= TIM_CR2_MMS_2;
            break;

        case OC2REF_IS_TRIGGER_OUTPUT:
            tim->CR2 |= (TIM_CR2_MMS_2 | TIM_CR2_MMS_0);
            break;

        case OC3REF_IS_TRIGGER_OUTPUT:
            tim->CR2 |= (TIM_CR2_MMS_2 | TIM_CR2_MMS_2);;
            break;

        case OC4REF_IS_TRIGGER_OUTPUT:
            tim->CR2 |= (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0);
            break;

        default:
            tim->CR2 &= ~TIM_CR2_MMS_Msk;
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim7
 * Defines the register where the DMA burst access shall start. Use this function **before** setting the length of the burst
 * transfer with **timerSetDmaBurstLength()**.
 *
 * @param  *tim : Pointer to the desired timer
 * @param   reg : Name of the register as it is defined in the TIM_TypeDef struct.
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
 *          <td>DCR</td>
 *          <td rowspan="1">DBA</td>
 *          <td rowspan="1">4...0</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetDmaBurstStartingRegister(TIM_TypeDef *tim, TIMER_DMA_BURST_STARTING_REGISTER_t reg)
{
    if (timerVerifyDmaAssignment(tim) != true)
    {
        return TIMER_INVALID_TIMER;
    }

    tim->DCR &= ~TIM_DCR_DBA_Msk;
    switch (reg)
    {
        case TIMX_CR1:      // 0b00000
            tim->DCR &= ~TIM_DCR_DBA_Msk;
            break;

        case TIMX_CR2:      // 0b00001
            tim->DCR |= TIM_DCR_DBA_0;
            break;

        case TIMX_SMCR:     // 0b00010
            tim->DCR |= TIM_DCR_DBA_1;
            break;

        case TIMX_DIER:     // 0b00011
            tim->DCR |= (TIM_DCR_DBA_1 | TIM_DCR_DBA_0);
            break;

        case TIMX_SR:       // 0b00100
            tim->DCR |= TIM_DCR_DBA_2;
            break;

        case TIMX_EGR:      // 0b00101
            tim->DCR |= (TIM_DCR_DBA_2 | TIM_DCR_DBA_0);
            break;

        case TIMX_CCMR1:    // 0b00110
            tim->DCR |= (TIM_DCR_DBA_2 | TIM_DCR_DBA_1);
            break;

        case TIMX_CCMR2:    // 0b00111
            tim->DCR |= (TIM_DCR_DBA_2 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0);
            break;

        case TIMX_CCER:     // 0b01000
            tim->DCR |= TIM_DCR_DBA_3;
            break;

        case TIMX_CNT:      // 0b01001
            tim->DCR |= (TIM_DCR_DBA_3 | TIM_DCR_DBA_0);
            break;

        case TIMX_PSC:      // 0b01010
            tim->DCR |= (TIM_DCR_DBA_3 | TIM_DCR_DBA_1);
            break;

        case TIMX_ARR:      // 0b01011
            tim->DCR |= (TIM_DCR_DBA_3 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0);
            break;

        case TIMX_RCR:      // 0b01100
            tim->DCR |= (TIM_DCR_DBA_3 | TIM_DCR_DBA_2);
            break;

        case TIMX_CCR1:     // 0b01101
            tim->DCR |= (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_0);
            break;

        case TIMX_CCR2:     // 0b01110
            tim->DCR |= (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1);
            break;

        case TIMX_CCR3:     // 0b01111
            tim->DCR |= (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0);
            break;

        case TIMX_CCR4:     // 0b10000
            tim->DCR |= TIM_DCR_DBA_4;
            break;

        case TIMX_BDTR:     // 0b10001
            tim->DCR |= (TIM_DCR_DBA_4 | TIM_DCR_DBA_0);
            break;

        case TIMX_DCR:      // 0b10010
            tim->DCR |= (TIM_DCR_DBA_4 | TIM_DCR_DBA_1);
            break;

        case TIMX_DMAR:     // 0b10011
            tim->DCR |= (TIM_DCR_DBA_4 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0);
            break;

        case TIMX_OR:       // 0b10100
            tim->DCR |= (TIM_DCR_DBA_4 | TIM_DCR_DBA_2);
            break;
    }


    return TIMER_OK;
}

/**
 * @ingroup tim7
 * Defines the number of registers which are affected by a burst transfer (including the starting register). numTransfers is
 * adjusted and will not exceed the total number of timer registers. The registers DCR, DMAR, and OR cannot be used as starting
 * register.
 *
 * @param  *tim      : Pointer to the desired timer
 * @param   numBurst : Number of registers which are affected by a burst transfer
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
 *          <td>DCR</td>
 *          <td rowspan="1">DBL</td>
 *          <td rowspan="1">12...8</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerSetDmaBurstLength(TIM_TypeDef *tim, TIMER_DMA_BURST_LEN_t numBurst)
{
    TIMER_DMA_BURST_STARTING_REGISTER_t reg = (tim->DCR & TIM_DCR_DBA_Msk) >> TIM_DCR_DBA_Pos;

    if (timerVerifyDmaAssignment(tim) != true)
    {
        return TIMER_INVALID_TIMER;
    }

    if (timerVerifyDmaBurstStartingRegister(reg) != true)
    {
        return TIMER_INVALID_DMA_BURST_STARTING_REGISTER;
    }

    if (reg + numBurst >= MAX_NUM_TIMER_REGISTERS)
    {
        return TIMER_INVALID_DMA_BURST_LENGTH;
    }

    tim->DCR &= ~TIM_DCR_DBL_Msk;
    switch(numBurst)
    {
        case TRANSF_X01:    // 0b00000
            // Nothing to be done. numBursts already set to minimum value.
            break;

        case TRANSF_X02:    // 0b00001
            tim->DCR |= TIM_DCR_DBL_0;
            break;

        case TRANSF_X03:    // 0b00010
            tim->DCR |= TIM_DCR_DBL_1;
            break;

        case TRANSF_X04:    // 0b00011
            tim->DCR |= (TIM_DCR_DBL_1 | TIM_DCR_DBL_0);
            break;

        case TRANSF_X05:    // 0b00100
            tim->DCR |= TIM_DCR_DBL_2;
            break;

        case TRANSF_X06:    // 0b00101
            tim->DCR |= (TIM_DCR_DBL_2 | TIM_DCR_DBL_0);
            break;

        case TRANSF_X07:    // 0b00110
            tim->DCR |= (TIM_DCR_DBL_2 | TIM_DCR_DBL_1);
            break;

        case TRANSF_X08:    // 0b00111
            tim->DCR |= (TIM_DCR_DBL_2 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0);
            break;

        case TRANSF_X09:    // 0b01000
            tim->DCR |= TIM_DCR_DBL_3;
            break;

        case TRANSF_X10:    // 0b01001
            tim->DCR |= (TIM_DCR_DBL_3 | TIM_DCR_DBL_0);
            break;

        case TRANSF_X11:    // 0b01010
            tim->DCR |= (TIM_DCR_DBL_3 | TIM_DCR_DBL_1);
            break;

        case TRANSF_X12:    // 0b01011
            tim->DCR |= (TIM_DCR_DBL_3 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0);
            break;

        case TRANSF_X13:    // 0b01100
            tim->DCR |= (TIM_DCR_DBL_3 | TIM_DCR_DBL_2);
            break;

        case TRANSF_X14:    // 0b01101
            tim->DCR |= (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_0);
            break;

        case TRANSF_X15:    // 0b01110
            tim->DCR |= (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_1);
            break;

        case TRANSF_X16:    // 0b01111
            tim->DCR |= (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0);
            break;

        case TRANSF_X17:    // 0b10000
            tim->DCR |= TIM_DCR_DBL_4;
            break;

        case TRANSF_X18:    // 0b10001
            tim->DCR |= (TIM_DCR_DBL_4 | TIM_DCR_DBL_0);
            break;
    }

    return TIMER_OK;
}


/* EOF */
