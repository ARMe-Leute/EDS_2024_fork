/**
 * @file    mcalTimer/interrupt.c
 * @author  Ralf Jesse
 * @date    MAR 15, 2021
 *
 * This file provides functions related to interrupts.
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @deprecated
 * @ingroup tim0
 * This bit must be reset after a counter update event
 *
 * @note
 * Use timerClearInterruptFlag(TIM_TypeDef *tim, TIMER_IRQ_t irq) instead.
 *
 * @param  *tim : Pointer to the timer
 * @param   irq : Interrupt flag that shall be cleared
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
 *          <td>SR</td>
 *          <td rowspan="1">UIF, CC1IF, CC2IF, CC3IF, CC4IF, TIF, COMIF, BIF, CC1OF, CC2OF, CC3OF, CC4OF</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Not all interrupts are supported by all timers</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerClearIrqFlag(TIM_TypeDef *tim, TIMER_IRQ_t irq)
{
    return timerClearInterruptFlag(tim, irq);
}

/**
 * @ingroup tim6
 * Enables the interrupts of timer tim.
 *
 * @param   *tim : Pointer to the timer tim
 * @param    irq : IRQ type
 * @return  TIMER_RETURN_CODE_t
 *
 * @note
 * Timers which don't provide any capture/compare channel or provide
 * only a limited number of capture/compare channels return a corresponding
 * error code.
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
 *          <td rowspan="1">UIE, CC1IE, CC2IE, CC3IE, CC4IE, TIE, COMIE, BIE</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Not all interrupts are supported by all timers</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerEnableInterrupt(TIM_TypeDef *tim, TIMER_IRQ_t irq)
{
    if (timerVerifyIrqAssignment(tim, irq) != true)
    {
        return TIMER_INVALID_IRQ_TYPE;
    }

    // TIM1 - TIM5/TIM8 provide four capture/compare channels
    switch (irq)
    {
        case TIM_IRQ_UIE:
            tim->DIER |= TIM_DIER_UIE;
            break;

        case TIM_CAPCOMP_IRQ_CH1:
            tim->DIER |= TIM_DIER_CC1IE;
            break;

        case TIM_CAPCOMP_IRQ_CH2:
            tim->DIER |= TIM_DIER_CC2IE;
            break;

        case TIM_CAPCOMP_IRQ_CH3:
            tim->DIER |= TIM_DIER_CC3IE;
            break;

        case TIM_CAPCOMP_IRQ_CH4:
            tim->DIER |= TIM_DIER_CC4IE;
            break;

        case TIM_TRG_IRQ:
            tim->DIER |= TIM_DIER_TIE;
            break;

        case TIM_COM_IRQ:
            tim->DIER |= TIM_DIER_COMIE;
            break;

        case TIM_BREAK_IRQ:
            tim->DIER |= TIM_DIER_BIE;
            break;

        // The following interrupts are handled by hardware only!
        case TIM_OVERCAPTURE_1:
        case TIM_OVERCAPTURE_2:
        case TIM_OVERCAPTURE_3:
        case TIM_OVERCAPTURE_4:
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim6
 * Disables the interrupt of timer tim.
 *
 * @param   *tim : Pointer to the timer tim
 * @param    irq : IRQ type
 * @return  TIMER_RETURN_CODE_t
 *
 * @note
 * Timers which don't provide any capture/compare channel or provide
 * only a limited number of capture/compare channels return a corresponding
 * error code.
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
 *          <td rowspan="1">UIE, CC1IE, CC2IE, CC3IE, CC4IE, TIE, COMIE, BIE</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Not all interrupts are supported by all timers</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerDisableInterrupt(TIM_TypeDef *tim, TIMER_IRQ_t irq)
{
    if (timerVerifyIrqAssignment(tim, irq) != true)
    {
        return TIMER_INVALID_IRQ_TYPE;
    }

    // TIM1 - TIM5/TIM8 provide four capture/compare channels
    switch (irq)
    {
        case TIM_IRQ_UIE:
            tim->DIER &= ~TIM_DIER_UIE_Msk;
            break;

        case TIM_CAPCOMP_IRQ_CH1:
            tim->DIER &= ~TIM_DIER_CC1IE_Msk;
            break;

        case TIM_CAPCOMP_IRQ_CH2:
            tim->DIER &= ~TIM_DIER_CC2IE_Msk;
            break;

        case TIM_CAPCOMP_IRQ_CH3:
            tim->DIER &= ~TIM_DIER_CC3IE_Msk;
            break;

        case TIM_CAPCOMP_IRQ_CH4:
            tim->DIER &= ~TIM_DIER_CC4IE_Msk;
            break;

        case TIM_TRG_IRQ:
            tim->DIER &= ~TIM_DIER_TIE_Msk;
            break;

        case TIM_COM_IRQ:
            tim->DIER &= ~TIM_DIER_COMIE_Msk;
            break;

        case TIM_BREAK_IRQ:
            tim->DIER &= ~TIM_DIER_BIE_Msk;
            break;

        // The following interrupts are handled by hardware only!
        case TIM_OVERCAPTURE_1:
        case TIM_OVERCAPTURE_2:
        case TIM_OVERCAPTURE_3:
        case TIM_OVERCAPTURE_4:
            break;
    }

    return TIMER_OK;
}

/**
 * @ingroup tim6
 * This bit must be reset after a counter update event
 *
 * @param  *tim : Pointer to the timer
 * @param   irq : Interrupt flag that shall be cleared
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
 *          <td>SR</td>
 *          <td rowspan="1">UIF, CC1IF, CC2IF, CC3IF, CC4IF, TIF, COMIF, BIF, CC1OF, CC2OF, CC3OF, CC4OF</td>
 *          <td rowspan="1">7...0</td>
 *          <td rowspan="1">Not all interrupts are supported by all timers</td>
 *      </tr>
 * </table>
 */
TIMER_RETURN_CODE_t timerClearInterruptFlag(TIM_TypeDef *tim, TIMER_IRQ_t irq)
{
    if (timerVerifyIrqAssignment(tim, irq) != true)
    {
        return TIMER_INVALID_IRQ_TYPE;
    }

    // TIM1 - TIM5/TIM8 provide four capture/compare channels
    switch (irq)
    {
        case TIM_IRQ_UIE:
            tim->SR &= ~TIM_SR_UIF;
            break;

        case TIM_CAPCOMP_IRQ_CH1:
            tim->SR &= ~TIM_SR_CC1IF;
            break;

        case TIM_CAPCOMP_IRQ_CH2:
            tim->SR &= ~TIM_SR_CC2IF;
            break;

        case TIM_CAPCOMP_IRQ_CH3:
            tim->SR &= ~TIM_SR_CC3IF;
            break;

        case TIM_CAPCOMP_IRQ_CH4:
            tim->SR &= ~TIM_SR_CC4IF;
            break;

        case TIM_TRG_IRQ:
            tim->SR &= ~TIM_SR_TIF;
            break;

        case TIM_COM_IRQ:
            tim->SR &= ~TIM_SR_COMIF;
            break;

        case TIM_BREAK_IRQ:
            tim->SR &= ~TIM_SR_BIF;
            break;

        case TIM_OVERCAPTURE_1:
            tim->SR &= ~TIM_SR_CC1OF_Msk;
            break;

        case TIM_OVERCAPTURE_2:
            tim->SR &= ~TIM_SR_CC2OF_Msk;
            break;

        case TIM_OVERCAPTURE_3:
            tim->SR &= ~TIM_SR_CC3OF_Msk;
            break;

        case TIM_OVERCAPTURE_4:
            tim->SR &= ~TIM_SR_CC4OF_Msk;
            break;
    }

    return TIMER_OK;
}
