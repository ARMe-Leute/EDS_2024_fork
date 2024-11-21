/**
 * @file deprecated.c
 * @author Ralf Jesse
 * @date MAR 15, 2021
 *
 * This file provides access to all <b>deprecated</b> timer functions.
 */

#include <mcalTimer/mcalTimer.h>

/**
 * @ingroup tim0
 * Don't use this function any longer. Use timerSelectTimer(TIM_TypeDef *tim) instead.
 *
 * @param   *tim : Pointer to the timer
 */
void timerBusClkOn(TIM_TypeDef *tim)
{
    timerSelectTimer(tim);
    tim->CR1 = 0;  // Reset all bits of TIMx_CR1 to 0
}

/**
 * @ingroup tim0
 * Don't use this function any longer. Use timerDeselectTimer(TIM_TypeDef *tim) instead.
 *
 * @param   *tim : Pointer to the timer
 */
void timerBusClkOff(TIM_TypeDef *tim)
{
    timerDeselectTimer(tim);
}

/**
 * @ingroup tim0
 * Don't use this function any longer. Use timerSetCapCompPreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, uint32_t ccrVal)
 * instead.
 *
 * @param   *tim    : Pointer to the timer
 * @param    chn    : Channel number
 * @param   *ccrVal : Pointer to the value which is to be compared with the current counter value
 * @return TIMER_RETURN_CODE_t
 */
TIMER_RETURN_CODE_t timerSetPreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, uint32_t ccrVal)
{
    timerSetCaptureComparePreloadValue(tim, chn, ccrVal);
    return TIMER_OK;
}

/**
 * @ingroup tim0
 * Don't use this function any longer. Use timerGetCapCompPreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, uint32_t *ccrVal)
 * instead.
 *
 * @param   *tim    : Pointer to the timer
 * @param    chn    : Channel number
 * @param   *ccrVal : Address of the variable where the current preload value shall be stored
 * @return TIMER_RETURN_CODE_t
 */
TIMER_RETURN_CODE_t  timerGetPreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, uint32_t *ccrVal)
{
    timerGetPreloadValue(tim, chn, ccrVal);

    return TIMER_OK;
}

/* EOF */
