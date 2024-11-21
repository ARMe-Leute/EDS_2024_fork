/**
 * @file    mcalTimer/verification.c
 * @author  Ralf Jesse
 * @date    Mar 15, 2021
 */
#include <mcalTimer/mcalTimer.h>

/**
 * Verifies the channel number (range check)
 */
bool timerVerifyChannelNum(TIMER_CHANNEL_NUM_t chn)
{
    if ((TIMIO_CH1 == chn) || (TIMIO_CH2 == chn) || (TIMIO_CH3 == chn) || (TIMIO_CH4 == chn))
    {
        return true;
    }
    return false;
}

/**
 * Verifies the timer <--> interrupt combination.
 */
bool timerVerifyIrqAssignment(TIM_TypeDef *tim, TIMER_IRQ_t irq)
{

    // Check for support of trigger interrupt
    if ((TIM10 == tim || TIM11 == tim ) && (TIM_TRG_IRQ == irq))
    {
        return false;
    }

    // Only TIM1 and TIM8 support COM and BREAK interrupt.
    if (TIM1 != tim  && (TIM_COM_IRQ == irq || TIM_BREAK_IRQ == irq))
    {
        return false;
    }

    // Timers which provide only one capture/compare channel
    if ((TIM10 == tim || TIM11 == tim ) && (irq > TIM_CAPCOMP_IRQ_CH1))
    {
        return false;
    }

    // Timers which provide two capture/compare channels
    if ((TIM9 == tim ) && (irq > TIM_CAPCOMP_IRQ_CH2))
    {
        return false;
    }

    return true;
}

/**
 * Verifies the timer <--> dma assignment. DMA is only supported by TIM1,
 * TIM2, TIM3, TIM4, TIM5, and TIM8.
 */
bool timerVerifyDmaAssignment(TIM_TypeDef *tim)
{
    if ( TIM10 == tim ||  TIM11 == tim )
    {
        return false;
    }

    return true;
}

/**
 * Verifies the assignment of capture/compare channels to the timers.
 */
bool timerVerifyCaptureCompareChannelAssignment(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t num)
{
    // TIM6 and TIM7 don't provide any capture/compare channel.


    // Check assignment for channels > TIMIO_CH1
    if ((TIM10 == tim || TIM11 == tim) && (num > TIMIO_CH1))
    {
        return false;
    }

    // Check assignment for channels > TIMIO_CH2
    if ((TIM9 == tim ) && (num > TIMIO_CH2))
    {
        return false;
    }

    return true;
}

/**
 * Verifies the integrity of the Master-Slave-Mode parameter
 */
bool timerVerifyTimerMSM(TIMER_MSM_MODE_t msm)
{
    if ((MSM_DISABLED_MODE  != msm) && (MSM_ENCODER_1_MODE != msm) && (MSM_ENCODER_2_MODE != msm) &&
        (MSM_ENCODER_3_MODE != msm) && (MSM_RESET_MODE     != msm) && (MSM_GATED_MODE     != msm) &&
        (MSM_TRIGGER_MODE   != msm) && (MSM_EXT_CLK_MODE   != msm))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the integrity of the action parameter
 */
bool timerVerifyTimerTriggerActionType(TIMER_TRIGGER_ACTION_t action)
{
    if ((NORMAL != action) && (DELAYED != action))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the integrity of the desired trigger
 */
bool timerVerifyTimerTrigger(TIMER_TRIGGER_t trg)
{
    if ((TRIGGER_0 != trg) && (TRIGGER_1 != trg) && (TRIGGER_2 != trg) && (TRIGGER_3 != trg) &&
        (TRIGGER_TIM1_EDGE_DETECTION     != trg) && (TRIGGER_TIM1_FILTERED_INPUT_1 != trg)   &&
        (TRIGGER_TIM1_FILTERED_INPUT_2   != trg) && (TRIGGER_EXTERNAL != trg))
    {
        return false;
    }
    return true;
}

/**
 * Verifies that the starting register of a timers' DMA burst address is valid.
 */
bool timerVerifyDmaBurstStartingRegister(TIMER_DMA_BURST_STARTING_REGISTER_t reg)
{
    if ((TIMX_DCR == reg) || (TIMX_DMAR == reg) || (TIMX_OR == reg))
    {
        return false;
    }
    return true;
}
