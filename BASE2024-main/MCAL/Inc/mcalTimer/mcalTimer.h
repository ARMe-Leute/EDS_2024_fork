/**
 * mcalTimer.h
 * @brief Constants and enumerations related to all timer classes.
 *
 *  Created on: 09.07.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALTIMER_H_
#define MCALTIMER_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include <mcalTimer/typedefs.h>

#ifdef __cplusplus
extern "C" {
#endif


/*********************************************************************/
#define TIMER_MAX_PRESCALER         (65536)


/**
 * Function prototypes with information in which file you can find the implementation.
 */

// Function implementations are coded in file mcalTimer/deprecated.c
extern void                 timerBusClkOn(TIM_TypeDef *tim);
extern void                 timerBusClkOff(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerClearIrqFlag(TIM_TypeDef *tim, TIMER_IRQ_t irq);
extern TIMER_RETURN_CODE_t  timerSetPreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, uint32_t ccrVal);


// Function implementations coded in file mcalTimer/verification.c
extern bool                 timerVerifyTimerTrigger(TIMER_TRIGGER_t trg);
extern bool                 timerVerifyTimerTriggerActionType(TIMER_TRIGGER_ACTION_t action);
extern bool                 timerVerifyTimerMSM(TIMER_MSM_MODE_t msm);
extern bool                 timerVerifyCaptureCompareChannelAssignment(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t num);
extern bool                 timerVerifyDmaAssignment(TIM_TypeDef *tim);
extern bool                 timerVerifyIrqAssignment(TIM_TypeDef *tim, TIMER_IRQ_t irq);
extern bool                 timerVerifyChannelNum(TIMER_CHANNEL_NUM_t chn);
extern bool                 timerVerifyDmaBurstStartingRegister(TIMER_DMA_BURST_STARTING_REGISTER_t reg);

// Function implementations are coded in file mcalTimer/mcalTimer.c: These functions are considered as 'common functions'.
extern void                 timerSelectTimer(TIM_TypeDef *tim);
extern void                 timerDeselectTimer(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerEnableAutoReloadPreload(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerDisableAutoReloadPreload(TIM_TypeDef *tim);
extern void                 timerSetPrescaler(TIM_TypeDef *tim, uint32_t psc);
extern uint32_t             timerGetPrescaler(TIM_TypeDef *tim);
extern void                 timerSetAutoReloadValue(TIM_TypeDef *tim, uint32_t reload);
extern uint32_t             timerGetAutoReloadValue(TIM_TypeDef *tim);
extern void                 timerSetCounter(TIM_TypeDef *tim, uint32_t cnt);
extern uint32_t             timerGetCounter(TIM_TypeDef *tim);
extern void                 timerResetCounter(TIM_TypeDef *tim);
extern void                 timerStartTimer(TIM_TypeDef *tim);
extern void                 timerStopTimer(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerEnableOneShot(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerDisableOneShot(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerGetOneShotMode(TIM_TypeDef *tim, bool *opm);
extern TIMER_RETURN_CODE_t  timerSetCountingDir(TIM_TypeDef *tim, TIMER_COUNT_DIR_t dir);
extern TIMER_RETURN_CODE_t  timerSetCaptureCompareClkRatio(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_RATIO_t ckd);

// Function implementations are coded in file mcalTimer/advancedControl.c
extern TIMER_RETURN_CODE_t  timerSetBreakAndDeadTime(TIM_TypeDef *tim, uint8_t deadTime);
extern TIMER_RETURN_CODE_t  timerBdtrClearRegister(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerBdtrEnableMainOutput(TIM_TypeDef *tim, TIMER_BDTR_MOE_t moeMode);
extern TIMER_RETURN_CODE_t  timerBdtrAutomaticOutputEnable(TIM_TypeDef *tim, TIMER_BDTR_AOE_t aoeMode);
extern TIMER_RETURN_CODE_t  timerBdtrSetBreakPolarity(TIM_TypeDef *tim, TIMER_BDTR_BKP_POLARITY_t bkpMode);
extern TIMER_RETURN_CODE_t  timerBdtrBreakEnable(TIM_TypeDef *tim, TIMER_BDTR_BKE_t bkeMode);
extern TIMER_RETURN_CODE_t  timerSetRepetitionCounter(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, uint8_t value);


// Common capture/compare functions are coded in file mcalTimer/commonCapComp.c
extern TIMER_RETURN_CODE_t  timerSetCaptureCompareChannelMapping(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                                 TIMER_CHANNEL_MAPPING_t type);
extern TIMER_RETURN_CODE_t  timerGetCaptureCompareChannelMapping(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                                 TIMER_CHANNEL_MAPPING_t *type);
extern TIMER_RETURN_CODE_t  timerEnableCaptureCompareChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);
extern TIMER_RETURN_CODE_t  timerDisableCaptureCompareChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);
extern TIMER_RETURN_CODE_t  timerEnableCaptureCompareComplementaryChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);
extern TIMER_RETURN_CODE_t  timerDisableCaptureCompareComplementaryChannel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);
extern TIMER_RETURN_CODE_t  timerSetCaptureCompareOutputActiveLevel(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel,
                                                                 TIMER_CAPCOMP_PIN_ACTIVE_LEVEL_t polarity);



extern TIMER_RETURN_CODE_t  timerSetCaptureComparePreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, uint32_t ccrVal);
extern TIMER_RETURN_CODE_t  timerGetCaptureComparePreloadValue(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, uint32_t *ccrVal);
extern TIMER_RETURN_CODE_t  timerSetCaptureComparePreloadCtrlBits(TIM_TypeDef *tim, TIMER_CCPC_MODE_t ccpcMode);
extern TIMER_RETURN_CODE_t  timerUpdateCaptureCompareCtrlBits(TIM_TypeDef *tim, TIMER_CAPCOMP_CTRL_t update);
extern TIMER_RETURN_CODE_t  timerSetCaptureCompareIdleStateOutput(TIM_TypeDef *tim, TIMER_IDLE_STATE_OUTPUT_t output,
                                                           TIMER_IDLE_STATE_t state);
extern TIMER_RETURN_CODE_t  timerEnableCaptureCompareTriggerGeneration(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerDisableCaptureCompareTriggerGeneration(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerEnableCaptureCompareBreakGeneration(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerDisableCaptureCompareBreakGeneration(TIM_TypeDef *tim);

// Input capture functions are coded in mcalTimer/inputCapture.c
extern TIMER_RETURN_CODE_t  timerSetInputCaptureMode(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_CHANNEL_MAPPING_t type);
extern TIMER_RETURN_CODE_t  timerSelectInputCaptureTriggerInput(TIM_TypeDef *tim, TRIGGER_INPUT_t input);
extern TIMER_RETURN_CODE_t  timerSetInputCaptureSamplingClkRatio(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_RATIO_t ratio);
extern TIMER_RETURN_CODE_t  timerGetInputCaptureSamplingClkRatio(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_RATIO_t *ratio);
extern TIMER_RETURN_CODE_t  timerSetInputCapturePrescaler(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, TIMER_IC_PRESCALER_t psc);
extern TIMER_RETURN_CODE_t  timerGetInputCapturePrescaler(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, TIMER_IC_PRESCALER_t *psc);
extern TIMER_RETURN_CODE_t  timerSetInputCaptureFilter(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel, TIMER_IC_FILTER_t filter);

// Output compare functions are coded in mcalTimer/outputCompare.c
extern TIMER_RETURN_CODE_t  timerSetOutputCompareMode(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn, TIMER_OUTPUT_COMPARE_MODE_t mode);
extern TIMER_RETURN_CODE_t  timerEnableOutputComparePreload(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);
extern TIMER_RETURN_CODE_t  timerDisableOutputComparePreload(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);
extern TIMER_RETURN_CODE_t  timerSelectOutputCompareCenterAlignMode(TIM_TypeDef *tim, TIMER_CENTER_ALIGN_MODE_t align);
extern TIMER_RETURN_CODE_t  timerEnableOutputCompareFastCompare(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel);
extern TIMER_RETURN_CODE_t  timerDisableOutputCompareFastCompare(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel);
extern TIMER_RETURN_CODE_t  timerEnableOutputCompareClearOutput(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel);
extern TIMER_RETURN_CODE_t  timerDisableOutputCompareClearOutput(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t channel);

// Function implementations are coded in file mcalTimer/dma.c
extern TIMER_RETURN_CODE_t  timerEnableDMA(TIM_TypeDef *tim, TIMER_DMA_t dma);
extern TIMER_RETURN_CODE_t  timerDisableDMA(TIM_TypeDef *tim, TIMER_DMA_t dma);
extern TIMER_RETURN_CODE_t  timerSelectDmaTriggerOutput(TIM_TypeDef *tim, TRIGGER_OUTPUT_t output);
extern TIMER_RETURN_CODE_t  timerSelectDMATriggerMode(TIM_TypeDef *tim, TIMER_DMA_TRIGGER_MODE_t mode);
extern TIMER_RETURN_CODE_t  timerSetDmaBurstStartingRegister(TIM_TypeDef *tim, TIMER_DMA_BURST_STARTING_REGISTER_t reg);
extern TIMER_RETURN_CODE_t  timerSetDmaBurstLength(TIM_TypeDef *tim, uint8_t len);

// Function implementations are coded in file mcalTimer/events.c
extern TIMER_RETURN_CODE_t  timerEnableCaptureCompareUpdateEvent(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerDisableCaptureCompareUpdateEvent(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerGenerateCaptureCompareUpdateEvent(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerEnableCaptureCompareEventGeneration(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);
extern TIMER_RETURN_CODE_t  timerDisableCaptureCompareEventGeneration(TIM_TypeDef *tim, TIMER_CHANNEL_NUM_t chn);


// Function implementations are coded in file mcalTimer/interrupt.c
extern TIMER_RETURN_CODE_t  timerEnableInterrupt(TIM_TypeDef *tim, TIMER_IRQ_t irq);
extern TIMER_RETURN_CODE_t  timerDisableInterrupt(TIM_TypeDef *tim, TIMER_IRQ_t irq);
extern TIMER_RETURN_CODE_t  timerClearInterruptFlag(TIM_TypeDef *tim, TIMER_IRQ_t irq);


// Function implementations are coded in file mcalTimer/masterSlave.c
extern TIMER_RETURN_CODE_t  timerSelectMasterSlaveTriggerAction(TIM_TypeDef *tim, TIMER_TRIGGER_ACTION_t action);
extern TIMER_RETURN_CODE_t  timerSelectMasterSlaveMode(TIM_TypeDef *tim, TIMER_MSM_MODE_t msm);
extern TIMER_RETURN_CODE_t  timerSelectMasterSlaveTriggerSyncInput(TIM_TypeDef *tim, TIMER_TRIGGER_t trigger);
extern TIMER_RETURN_CODE_t  timerSetMasterSlaveExternalTriggerPolarity(TIM_TypeDef *tim, TIMER_EXT_TRIG_POL_t polarity);
extern TIMER_RETURN_CODE_t  timerEnableMasterSlaveExternalTriggerClock(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerDisableMasterSlaveExternalTriggerClock(TIM_TypeDef *tim);
extern TIMER_RETURN_CODE_t  timerSelectMasterSlaveExternalTriggerPrescaler(TIM_TypeDef *tim, TIMER_EXT_TRIG_PSC extPsc);
extern TIMER_RETURN_CODE_t  timerSelectMasterSlaveExternalTriggerFilter(TIM_TypeDef *tim, TIMER_EXT_TRIG_FILTER_t filter);

#ifdef __cplusplus
}
#endif


#endif /* MCALTIMER_H_ */
