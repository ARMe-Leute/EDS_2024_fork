/**
 * typedefs.h
 *
 *  Created on: Mar 15, 2021
 *      Author: Ralf Jesse
 */

#ifndef MCALTIMER_TYPEDEFS_H_
#define MCALTIMER_TYPEDEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup tim11
 * @{
 */

#define MAX_NUM_TIMER_REGISTERS              (18)       // Needed for timer DMA burst transfer(s)

typedef enum
{
    TIMER_OK                                  =   0,
    TIMER_INVALID_TIMER                       = -40,
    TIMER_INVALID_CHANNEL                     = -41,
    TIMER_INVALID_CHANNEL_TYPE                = -42,
    TIMER_INVALID_IRQ_TYPE                    = -43,
    TIMER_CAPCOMP_NOT_SUPPORTED               = -44,
    TIMER_REPETITION_NOT_SUPPORTED            = -45,
    TIMER_BDTR_NOT_SUPPORTED                  = -46,
    TIMER_INVALID_DMA_ASSIGNMENT              = -47,
    TIMER_ONE_SHOT_NOT_SUPPORTED              = -48,
    TIMER_INVALID_TIMER_CHANNEL_ASSIGNMENT    = -49,
    TIMER_INVALID_MASTER_SLAVE_MODE_TYPE      = -50,
    TIMER_INVALID_TRIGGER                     = -51,
    TIMER_INVALID_TRIGGER_ACTION              = -52,
    TIMER_INVALID_EXT_TRIGGER_FILTER          = -53,
    TIMER_INVALID_CLOCK_DIVIDER               = -54,
    TIMER_CLOCK_DIVISION_NOT_SUPPORTED        = -55,
    TIMER_INVALID_REGISTER_OFFSET             = -56,
    TIMER_INVALID_DMA_BURST_STARTING_REGISTER = -57,
    TIMER_INVALID_DMA_BURST_LENGTH            = -58
} TIMER_RETURN_CODE_t;

/* Clock divider for sampling frquency, Bits 8 + 9 */
typedef enum
{
    CKD_RATIO_IS_1 = 0,
    CKD_RATIO_IS_2,
    CKD_RATIO_IS_4
} TIMER_SAMPLING_RATIO_t;

/* Auto-Reload preload enable, Bit 7  */
typedef enum
{
    ARPE_BUFFER_OFF  = 0,
    ARPE_BUFFER_ON
} TIMER_ARPE_BUF_t;

/* PWM: Edge or center-aligned mode, Bits 6 + 5 */
typedef enum
{
    PWM_EDGE     = 0,
    PWM_CENTER_1,
    PWM_CENTER_2,
    PWM_CENTER_3,
} TIMER_PWM_MODE_t;

/* DIR (counting direction), Bit 4 */
typedef enum
{
    COUNTUP = 0,
    COUNTDOWN
} TIMER_COUNT_DIR_t;

/* OPM: Single-shot mode */
typedef enum
{
    ONE_PULSE_MODE_OFF = 0,
    ONE_PULSE_MODE_ON
} TIMER_OPM_MODE_t;

/* URS: Defines the source of an update request */
typedef enum
{
    UPDATE_ALL_SOURCES = 0,        // Generates an event on underflow/overflow,
                                   // when setting the UG bit, or update generation
                                   // requested by the slave controller
    UPDATE_OVERFLOW_UNDERFLOW      // Generates an event only on underflow/overflow
} TIMER_URS_t;

/* UDIS: Update disable */
typedef enum
{
    UPDATE_ENABLE = 0,
    UPDATE_DISABLE
} TIMER_UDIS_t;

/* CEN: Enable/disable timer */
typedef enum
{
    START_TIMER = 0,
    STOP_TIMER
} TIMER_STARTSTOP_t;

/* Timer channels */
typedef enum
{
    TIMIO_CH1 = 1,
    TIMIO_CH2,
    TIMIO_CH3,
    TIMIO_CH4
} TIMER_CHANNEL_NUM_t;

/* Select between input or output channel */
typedef enum
{
    TIMIO_OUTPUT = 0,
    TIMIO_INPUT
} TIMER_CHANNELTYPE_t;

//typedef enum
//{
//    MAP_CHN_TO_SAME_TIMER       = 0,
//    MAP_CHN_TO_OPPOSITE_TIMER,
//    MAP_CHN_TO_TRC
//} TIMER_INPUT_CHN_MAPPING_t;

typedef enum
{
    MAP_TO_OUTPUT               = 0,
    MAP_CHN_TO_SAME_TIMER,
    MAP_CHN_TO_OPPOSITE_TIMER,
    MAP_CHN_TO_TRC
} TIMER_CHANNEL_MAPPING_t;

/* Capture/Compare operational modes */
typedef enum
{
    CHN_FROZEN      = 0,    // No activity on capture/compare match
    CHN_ACTIVE_ON_MATCH,    // All channels: Set high on match
    CHN_INACTIVE_ON_MATCH,  // All channels: Set low on match
    CHN_TOGGLE_ON_MATCH,    // All channels: Toggle on match
    CHN_FORCE_LOW,          // All channels: Force output to low level
    CHN_FORCE_HIGH,         // All channels: Force output to high level
    CHN_PWM_MODE_1,         // All channels: Set output high while upcounting, else low
    CHN_PWM_MODE_2          // All channels: Set output low while upcounting, else high
} TIMER_OUTPUT_COMPARE_MODE_t;

typedef enum
{
    TIM_IRQ_UIE             = 0,
    TIM_CAPCOMP_IRQ_CH1,
    TIM_CAPCOMP_IRQ_CH2,
    TIM_CAPCOMP_IRQ_CH3,
    TIM_CAPCOMP_IRQ_CH4,
    TIM_TRG_IRQ,
    TIM_COM_IRQ,
    TIM_BREAK_IRQ,
    TIM_OVERCAPTURE_1,
    TIM_OVERCAPTURE_2,
    TIM_OVERCAPTURE_3,
    TIM_OVERCAPTURE_4
} TIMER_IRQ_t;

typedef enum
{
    TIM_UPDATE_DMA          = 0,
    TIM_CAPCOMP_DMA_CH1,
    TIM_CAPCOMP_DMA_CH2,
    TIM_CAPCOMP_DMA_CH3,
    TIM_CAPCOMP_DMA_CH4,
    TIM_COM_DMA,
    TIM_TRG_DMA
} TIMER_DMA_t;

typedef enum
{
    COMPL_OFF = 0,
    COMPL_ON
} TIMER_COMPLEMENT_t;

typedef enum
{
    CHN_ENABLE         = 1,
    CHN_POLARITY       = 2,
    CHN_COMPL_ENABLE   = 4,
    CHN_COMPL_POLARITY = 8
} TIMER_CAPCOMP_t;

typedef enum
{
    MOE_OFF = 0,
    MOE_ON
} TIMER_BDTR_MOE_t;

typedef enum
{
    AOE_SW_ONLY = 0,
    AOE_SW_AND_EVENT
} TIMER_BDTR_AOE_t;

typedef enum
{
    BKP_ACT_HIGH = 0,
    BKP_ACT_LOW
} TIMER_BDTR_BKP_POLARITY_t;

typedef enum
{
    BRK_ENABLE = 0,
    BRK_DISABLE
} TIMER_BDTR_BKE_t;

typedef enum
{
    CKD_1,
    CKD_2,
    CKD_4
} TIMER_RATIO_t;

typedef enum
{
    NORMAL,
    DELAYED
} TIMER_TRIGGER_ACTION_t;

typedef enum
{
    MSM_DISABLED_MODE,
    MSM_ENCODER_1_MODE,
    MSM_ENCODER_2_MODE,
    MSM_ENCODER_3_MODE,
    MSM_RESET_MODE,
    MSM_GATED_MODE,
    MSM_TRIGGER_MODE,
    MSM_EXT_CLK_MODE
} TIMER_MSM_MODE_t;

typedef enum
{
    TRIGGER_0,
    TRIGGER_1,
    TRIGGER_2,
    TRIGGER_3,
    TRIGGER_TIM1_EDGE_DETECTION,
    TRIGGER_TIM1_FILTERED_INPUT_1,
    TRIGGER_TIM1_FILTERED_INPUT_2,
    TRIGGER_EXTERNAL
} TIMER_TRIGGER_t;

typedef enum
{
    RESET_TRIGGER_OUTPUT,
    CEN_IS_TRIGGER_OUTPUT,
    UPDATE_IS_TRIGGER_OUTPUT,
    SEND_COMPARE_PULSE_ON_TRG_OUTPUT,
    OC1REF_IS_TRIGGER_OUTPUT,
    OC2REF_IS_TRIGGER_OUTPUT,
    OC3REF_IS_TRIGGER_OUTPUT,
    OC4REF_IS_TRIGGER_OUTPUT
} TRIGGER_OUTPUT_t;

typedef enum
{
    SEND_ON_CCX_MATCH,
    SEND_ON_UPDT_EVENT
} TIMER_DMA_TRIGGER_MODE_t;

typedef enum
{
    ASSIGN_TIMX_CH1,
    ASSIGN_XORED_TIMX_CH1_2_3
} TRIGGER_INPUT_t;

typedef enum
{
    TIM_CLOCK_X1,
    TIM_CLOCK_X2,
    TIM_CLOCK_X4
} TIMER_CLOCK_DIVISION_t;

typedef enum
{
    EDGE_ALIGNMENT,
    CENTER_ALIGN_1,         // Countup/Countdown. Sets interrupt flags only when counting down
    CENTER_ALIGN_2,         // Countup/Countdown. Sets interrupt flags only when counting up
    CENTER_ALIGN_3          // Countup/Countdown. Sets interrupt flags when counting up AND down
} TIMER_CENTER_ALIGN_MODE_t;

typedef enum
{
    ACTIVE_HIGH_OR_RISING_EDGE,
    ACTIVE_LOW_OR_FALLING_EDGE
} TIMER_EXT_TRIG_POL_t;

typedef enum
{
    EXTTRIG_FILTER_OFF,
    EXTTRIG_FCK_INT_2,
    EXTTRIG_FCK_INT_4,
    EXTTRIG_FCK_INT_8,
    EXTTRIG_FDTS_DIV_2_N6,
    EXTTRIG_FDTS_DIV_2_N8,
    EXTTRIG_FDTS_DIV_4_N6,
    EXTTRIG_FDTS_DIV_4_N8,
    EXTTRIG_FDTS_DIV_8_N6,
    EXTTRIG_FDTS_DIV_8_N8,
    EXTTRIG_FDTS_DIV_16_N5,
    EXTTRIG_FDTS_DIV_16_N6,
    EXTTRIG_FDTS_DIV_16_N8,
    EXTTRIG_FDTS_DIV_32_N5,
    EXTTRIG_FDTS_DIV_32_N6,
    EXTTRIG_FDTS_DIV_32_N8
} TIMER_EXT_TRIG_FILTER_t;

typedef enum
{
    NO_FILTER,              // FCKINT = internal clock
    FCKINT_N2,
    FCKINT_N4,
    FCKINT_N8,
    FTDS_DIV_2_N6,          // FDTS = Is the clock frequency for all filters. It is derived from FCKINT
    FTDS_DIV_2_N8,          // Nx   = Number of conscutive events that are needed to validate a transition on the outpus
    FTDS_DIV_4_N6,
    FTDS_DIV_4_N8,
    FTDS_DIV_8_N6,
    FTDS_DIV_8_N8,
    FTDS_DIV_16_N5,
    FTDS_DIV_16_N8,
    FTDS_DIV_16_N6,
    FTDS_DIV_32_N5,
    FTDS_DIV_32_N8,
    FTDS_DIV_32_N6
} TIMER_IC_FILTER_t;        // IC = Input Capture

typedef enum
{
    PSC_OFF,
    PSC_DIV_2,
    PSC_DIV_4,
    PSC_DIV_8
} TIMER_EXT_TRIG_PSC;

typedef enum
{
    CCPC_OFF,
    CCPC_ON
} TIMER_CCPC_MODE_t;

typedef enum
{
    COMG_ONLY,
    COMG_AND_RISING_EDGE
} TIMER_CAPCOMP_CTRL_t;

typedef enum
{
    IDLE_STATE_OUT_1,
    IDLE_STATE_OUT_1N,
    IDLE_STATE_OUT_2,
    IDLE_STATE_OUT_2N,
    IDLE_STATE_OUT_3,
    IDLE_STATE_OUT_3N,
    IDLE_STATE_OUT_4
} TIMER_IDLE_STATE_OUTPUT_t;

typedef enum
{
    IDLE_STATE_OFF,
    IDLE_STATE_ON
} TIMER_IDLE_STATE_t;

typedef enum
{
    CAPCOMP_EVENT_OFF,
    CAPCOMP_EVENT_ON
} TIMER_CAPCOMP_EVENT_t;

typedef enum
{
    CAPTURE_ALL,
    CAPTURE_EVERY_2ND,
    CAPTURE_EVERY_4TH,
    CAPTURE_EVERY_8TH
} TIMER_IC_PRESCALER_t;

typedef enum
{
    ACTIVE_LOW,
    ACTIVE_HIGH
} TIMER_CAPCOMP_PIN_ACTIVE_LEVEL_t;

typedef enum
{
    TIMX_CR1,
    TIMX_CR2,
    TIMX_SMCR,
    TIMX_DIER,
    TIMX_SR,
    TIMX_EGR,
    TIMX_CCMR1,
    TIMX_CCMR2,
    TIMX_CCER,
    TIMX_CNT,
    TIMX_PSC,
    TIMX_ARR,
    TIMX_RCR,
    TIMX_CCR1,
    TIMX_CCR2,
    TIMX_CCR3,
    TIMX_CCR4,
    TIMX_BDTR,
    TIMX_DCR,
    TIMX_DMAR,
    TIMX_OR
} TIMER_DMA_BURST_STARTING_REGISTER_t;

typedef enum
{
    TRANSF_X01,
    TRANSF_X02,
    TRANSF_X03,
    TRANSF_X04,
    TRANSF_X05,
    TRANSF_X06,
    TRANSF_X07,
    TRANSF_X08,
    TRANSF_X09,
    TRANSF_X10,
    TRANSF_X11,
    TRANSF_X12,
    TRANSF_X13,
    TRANSF_X14,
    TRANSF_X15,
    TRANSF_X16,
    TRANSF_X17,
    TRANSF_X18
} TIMER_DMA_BURST_LEN_t;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif /* MCALTIMER_TYPEDEFS_H_ */
