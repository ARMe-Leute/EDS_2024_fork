/**
 * @file mcalRCC.h
 *
 *  Created on: 19.06.2021
 *      Author: Ralf
 */

#ifndef MCALRCC_H_
#define MCALRCC_H_

#include <stm32f4xx.h>
#include <system_stm32f4xx.h>
#include <mcalGPIO.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup rcc4
 * @{
 */

#define HSI_VALUE       ((uint32_t) 16000000ul)
#define HSE_VALUE       ((uint32_t)  8000000ul) // Available on the ST-Link part of the NUCLEO board

typedef enum
{
    SYSCLKSRC_HSI,
    SYSCLKSRC_HSE,
    SYSCLKSRC_PLLP,
    SYSCLKSRC_PLLR
} SYSCLK_SRC_t;

typedef enum
{
    PLL_SRC_HSI,
    PLL_SRC_HSE
} PLL_CLOCK_SRC_t;

typedef enum
{
    MCO1,
    MCO2
} MCO_TypeDef_t;

typedef enum
{
    SYSCLK,
    PLL_I2S,
    HSE,
    PLL_MAIN,
    HSI          = 0,
    LSE          = 1
} MCO_CLK_SRC_t;

typedef enum
{
    APB1,
    APB2
} APB_TypeDef_t;

typedef enum
{
    SYSCLK_DIV_1,
    SYSCLK_DIV_2,
    SYSCLK_DIV_4,
    SYSCLK_DIV_8,
    SYSCLK_DIV_16,
    SYSCLK_DIV_64,
    SYSCLK_DIV_128,
    SYSCLK_DIV_256,
    SYSCLK_DIV_512
} AHB_DIVIDER_t;

typedef enum
{
    AHB_DIV_1,
    AHB_DIV_2,
    AHB_DIV_4,
    AHB_DIV_8,
    AHB_DIV_16
} APB_DIVIDER_t;

/**
 * @}
 */
extern void     rccEnableHSE(void);
extern void     rccEnableHSI(void);
extern void     rccEnableMainPLL(void);
extern void     rccDisableMainPLL(void);
extern void     rccEnableSAIPLL(void);
extern void     rccDisableSAIPLL(void);
extern void     rccEnableI2SPLL(void);
extern void     rccDisableI2SPLL(void);

extern void     rccEnableClockSecuritySystem(void);
extern void     rccDisableClockSecuritySystem(void);
extern void     rccSetHSITrimmingValue(uint8_t trim);
extern uint8_t  rccGetHSICalibrationValue(void);

extern void     rccSelectPLLClockSource(PLL_CLOCK_SRC_t clkSrc);
extern void     rccSelectSysclkSrc(SYSCLK_SRC_t src);
extern void     rccSetSysclkFreq(uint8_t fOsc, uint16_t freq);
extern void     rccAssignClk2MCO(MCO_CLK_SRC_t clkSrc, MCO_TypeDef_t mco);
extern void     rccSetAHBPrescaler(AHB_DIVIDER_t div);
extern void     rccSetAPBPrescaler(APB_DIVIDER_t div);
extern void     rccSetAPB1Prescaler(APB_DIVIDER_t div);
extern void     rccSetAPB2Prescaler(APB_DIVIDER_t div);

extern uint32_t rccGetSysClock(void);
extern uint32_t rccGetHclkFreq(void);
extern uint32_t rccGetPclk1Freq(void);
extern uint32_t rccGetPclk2Freq(void);


#ifdef __cplusplus
}
#endif

#endif /* MCALRCC_H_ */
