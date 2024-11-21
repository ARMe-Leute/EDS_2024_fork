/**
 * @defgroup rcc  Reset and Clock Control (mcalRCC.h/mcalRCC.c)
 * @defgroup rcc0 Deprecated System Functions
 * @ingroup  rcc
 * @defgroup rcc1 System Macros
 * @ingroup  rcc
 * @defgroup rcc2 System Standard Functions
 * @ingroup  rcc
 * @defgroup rcc3 Advanced System Functions
 * @ingroup  rcc
 * @defgroup rcc4 RCC Enumerations and Definitions
 * @ingroup  rcc
 */

/**
 * @mainpage
 * MCAL-STM ist eine Sammlung von Bare-Metal-Funktionen, die die Programmierung von Mikrocontrollern der STM32F4xx-
 * Familie erleichtern soll.
 *
 * MCAL-STM is a collection of bare-metal functions which shall help to simplify the programming of the STM32F4xx
 * microcontroller family.
 *
 * @file    mcalRCC.c (Genuine name was : stm32f4xx_hal_rcc.c)
 * @author  Ralf Jesse
 * @version V0.1
 * @date    19-June-2021
 *
 * @details
 * RCC-Treiber (RCC = Reset and Clock Control)
 * ===========================================
 * Die Funktionen in **mcalRCC.c** werden in Zusammenhang mit der Konfiguration der verschiedenen Systemtakte verwendet.
 *
 * RCC module driver
 * =================
 * This file provides firmware functions to manage the following functionalities of the Reset and Clock Control (RCC)
 * peripheral:
 *  + Initialization and de-initialization functions
 *  + Peripheral Control functions
 */

#include <stdint.h>
#include <stdbool.h>

#include "mcalRCC.h"

#define __HAL_RCC_GET_PLL_OSCSOURCE() ((uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC))

/* Private variables ---------------------------------------------------------*/
const uint8_t APBAHBPrescTable[16] =
{
    0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9
};

/**********************************************************
 * Deprecated functions                                   *
 *********************************************************/

 /**
  * @deprecated
  * @ingroup rcc0
  * Dient zur Einstellung der Taktfrequenz des APB2-Busses. Die Taktfrequenz wird vom Systemtakt (SYSCLK) abgeleitet.
  *
  * @deprecated
  * This function can be used for the setting of the APB2 clock rate. It is derived from the system clock (SYSCLK).
  *
  * @param  div : Teiler, durch den die SYSCLK-Frequenz geteilt wird / This is the divider by which AHB clock is divided.
  *
  * @note
  * Nicht mehr verwenden. Diese Funktion wurde durch **rccSetAPB1Prescaler() und rccSetAPB2Prescaler() ersetzt.
  *
  * @note
  * Don't use this funktion any longer. It has been replaced with rccSetAPB1Prescaler() and rccSetAPB2Prescaler().
  *
  * @par
  * Diese Funktion diente früher zur gemeinsamen Einstellung der APB1- und APB2-Taktfrequenz, wobei die APB1-Frequenz
  * unmittelbar und unveränderlich auf 50% der APB2-Taktrate eingestellt wurde.
  *
  * @par
  * This single function was formerly used to set the prescaler of both, the APB1 and APB2 clock rate with the APB1 clock
  * rate implicitly set to 50% of the APB2 clock rate. However, this is a limitation that is not useful for every
  * application. It makes the usage of peripheral components much more flexible when setting the APB1 and APB2 prescaler
  * separately.
  *
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
  *          <td>CFGR</td>
  *          <td rowspan="1">PPRE1/PPRE2</td>
  *          <td rowspan="1">12...10/15...13</td>
  *          <td rowspan="1">Selects the prescaler for APB2. APB1 is then set to 50% of APB2.</td>
  *      </tr>
  * </table>
  */
 void rccSetAPBPrescaler(APB_DIVIDER_t div)
 {
     RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
     switch (div)
     {
         case AHB_DIV_1:
             RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
             RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
             break;

         case AHB_DIV_2:
             RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
             RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
             break;

         case AHB_DIV_4:
             RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;
             RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;
             break;

         case AHB_DIV_8:
             RCC->CFGR |= RCC_CFGR_PPRE2_DIV8;
             RCC->CFGR |= RCC_CFGR_PPRE1_DIV16;
             break;

         case AHB_DIV_16:
             RCC->CFGR |= RCC_CFGR_PPRE2_DIV16;
             RCC->CFGR |= RCC_CFGR_PPRE1_DIV16;      // The maximum divider is 16!
             break;
     }
 }


/**
 * @ingroup rcc2
 * @brief This function is almost identical to the HAL_RCC_GetSysClockFreq() function.
 *
 * @return System frequency
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CFGR</td>
 *          <td rowspan="1">SWS</td>
 *          <td rowspan="1">3...2</td>
 *          <td rowspan="1">Gets the source of the system clock</td>
 *      </tr>
 *      <tr>
 *          <td>PLLCFGR</td>
 *          <td rowspan="1">PLLM, PLLP</td>
 *          <td rowspan="1">5...0 (PLLM), 17...16 (PLLP)</td>
 *          <td rowspan="1">Values used to calculate the sysclock frequency</td>
 *      </tr>
 * </table>
 */
uint32_t rccGetSysClock(void)
{
    uint32_t pllm = 0u;
    uint32_t pllvco = 0u;
    uint32_t pllp = 0u;
    uint32_t sysClock = 0u;

    switch (RCC->CFGR & RCC_CFGR_SWS)
    {
        case RCC_CFGR_SWS_HSI:
            sysClock = HSI_VALUE;    // High-speed internal = 16MHz
            break;

        case RCC_CFGR_SWS_HSE:
            sysClock = HSE_VALUE;    // High-speed external = 4 - 25MHz (max.)
            break;

        case RCC_CFGR_SWS_PLL:
            pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

            if (__HAL_RCC_GET_PLL_OSCSOURCE() != RCC_PLLCFGR_PLLSRC_HSI)
            {
                /* HSE used as PLL clock source */
                pllvco = (uint32_t) ((((uint64_t) HSE_VALUE
                    * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t) pllm);
            }
            else
            {
                /* HSI used as PLL clock source */
                pllvco = (uint32_t) ((((uint64_t) HSI_VALUE
                    * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t) pllm);
            }
            pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) * 2U);

            sysClock = pllvco / pllp;
            break;

        default:
            sysClock = HSI_VALUE;
            break;
    }

    return sysClock;
}

/**
 * @ingroup rcc2
 * @brief  Returns the HCLK frequency
 * @note   Each time HCLK changes, this function must be called to update the
 *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
 *
 * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
 *         and updated within this function
 * @retval HCLK frequency
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * Uses internal function rccGetSysClock().
 */
uint32_t rccGetHclkFreq(void)
{
    SystemCoreClock = rccGetSysClock() >> APBAHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> POSITION_VAL(RCC_CFGR_HPRE)];
    return SystemCoreClock;
}

/**
 * @ingroup rcc2
 * @brief  Returns the PCLK1 frequency which is provided to the peripheral components
 *         which are attached to the APB1 peripheral bus.
 *
 * @note   Each time PCLK1 changes, this function must be called to update the
 *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
 * @retval PCLK1 frequency
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * Uses internal function rccGetHclkFrequency().
 */
uint32_t rccGetPclk1Freq(void)
{
    /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
    return (rccGetHclkFreq() >> APBAHBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> POSITION_VAL(RCC_CFGR_PPRE1)]);
}

/**
 * @ingroup rcc2
 * @brief  Returns the PCLK2 frequency which is provided to the peripheral components
 *         which are attached to the APB2 peripheral bus.
 *
 * @note   Each time PCLK2 changes, this function must be called to update the
 *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
 * @retval PCLK2 frequency
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * Uses internal function rccGetHclkFrequency().
 */
uint32_t rccGetPclk2Freq(void)
{
    /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
    return (rccGetHclkFreq() >> APBAHBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> POSITION_VAL(RCC_CFGR_PPRE2)]);
}

//----- New functions ---

/**
 * @ingroup rcc2
 * Enables the HSE (High-speed external) as clock source. Implicitely disables HSI.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">HSE ON</td>
 *          <td rowspan="1">16</td>
 *          <td rowspan="1">Turns HSE on</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">HSE RDY</td>
 *          <td rowspan="1">17</td>
 *          <td rowspan="1">Waits until HSE RDY is '1'</td>
 *      </tr>
 * </table>
 */
void rccEnableHSE(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
    {
        // Wait until HSE is ready and stable.
    }
    RCC->CR &= ~(RCC_CR_HSIRDY_Msk | RCC_CR_HSION_Msk); // Disable HSI
}

/**
 * @ingroup rcc2
 * Enables the HSI (High-speed internal) as clock source. Implicetely disables HSE.
 * <br>

 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *          <th>Comment</th>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">HSE ON</td>
 *          <td rowspan="1">16</td>
 *          <td rowspan="1">Resets the HSE ON bit</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">HSI ON</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Sets HSI ON to '1'</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">HSI RDY</td>
 *          <td rowspan="1">2</td>
 *          <td rowspan="1">Waits until HSI RDY is '1'</td>
 *      </tr>
 * </table>
 */
void rccEnableHSI(void)
{
    RCC->CR &= ~(RCC_CR_HSEBYP_Msk | RCC_CR_HSERDY_Msk | RCC_CR_HSEON_Msk); // Disable HSE
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
        // Wait until HSE is ready and stable.
    }
}

/**
 * @ingroup rcc2
 * Enables the main PLL.
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
 *          <td>CR</td>
 *          <td rowspan="1">PLL ON</td>
 *          <td rowspan="1">24</td>
 *          <td rowspan="1">Turns the PLL ON bit on</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">PLL RDY</td>
 *          <td rowspan="1">1</td>
 *          <td rowspan="1">Wait until PLL RDY = '1'</td>
 *      </tr>
 * </table>
 */
void rccEnableMainPLL(void)
{
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
    {
        // Wait until main PLL is ready and stable.
    }
}

/**
 * @ingroup rcc2
 * Disables the main PLL.
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
 *          <td>CR</td>
 *          <td rowspan="1">PLL ON</td>
 *          <td rowspan="1">24</td>
 *          <td rowspan="1">Resets the PLL ON bit</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">PLL RDY</td>
 *          <td rowspan="1">25</td>
 *          <td rowspan="1">Waits until the PLL RDY bit = '0'</td>
 *      </tr>
 * </table>
 */
void rccDisableMainPLL(void)
{
    RCC->CR &= ~RCC_CR_PLLON_Msk;
    while (RCC->CR & RCC_CR_PLLRDY)
    {
        // Wait until main PLL is down.
    }
}

/**
 * @ingroup rcc2
 * Enables the SAI PLL.
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
 *          <td>CR</td>
 *          <td rowspan="1">PLLSAI ON</td>
 *          <td rowspan="1">28</td>
 *          <td rowspan="1">Sets the PLL ON bit</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">PLLSAI RDY</td>
 *          <td rowspan="1">29</td>
 *          <td rowspan="1">Waits until the PLLSAI RDY bit = '1'</td>
 *      </tr>
 * </table>
 */

/* dissabled for STM32F40x
void rccEnableSAIPLL(void)
{
    RCC->CR |= RCC_CR_PLLSAION;
    while (!(RCC->CR & RCC_CR_PLLSAIRDY))
    {
        // Wait until PLLSAI is ready and stable.
    }
}
*/
/**
 * @ingroup rcc2
 * Disables the SAI PLL,
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
 *          <td>CR</td>
 *          <td rowspan="1">PLLSAI ON</td>
 *          <td rowspan="1">28</td>
 *          <td rowspan="1">Resets the PLLSAI ON bit</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">PLLSAI RDY</td>
 *          <td rowspan="1">29</td>
 *          <td rowspan="1">Waits until the PLLSAI RDY bit = '0'</td>
 *      </tr>
 * </table>
 */
/* dissabled for STM32F40x
void rccDisableSAIPLL(void)
{
    RCC->CR &= ~RCC_CR_PLLSAIRDY_Msk;
    while (RCC->CR & RCC_CR_PLLSAIRDY)
    {
        // Wait until SAI PLL is down.
    }
}
*/
/**
 * @ingroup rcc2
 * Enables the I2S PLL.
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
 *          <td>CR</td>
 *          <td rowspan="1">PLLI2S ON</td>
 *          <td rowspan="1">26</td>
 *          <td rowspan="1">Sets the PLLI2S ON bit</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">PLLI2S RDY</td>
 *          <td rowspan="1">27</td>
 *          <td rowspan="1">Waits until the PLLSAI RDY bit = '1'</td>
 *      </tr>
 * </table>
 */
void rccEnableI2SPLL(void)
{
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY))
    {
        // Wait until I2S PLL is ready and stable.
    }
}

/**
 * @ingroup rcc2
 * Disbales the I2S PLL.
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
 *          <td>CR</td>
 *          <td rowspan="1">PLLI2S ON</td>
 *          <td rowspan="1">26</td>
 *          <td rowspan="1">Resets the PLLI2S ON bit</td>
 *      </tr>
 *      <tr>
 *          <td>CR</td>
 *          <td rowspan="1">PLLI2S RDY</td>
 *          <td rowspan="1">29</td>
 *          <td rowspan="1">Waits until the PLLI2S RDY bit = '0'</td>
 *      </tr>
 * </table>
 */
void rccDisableI2SPLL(void)
{
    RCC->CR &= ~RCC_CR_PLLI2SON_Msk;
    while (RCC->CR & RCC_CR_PLLI2SRDY)
    {
        // Wait until I2S PLL is down.
    }
}

/**
 * @ingroup rcc2
 * Enables the clock security system CSS.
 *
 * If this bit is set to '1' the CSS is activated by hardware
 * after the HSE became stable (HSERDY = 1).
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
 *          <td>CR</td>
 *          <td rowspan="1">CSSON</td>
 *          <td rowspan="1">19</td>
 *          <td rowspan="1">Enables clock security</td>
 *      </tr>
 * </table>
 */
void rccEnableClockSecuritySystem(void)
{
    RCC->CR |= RCC_CR_CSSON;
}

/**
 * @ingroup rcc2
 * Disables the clock security system CSS.
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
 *          <td>CR</td>
 *          <td rowspan="1">CSSON</td>
 *          <td rowspan="1">19</td>
 *          <td rowspan="1">Disables the clock security</td>
 *      </tr>
 * </table>
 */
void rccDisableClockSecuritySystem(void)
{
    RCC->CR &= ~RCC_CR_CSSON_Msk;
}

/**
 * @ingroup rcc3
 * Allows trimming of the of the internal HSI clock when
 * the clock frequency varies because of temperature or
 * voltage deviation.
 *
 * @param  trim : Trimming value [0 ... 31]
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
 *          <td>CR</td>
 *          <td rowspan="1">HSITRIM</td>
 *          <td rowspan="1">7...3</td>
 *          <td rowspan="1">Used to calibrate the HSI clock</td>
 *      </tr>
 * </table>
 */
void rccSetHSITrimmingValue(uint8_t trim)
{
    if ((trim >= 0) && (trim < 32))
    {
        RCC->CR |= trim << RCC_CR_HSITRIM_Pos;
    }
}

/**
 * @ingroup rcc3
 * Returns the HSI calibration value defined by the manufacturer.
 *
 * @return The calibration value set by the manufacturer.
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
 *          <td>CR</td>
 *          <td rowspan="1">HSICAL ON</td>
 *          <td rowspan="1">15...8</td>
 *          <td rowspan="1">Returns the manufacturer calibration value. Can't be modified.</td>
 *      </tr>
 * </table>
 */
uint8_t rccGetHSICalibrationValue(void)
{
    uint8_t retVal = 0;

    retVal = ((RCC->CR & RCC_CR_HSICAL_7) + (RCC->CR & RCC_CR_HSICAL_6) + (RCC->CR & RCC_CR_HSICAL_5)
        + (RCC->CR & RCC_CR_HSICAL_4) + (RCC->CR & RCC_CR_HSICAL_3) + (RCC->CR & RCC_CR_HSICAL_2)
        + (RCC->CR & RCC_CR_HSICAL_1) + (RCC->CR & RCC_CR_HSICAL_0)) >> RCC_CR_HSICAL_Pos;

    return retVal;
}

/**
 * @ingroup rcc2
 * Selects the clock source which provides the clock to the main PLL and the audio PLL.
 *
 * @param  clkSrc : Clock sources might be PLL_SRC_HSI or PLL_SRC_HSE
 *
 * @note
 * This function is different from rccSelectSysclkSrc() in that way that the parameter defines the
 * <b>input</b> to the main PLL circuit! This function is not required when the main PLL circuit is
 * not used, i.e. if HSI or HSE are selected as MCU clock.
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
 *          <td>PLLCFGR</td>
 *          <td rowspan="1">PLLSRC ON</td>
 *          <td rowspan="1">22</td>
 *          <td rowspan="1">0: HSI used as source for PLL and PLLI2S, 1: HSE used for PLL and PLLI2S</td>
 *      </tr>
 * </table>
 */
void rccSelectPLLClockSource(PLL_CLOCK_SRC_t clkSrc)
{
    if (PLL_SRC_HSE == clkSrc)
    {
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    }
    else if (PLL_SRC_HSI == clkSrc)
    {
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC_HSE_Msk;
    }
}

/**
 * @ingroup rcc2
 * This function selects the source of the SYSCLOCK.
 *
 * @param  sysclkSrc : The clock which is used to set the clock of the MCU.
 *
 * @note
 * This function is different from rccSelectPLLClockSource() in that way that it controls the MCU clock.
 * It can be either SYSCLKSRC_HSI, SYSCLKSRC_HSE, SYSCLKSRC_PLLP, or SYSCLKSRC_PLLR. Consider that
 * parameters 3 and 4 are the <b>output</b> of the main PLL circuit! To setup the MCU clock (SYSCLK) it
 * is mandatory to use at least this function.
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
 *          <td>RCC</td>
 *          <td rowspan="1">CFGR</td>
 *          <td rowspan="1">1...0</td>
 *          <td rowspan="1">Selects the source of the MCU system clock</td>
 *      </tr>
 * </table>
 */
void rccSelectSysclkSrc(SYSCLK_SRC_t sysclkSrc)
{
    switch (sysclkSrc)
    {
        case SYSCLKSRC_HSI:
            RCC->CFGR &= ~RCC_CFGR_SW_Msk;
            break;

        case SYSCLKSRC_HSE:
            RCC->CFGR |= RCC_CFGR_SW_0;
            break;

        case SYSCLKSRC_PLLP:
            RCC->CFGR |= RCC_CFGR_SW_1;
            break;

        case SYSCLKSRC_PLLR:
            RCC->CFGR |= (RCC_CFGR_SW_1 | RCC_CFGR_SW_0);
            break;
    }
}

/**
 * @ingroup rcc2
 * This function calculates the PLLM, PLLN and PLLP settings from the parameters <b>fOsc</b>
 * (oscillator frequency) and <b>mcuFreq</b> (the frequency at which the MCU shall run).
 *
 * @param fOsc    : Frequency of the oscillator used as HSI or HSE
 * @param mcuFreq : The frequency which shall be achieved (must be greater than 12)
 *
 * @note
 * The maximum frequency which is applicable to the members of the STM32F4 family varies
 * depending on the concrete MCU type. If the value of <b>mcuFreq</b> is higher than the
 * maximum frequency of the selected MCU it is automatically corrected to the maximum value
 * which is specified in the datasheet of the device.
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
 *          <td>PLLCFGR</td>
 *          <td rowspan="1">PLLN</td>
 *          <td rowspan="1">14...6</td>
 *          <td rowspan="1"></td>
 *      </tr>
 *      <tr>
 *          <td>PLLCFGR</td>
 *          <td rowspan="1">PLLM</td>
 *          <td rowspan="1">5...0</td>
 *          <td rowspan="1">Calculates PLLM from the desired clock frequency</td>
 *      </tr>
 *      <tr>
 *          <td>PLLCFGR</td>
 *          <td rowspan="1">PLLN</td>
 *          <td rowspan="1">14...6</td>
 *          <td rowspan="1">Calculates PLLN from the desired clock frequency</td>
 *      </tr>
 *      <tr>
 *          <td>PLLCFGR</td>
 *          <td rowspan="1">PLLP</td>
 *          <td rowspan="1">17...16</td>
 *          <td rowspan="1">Calculates PLLP from the desired clock frequency</td>
 *      </tr>
 * </table>
 */
void rccSetSysclkFreq(uint8_t fOsc, uint16_t mcuFreq)
{
    uint16_t plln = 0U;
    uint8_t  pllp = 0U;
    uint8_t  pllm;

    // Sets the maximum frequency if wanted Frequence exceeds the specified frequency of the MCU.
#if defined(STM32F401xx) || defined(STM32F429xx) || defined(STM32F427xx)
    if (mcuFreq > 84)
    {
        mcuFreq = 84;
    }
#endif

#if defined(STM32F410xx) || defined(STM32F411xx) || defined(STM32F412xx) || defined(STM32F413xx) || \
    defined(STM32F423xx)
    if (mcuFreq > 100)
    {
        mcuFreq = 100;
    }
#endif

#if defined(STM32F405xx) || defined(STM32F415xx) || defined (STM32F407xx) || defined (STM32F417xx)
    if (mcuFreq > 168)
    {
        mcuFreq = 168;
    }
#endif

#if defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || \
    defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
    if (fOsc > 180)
    {
        mcuFreq = 180;
    }
#endif

    // fOsc is used as parameter PLLM. Value must be in the range [2 .. 63].
    pllm = fOsc;

    // 432 is the maximum value of the VCO input frequency
    // Valid values for PLLP are { 2, 4, 6, 8 }
    if ((mcuFreq * 8) <= 432)
    {
        pllp = 8;
    }
    else if ((mcuFreq * 6) <= 432)
    {
        pllp = 6;
    }
    else if ((mcuFreq * 4) <= 432)
    {
        pllp = 4;
    }
    else
    {
        pllp = 2;
    }

    plln = mcuFreq * pllp;
    pllp = (pllp / 2) - 1;

    // set only highest bit of pllm and plln as reset
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_5;
    RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLM_Msk | 1 << RCC_PLLCFGR_PLLM_Pos);

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_8;
    RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLN_Msk | 1 << RCC_PLLCFGR_PLLN_Pos);

    // set plln and pllm
    RCC->PLLCFGR |= pllm << RCC_PLLCFGR_PLLM_Pos;
    RCC->PLLCFGR |= plln << RCC_PLLCFGR_PLLN_Pos;
    RCC->PLLCFGR |= pllp << RCC_PLLCFGR_PLLP_Pos;

    // if plln/pllm does not use highest bit (which was set for reset),
    // set highest bit to 0 again
    if (pllm < 0x20)
    {
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_5;
    }
    if (plln < 0x100)
    {
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_8;
    }
}

/**
 * @ingroup rcc2
 * This function assigns the selected clock to either MCO1 (PA8)
 * or MCO2 (PC9) output.
 *
 * @param  clkSrc : Supported clock types which can be assigned to the MCO
 * @param  mco    : Can be either MCO1 or MCO2
 *
 * @note
 * This function takes care about the proper assignment of MCO1/MCO2 to
 * the corresponding GPIO port/pin.
 *
 * @note
 * The selected clock to output onto MCO must not exceed 100MHz!
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
 *          <td>CFGR</td>
 *          <td rowspan="1">MCO1/MCO2</td>
 *          <td rowspan="1">22/31</td>
 *          <td rowspan="1">Selects either MCO1 or MCO2 as clock output. Configures the desired GPIO pin internally.</td>
 *      </tr>
 * </table>
 */
void rccAssignClk2MCO(MCO_CLK_SRC_t clkSrc, MCO_TypeDef_t mco)
{
    GPIO_TypeDef *portMCO1, *portMCO2;
    PIN_NUM_t pinMCO1, pinMCO2;

    // MCO1/2 port/pin assignment verified for all STM32F4xx MCUs
    // Might be different for other MCU types, e.g. Cortex-M7
    portMCO1 = GPIOA;
    pinMCO1 = PIN8;
    portMCO2 = GPIOC;
    pinMCO2 = PIN9;

    if (MCO1 == mco)
    {
        RCC->CFGR &= ~RCC_CFGR_MCO1_Msk;
        switch (clkSrc)
        {
            case HSI:
                RCC->CFGR &= ~RCC_CFGR_MCO1_Msk;
                break;

            case LSE:
                RCC->CFGR |= RCC_CFGR_MCO1_0;
                break;

            case HSE:
                RCC->CFGR |= RCC_CFGR_MCO1_1;
                break;

            case PLL_MAIN:
                RCC->CFGR |= (RCC_CFGR_MCO1_0 | RCC_CFGR_MCO1_1);
                break;

            default:
                RCC->CFGR &= RCC_CFGR_MCO1_Msk;
                break;
        }

        gpioInitPort(portMCO1);
        gpioSelectPinMode(portMCO1, pinMCO1, ALTFUNC);
        gpioSelectAltFunc(portMCO1, pinMCO1, AF0);
        gpioSetOutputSpeed(portMCO1, pinMCO1, HIGH_SPEED);

    }
    else if (MCO2 == mco)
    {
        RCC->CFGR &= ~RCC_CFGR_MCO2_Msk;
        switch (clkSrc)
        {
            case SYSCLK:
                RCC->CFGR &= ~RCC_CFGR_MCO2_Msk;
                break;

            case PLL_I2S:
                RCC->CFGR |= RCC_CFGR_MCO2_0;
                break;

            case HSE:
                RCC->CFGR |= RCC_CFGR_MCO2_1;
                break;

            case PLL_MAIN:
                RCC->CFGR |= (RCC_CFGR_MCO2_0 | RCC_CFGR_MCO2_1);
                break;

            default:
                RCC->CFGR &= ~RCC_CFGR_MCO2_Msk;
                break;
        }

        gpioInitPort(portMCO2);
        gpioSelectPinMode(portMCO2, pinMCO2, ALTFUNC);
        gpioSelectAltFunc(portMCO2, pinMCO2, AF0);
        gpioSetOutputSpeed(portMCO2, pinMCO2, HIGH_SPEED);
    }
}

/**
 * @ingroup rcc2
 * Sets the AHB clock. It depends directly on the previously set system clock.
 *
 * @param  div : This is the divider by which the system clock is divided.
 *
 * @note
 * If Ethernet is used the AHB clock must be set to at least 25MHz.
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
 *          <td>CFGR</td>
 *          <td rowspan="1">HPRE</td>
 *          <td rowspan="1">7...4</td>
 *          <td rowspan="1">Selects the prescaler for the AHB</td>
 *      </tr>
 * </table>
 */
void rccSetAHBPrescaler(AHB_DIVIDER_t div)
{
    RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
    switch (div)
    {
        case SYSCLK_DIV_1:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
            break;

        case SYSCLK_DIV_2:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV2;
            break;

        case SYSCLK_DIV_4:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV4;
            break;

        case SYSCLK_DIV_8:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV8;
            break;

        case SYSCLK_DIV_16:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV16;
            break;

        case SYSCLK_DIV_64:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV64;
            break;

        case SYSCLK_DIV_128:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV128;
            break;

        case SYSCLK_DIV_256:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV256;
            break;

        case SYSCLK_DIV_512:
            RCC->CFGR |= RCC_CFGR_HPRE_DIV512;
            break;
    }
}

/**
 * @ingroup rcc2
 * Sets the APB1 clock rate. It is derived from the AHB clock.
 *
 * @param  div : This is the divider by which AHB clock is divided.
 *
 * @note
 * This function is a replacement for the now deprecated function rccSetAPBPrescaler(). That function set the APB1
 * clock internally to be 50% of the APB2 clock rate.
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
 *          <td>CFGR</td>
 *          <td rowspan="1">PPRE1/PPRE2</td>
 *          <td rowspan="1">12...10/15...13</td>
 *          <td rowspan="1">Selects the prescaler for APB2. APB1 is then set to 50% of APB2.</td>
 *      </tr>
 * </table>
 */
void rccSetAPB1Prescaler(APB_DIVIDER_t div)
{
    RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
    switch (div)
    {
        case AHB_DIV_1:
            RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
            break;

        case AHB_DIV_2:
            RCC->CFGR |= RCC_CFGR_PPRE1_2;
            break;

        case AHB_DIV_4:
            RCC->CFGR |= (RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0);
            break;

        case AHB_DIV_8:
            RCC->CFGR |= (RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1);
            break;

        case AHB_DIV_16:
            RCC->CFGR |= (RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_0);    // The maximum divider is 16!
            break;
    }
}

/**
 * @ingroup rcc2
 * Sets the APB2 clock rate. It is derived from the AHB clock.
 *
 * @param  div : This is the divider by which AHB clock is divided.
 *
 * @note
 * This function is a replacement for the now deprecated function rccSetAPBPrescaler(). That function set the APB2
 * clock rate and implicitly the APB1 clock rate. However, making two independent functions improves the flexibiblity
 * of clock setting.
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
 *          <td>CFGR</td>
 *          <td rowspan="1">PPRE1/PPRE2</td>
 *          <td rowspan="1">12...10/15...13</td>
 *          <td rowspan="1">Selects the prescaler for APB2. APB1 is then set to 50% of APB2.</td>
 *      </tr>
 * </table>
 */
void rccSetAPB2Prescaler(APB_DIVIDER_t div)
{
    RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
    switch (div)
    {
        case AHB_DIV_1:
            RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
            break;

        case AHB_DIV_2:
            RCC->CFGR |= RCC_CFGR_PPRE2_2;
            break;

        case AHB_DIV_4:
            RCC->CFGR |= (RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_0);
            break;

        case AHB_DIV_8:
            RCC->CFGR |= (RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_1);
            break;

        case AHB_DIV_16:
            RCC->CFGR |= (RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_0);    // The maximum divider is 16!
            break;
    }
}

/* EOF */
