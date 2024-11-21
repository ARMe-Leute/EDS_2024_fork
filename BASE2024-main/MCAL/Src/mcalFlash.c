/**
 * @defgroup flash  Funktionen des Flash-Controllers (mcalFlash.h/.c)
 * @defgroup flash0 Deprecated System Functions
 * @ingroup  flash
 * @defgroup flash1 System Macros
 * @ingroup  flash
 * @defgroup flash2 System Standard Functions
 * @ingroup  flash
 * @defgroup flash3 Advanced System Functions
 * @ingroup  flash
 * @defgroup flash4 RCC Enumerations and Definitions
 * @ingroup  flash
 */
/**
 * mcalFlashCtrl.c
 *
 *  Created on: 03.07.2021
 *      Author: Ralf
 */

#include <mcalFlash.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/**
 * @ingroup flash2
 * Configures the number of wait states (WS) when accessing the internal Flash memory.
 *
 * @param  sysclk : The number of wait states depends on the MCU clock and the supply voltage. However, this function
 * doesn't take care of the supply voltage.
 *
 * @note
 * The number of WS depends immediately from the supply voltage and the selected MCU clock. This function assumes that
 * the supply voltage is in the range of 2.7 to 3.6V. Other voltages must be handled separately. This function enables
 * all caches by default.
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
 *          <td>Flash ACR</td>
 *          <td rowspan="1">LATENCY</td>
 *          <td rowspan="1">3...0</td>
 *          <td rowspan="1">Sets the Flas wait states according to the desired system clock</td>
 *      </tr>
 * </table>
 */
void flashConfigWaitStates(uint16_t sysclk)
{
    // Reset the Flash Access Control Register to the default value after reset
    FLASH->ACR &= ~(FLASH_ACR_LATENCY_Msk || FLASH_ACR_DCRST_Msk || FLASH_ACR_DCEN_Msk || FLASH_ACR_ICRST_Msk ||
                    FLASH_ACR_ICEN_Msk);
    // Enable the caches
    FLASH->ACR |= (FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN);

    if (sysclk <= 30)
    {
        FLASH->ACR &= ~FLASH_ACR_LATENCY_0WS;
    }
    else if ((sysclk > 30) && (sysclk <= 60))
    {
        FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
    }
    else if ((sysclk > 60) && (sysclk <= 90))
    {
        FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
    }
    else if ((sysclk > 90) && (sysclk <= 120))
    {
        FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
    }
    else if ((sysclk > 120) && (sysclk <= 150))
    {
        FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
    }
    else if ((sysclk > 150) && (sysclk <= 180))
    {
        FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
    }
}
