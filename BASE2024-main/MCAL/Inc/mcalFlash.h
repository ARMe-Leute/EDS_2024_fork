/**
 * mcalFlash.h
 *
 *  Created on: 03.07.2021
 *      Author: Ralf
 */

#ifndef MCALFLASH_H_
#define MCALFLASH_H_

#include <stm32f4xx.h>
#include <mcalGPIO.h>

#ifdef _cplusplus
extern "C" {
#endif

/**
 * @ingroup flash4
 * @{
 */

/**
 * @}
 */

extern void flashConfigWaitStates(uint16_t sysclk);

#ifdef _cplusplus
}
#endif

#endif /* MCALFLASH_H_ */
