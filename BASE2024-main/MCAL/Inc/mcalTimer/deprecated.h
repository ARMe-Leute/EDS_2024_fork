/**
 * deprecated.h
 *
 *  Created on: Mar 15, 2021
 *      Author: Ralf Jesse
 */

#ifndef MCALTIMER_DEPRECATED_H_
#define MCALTIMER_DEPRECATED_H_

#include <stm32f4xx.h>
#include <mcalTimer/deprecated.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void timerBusClkOn(TIM_TypeDef *tim);
extern void timerBusClkOff(TIM_TypeDef *tim);

extern void timerSelectTimer(TIM_TypeDef *tim);
extern void timerDeselectTimer(TIM_TypeDef *tim);

#ifdef __cplusplus
}
#endif

#endif /* MCALTIMER_DEPRECATED_H_ */
