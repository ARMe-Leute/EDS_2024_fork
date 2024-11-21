/*
 * systick.h
 *
 *  Created on: Apr 13, 2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALSYSTICK_H_
#define MCALSYSTICK_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup systick4
 * @{
 */
/* Makros */
#define DECREMENT_TIMER( timer )   \
    ( {                            \
        if ( timer > 0 )           \
            --timer;               \
    } )

/* Defines */
#define SYSTICK_1US     (1000000)
#define SYSTICK_10US	(100000)
#define SYSTICK_100US	(10000)
#define SYSTICK_1MS		(1000)
#define SYSTICK_10MS	(100)
#define SYSTICK_100MS	(10)
#define SYSTICK_1S		(1)

/**
 * @}
 */

/* Function prototypes */
extern void systickInit(uint32_t divisor);
extern void systickSetMillis(uint32_t *timer, uint32_t millis);
extern void systickSetMicros(uint32_t *timer, uint32_t micros_x_10);
extern void systickSetTicktime(uint32_t *timer, uint32_t ticktime);
extern bool isSystickExpired(uint32_t timer);
extern void SysTick_Handler(void);
extern void systickUpdateTimer(uint32_t *timer);
extern void systickUpdateTimerList(uint32_t *list, uint8_t arraySize);
extern void systickDelay(uint32_t *timer, uint32_t delay);

/* Externe Variablen */
extern bool timerTrigger;

#ifdef __cplusplus
}
#endif


#endif /* MCALSYSTICK_H_ */
