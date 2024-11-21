/**
 * @defgroup sysTick  Systick Timer Functions (mcalSysTick.h/.c)
 * @defgroup sysTick0 Deprecated Systick Functions
 * @ingroup  sysTick
 * @defgroup sysTick1 Systick Macros
 * @ingroup  sysTick
 * @defgroup sysTick2 Systick Standard Functions
 * @ingroup  sysTick
 * @defgroup sysTick3 Advanced Systick Functions
 * @ingroup  sysTick
 * @defgroup sysTick4 Systick Enumerations and Definitions
 * @ingroup  sysTick
 *
 * @file        mcalSysTick.c
 * @brief       mcalSysTick.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 12, 2020
 *
 * @version     0.2
 * @copyright   GNU Public License Version 3 (GPLv3)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @note
 * This module <b>needs</b> a globally defined variable with exactly the type/name <b>uint32_t timer</b>!
 */

#include <stm32f4xx.h>
#include <system_stm32f4xx.h>
#include <mcalSysTick.h>

/* Makros */
/* @ingroup sysTick1 */
#define DECREMENT_TIMER( timer )   \
    ( {                            \
        if ( timer > 0 )           \
            --timer;               \
    } )

/**********************************************************
 * Deprecated functions                                   *
 *********************************************************/

/**
 * @deprecated
 * @ingroup sysTick0
 * Sets the tick interval of the software timer in milliseconds.
 *
 * @param  *timer  : Pointer to the software timer
 * @param   millis : Tick time of the software timer in milliseconds
 *
 * @note
 * This function is formally 100% compatible with the newly provided function systickSetTicktime(), i.e. the code is
 * identical to the new function **systickSetTicktime()**. The reason for the replacement is as follows: The name
 * **systickSetMillis()** might be confusing since the user might expect to get a ticktime of a multiple of **one**
 * millisecond. However, depending on the setting of the divider used in function systickInit() the real ticktime might
 * be also a multiple of 10 or 100 milliseconds.
 */
void systickSetMillis(uint32_t *timer, uint32_t millis)
{
    *timer = millis;
}


/**
 * @deprecated
 * @ingroup sysTick0
 * Sets the tick interval of the software timer in microseconds.
 *
 * @param  *timer  : Pointer to the software timer
 * @param   micros : Tick time of the software timer in microseconds
 *
 * @note
 * This function is formally 100% compatible with the newly provided function systickSetTicktime(), i.e. the code is
 * identical to the new function **systickSetTicktime()**. The reason for the replacement is as follows: The name
 * **systickSetMicros()** might be confusing since the user might expect to get a ticktime of a multiple of **one**
 * microsecond. However, depending on the setting of the divider used in function systickInit() the real ticktime might
 * be also a multiple of 10 or 100 microseconds.
 */
void systickSetMicros(uint32_t *timer, uint32_t micros)
{
    *timer = micros;
}


/**********************************************************
 * Use the following functions only                       *
 *********************************************************/

/**
 * @ingroup sysTick2
 * Initialization of the SysTick timer
 *
 * @param  divisor : Sets the tick time of SysTick
 */
void systickInit(uint32_t divisor)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / divisor);
}

/**
 * @ingroup sysTick2
 * Sets the tick interval of the software timer based on the initial timebase of the systick timer set with
 * systickInit().
 *
 * @param  *timer    : Pointer to the software timer
 * @param   ticktime : Tick time of the software timer in milliseconds
 *
 * @note
 * This function is a replacement for the deprecated functions systickSetMillis() and systickSetMicros(). The reason for
 * this replacement is documented in the 'Deprecated Systick Functions' section.
 */
void systickSetTicktime(uint32_t *timer, uint32_t ticktime)
{
    *timer = ticktime;
}

/**
 * @ingroup sysTick2
 * Returns *true* if the given timer is expired, otherwise *false*. This
 * function can be used for multiple software timers which all depend
 * on the Systick timer.
 *
 * @param   timer : This is a software timer.
 */
bool isSystickExpired(uint32_t timer)
{
	bool timerState = false;

	if (0 == timer)
	{
		timerState = true;
	}

	return timerState;
}

/**
 * @ingroup sysTick2
 * Interrupt service handler (ISR) for the SysTick timer
 *
 * @note
 * It is very important to provide a globally defined boolean variable with exactly the name <b>timerTrigger</b>. A very good place
 * for the introduction of this variable is the file <b>main.c</b> of your personal project.
 */
void SysTick_Handler(void)
{
	timerTrigger = true;
}

/**
 * @ingroup sysTick2
 * Updates the given timer.
 *
 * @param *timer : A pointer to the variable representing this timer.
 */
void systickUpdateTimer(uint32_t *timer)
{
    DECREMENT_TIMER(*timer);
    timerTrigger = false;
}

/**
 * @ingroup sysTick2
 * Most complex software projects use multiple software timers. This function
 * is a conveniant opportunity to update all software timers at the same time.
 * To achieve this you have to provide an array which holds the addresses of
 * that software timers. This array should be a part of your application! This
 * function "walks" throug the list of timers and executes the DECREMENT_TIMER
 * macro for each single software timer. Since this function works on the
 * addresses of the software timers each of them can be used in your application
 * by using its' original name (e.g. pwmUpdtTimer instead of timerList[n].
 *
 * @param  *list      : Pointer to an array of pointers
 * @param   arraySize : Size of the list
 */
void systickUpdateTimerList(uint32_t *list, uint8_t arraySize)
{
	uint32_t *timer;
	uint8_t  i;

	for (i = 0; i < (arraySize); ++i)
	{
		timer = (uint32_t *) list[i];
		DECREMENT_TIMER(*timer);
	}
    timerTrigger = false;
}

/**
 * @ingroup sysTick2
 * Implementation of a blocking delay() function.
 *
 * @param  *timer : Pointer to the variable that is used as software timer
 * @param   delay : Duration of the delay.
 *
 * @note
 * This function is a blocking function! All other parts of a software projects
 * are blocked from executing their functionality. Use this function only outside
 * of the while(1) loop, e.g. when initialization of a hardware component needs
 * time to perform one initialization step (e.g. if the datasheet of that component
 * demands a delay before doing the next initialization step).
 */
void systickDelay(uint32_t *timer, uint32_t delay)
{
    systickSetMillis(timer, delay);
    while (!isSystickExpired(*timer))
    {
        if (timerTrigger == true)
        {
            systickUpdateTimer(timer);
        }
    }
}
