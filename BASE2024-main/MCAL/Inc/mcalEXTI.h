/**
 * mcalEXTI.h
 *
 *  Created on: 16.07.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALEXTI_H_
#define MCALEXTI_H_

#include <stm32f4xx.h>
#include <system_stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#include <mcalGPIO.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup exti2
 * @{
 */
typedef enum
{
    EXTI_OK                 =   0,
    EXTI_INVALID_IRQNUM     = -40,
    EXTI_INVALID_TRIGGER    = -41
} EXTI_RETURNCODE_t;

/**
 * @brief Enumeration of EXTI ports
 */
typedef enum
{
    PORT_A = 0,    // EXTI mask for GPIOA
    PORT_B,        // EXTI mask for GPIOB
    PORT_C,        // EXTI mask for GPIOC
    PORT_D,        // ...
    PORT_E,
    PORT_F,
    PORT_G,
    PORT_H         // EXTI mask for GPIOH
} EXIT_PORT;

/**
 * @brief Enumeration of EXTI interrupt sources
 */
typedef enum
{
    EXTI_PIN0 = 0,
    EXTI_PIN1,
    EXTI_PIN2,
    EXTI_PIN3,
    EXTI_PIN4,
    EXTI_PIN5,
    EXTI_PIN6,
    EXTI_PIN7,
    EXTI_PIN8,
    EXTI_PIN9,
    EXTI_PIN10,
    EXTI_PIN11,
    EXTI_PIN12,
    EXTI_PIN13,
    EXTI_PIN14,
    EXTI_PIN15,
    EXTI_VOLTAGE_DETECTION,
    EXTI_RTC_ALARM,
    EXTI_USB_OTG_FS,          // USB on-the-go full speed
    EXTI_DONT_USE,
    EXTI_USB_OTG_HS,          // USB on-the-go high speed
    EXTI_RTC_TAMPER,
    EXTI_RTC_WAKEUP
} EXTI_IRQ_NUM;

/**
 * @brief Enumeration of trigger types
 */
typedef enum
{
    RISING_EDGE = 0,
    FALLING_EDGE,
    RISING_AND_FALLING
} EXTI_TRIGGER;

/**
 * @}
 */

/**
 * Function prototypes
 */
extern void extiInit(void);
extern EXTI_IRQ_NUM extiConfigIrq(GPIO_TypeDef *port, PIN_NUM_t pin);
extern EXTI_IRQ_NUM extiEnableIrq(EXTI_IRQ_NUM irqNum);
extern EXTI_IRQ_NUM extiDisableIrq(EXTI_IRQ_NUM irqNum);
extern EXTI_IRQ_NUM extiSetTriggerEdge(EXTI_IRQ_NUM irqNum, EXTI_TRIGGER trigger);
extern void         extiResetPendingIRQ(EXTI_IRQ_NUM irq);

extern bool extiVerifyIrqNum(EXTI_IRQ_NUM irqNum);
extern bool extiVerifyTrigger(EXTI_TRIGGER trigger);

#ifdef __cplusplus
}
#endif



#endif /* MCALEXTI_H_ */
