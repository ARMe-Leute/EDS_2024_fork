/*
 * bluetooth.h
 *
 *  Created on: 11.11.2024
 *      Author:
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include <bluetooth.h>
#include <stm32f401xe.h>
#include <system_stm32f4xx.h>

#include <stdint.h>
#include <stdbool.h>

#include <mcalGPIO.h>
#include <mcalUsart.h>


extern void bluetoothGetStatus();
extern void bluetoothInit();


#endif /* INC_BLUETOOTH_H_ */