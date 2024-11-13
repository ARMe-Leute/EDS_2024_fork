/*
 * bluetooth.c
 *
 *  Created on: 11.11.2024
 *      Author:
 */

#include <bluetooth.h>
#include <stm32f401xe.h>
#include <system_stm32f4xx.h>

#include <stdint.h>
#include <stdbool.h>

#include <mcalGPIO.h>
#include <mcalUsart.h>

// Create a data type named BluetoothModule_t
typedef struct BluetoothModule {
	// Todo: Variablen und Funktionen

	void (*init) (BluetoothModule_t*);
} BluetoothModule_t;
