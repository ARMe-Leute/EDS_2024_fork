/****************************************************************************************
*
* Hardware configuration and low level functions
*
*
****************************************************************************************/
#include <stdio.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <Balancer.h>

// Init hardware



void i2cActivate()
{
	I2C_TypeDef   *i2c  = I2C1;
#ifdef BALA2024
	I2C_TypeDef   *i2c2  = I2C2;
#endif /* BALA2024 */
	GPIO_TypeDef  *portB = GPIOB;
    // GPIOB-Bustakt aktivieren wegen der Verwendung von PB8/PB9 (I2C).
    i2cSelectI2C(i2c);                           // I2C1: Bustakt aktivieren
    //i2cDisableDevice(i2c);
    gpioInitPort(portB);
    gpioSelectPinMode(portB, PIN8, ALTFUNC);
    gpioSelectAltFunc(portB, PIN8, AF4);         // PB8 : I2C1 SCL
    gpioSelectPinMode(portB, PIN9, ALTFUNC);
    gpioSelectAltFunc(portB, PIN9, AF4);         // PB9 : I2C1 SDA

    /**
     * Verwenden Sie auf keinen Fall die MCU-internen Pull-up-Widerstaende!
     * Widerstandswerte: jeweils 4k7 fuer SDA und SCL!
     */
    gpioSetOutputType(portB, PIN8, OPENDRAIN);   // Immer externe Pull-up-
    gpioSetOutputType(portB, PIN9, OPENDRAIN);   // Widerstaende verwenden!!!
    // Initialisierung des I2C-Controllers
    i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_1Mz);		//I2C_CLOCK_1Mz
    i2cEnableDevice(i2c);                        // MCAL I2C1 activ
#ifdef BALA2024

    // GPIOB-Bustakt aktivieren wegen der Verwendung von PB10/PB3 (I2C).
    i2cSelectI2C(i2c2);                           // I2C2: Bustakt aktivieren
    gpioSelectPinMode(portB, PIN10, ALTFUNC);
    gpioSelectAltFunc(portB, PIN10, AF4);         // PB10 : I2C2 SCL
    gpioSelectPinMode(portB, PIN3, ALTFUNC);
    gpioSelectAltFunc(portB, PIN3, AF9);         // PB3 : 	I2C2 SDA

    /**
     * Verwenden Sie auf keinen Fall die MCU-internen Pull-up-Widerstaende!
     * Widerstandswerte: jeweils 4k7 fuer SDA und SCL!
     */
    gpioSetOutputType(portB, PIN10, OPENDRAIN);   // Immer externe Pull-up-
    gpioSetOutputType(portB, PIN3, OPENDRAIN);   // Widerstaende verwenden!!!
    // Initialisierung des I2C-Controllers
    i2cInitI2C(i2c2, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_50);
    i2cEnableDevice(i2c2);                        // MCAL I2C2 activ
#endif /* BALA2024 */
}






void ledActivate(void)
{
    gpioSelectPort(LED_GPIO);
    gpioSelectPinMode(LED_GPIO, LED_red, OUTPUT);
    gpioSetOutputType(LED_GPIO, LED_red	, PUSHPULL);
    gpioSelectPushPullMode(LED_GPIO, LED_red, PULLUP);
    LED_red_off;

    gpioSelectPinMode(LED_GPIO, LED_green, OUTPUT);
    gpioSetOutputType(LED_GPIO, LED_green, PUSHPULL);
    gpioSelectPushPullMode(LED_GPIO, LED_green, PULLUP);
    LED_green_off;

    gpioSelectPinMode(LED_GPIO, LED_blue, OUTPUT);
    gpioSetOutputType(LED_GPIO, LED_blue, PUSHPULL);
    gpioSelectPushPullMode(LED_GPIO, LED_blue, PULLUP);
    LED_blue_off;
}





void BalaHWsetup(void)
{
    // Configuration LED Pin

   ledActivate();
   i2cActivate();

}


