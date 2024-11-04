/**
 * Projekt Kap06-GPIO-01
 *
 * Einfaches Beispiel zum "Toggeln" von GPIO-Port A / Pin 0
 */

//#include <stm32f446xx.h>
#include <stm32f401xe.h>
#include <system_stm32f4xx.h>

/**
 * Kommentar in Zeile 14 entfernen, wenn Sie die MCAL testen möchten.
 */

#include <mcalGPIO.h>

// Funktionsprototypen
void delayMillis(uint16_t delay);

#define 	LED_GPIO		GPIOB
#define 	LED_red			PIN3
#define		LED_green		PIN4
#define		LED_blue		PIN10
#define 	Bit_red			(unsigned int) 1
#define		Bit_blue		(unsigned int) 2
#define		Bit_green		(unsigned int) 4

int main(void)
{

    gpioSelectPort(LED_GPIO);
    gpioSelectPinMode(LED_GPIO, LED_red, OUTPUT);
    gpioSelectPinMode(LED_GPIO, LED_green, OUTPUT);
    gpioSelectPinMode(LED_GPIO, LED_blue, OUTPUT);

    unsigned int color = 7;

    while(1)
    {
        //gpioTogglePin(GPIOB, PIN3);
    	if (color & Bit_red)
    	{
    		gpioResetPin(LED_GPIO, LED_red);
    	}
    	else
    	{
    	 	gpioSetPin(LED_GPIO, LED_red);
    	}
     	if (color & Bit_green)
        {
        	gpioResetPin(LED_GPIO, LED_green);
        }
        else
        {
         	gpioSetPin(LED_GPIO, LED_green);
        }
     	if (color & Bit_blue)
        {
        	gpioResetPin(LED_GPIO, LED_blue);
        }
        else
        {
         	gpioSetPin(LED_GPIO, LED_blue);
        }
        delayMillis(500);
        if (--color < 0)
        {
        	color = 7;
        }
    }
}

/**
 * !!!Sehr schlechte Version eines Delay!!!
 */
void delayMillis(uint16_t delay)
{
    uint16_t i = 0;

    for (; delay > 0; --delay)
    {
        for (i = 0; i < 1245; ++i)
        {
            ;
        }
    }
}
