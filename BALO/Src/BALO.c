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
#include <ST7735.h>
#include <BALO.h>

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
    i2cInitI2C(i2c, IC2_DUTY_CYCLE_16_9, 15, I2C_CLOCK_200);
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
    i2cInitI2C(i2c2, IC2_DUTY_CYCLE_16_9, 15, I2C_CLOCK_1Mz);
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

void BALOsetup(void)
{
    // Configuration LED Pin

   ledActivate();
   i2cActivate();

}



uint16_t AlBeOszi(float *AlphaBeta)
{

// Variables for Oszi Function
const int16_t oszi[3] = {70,50,159};			//oszi ypos-Zero Level, y-amplitude, t_lenght
const uint16_t osziColor = tft_GREY;
const uint16_t aColor = tft_YELLOW;
const uint16_t bColor = tft_MAGENTA;

static int16_t timepos = 0;

// kleines Oszi als Zeitmitschrieb
	if (AlphaBeta[0] > 1) {AlphaBeta[0]= 1;}
	if (AlphaBeta[0] < -1) {AlphaBeta[0]= -1;}
	if (AlphaBeta[1] > 1) {AlphaBeta[1]= 1;}
	if (AlphaBeta[1] < -1) {AlphaBeta[1]= -1;}
	int16_t Ya = oszi[0] - (int16_t)((oszi[1]-1) * AlphaBeta[0]);			// - ST7735 y = 0 upper line inverter direct to y Scale
	int16_t Yb = oszi[0] - (int16_t)((oszi[1]-1) * AlphaBeta[1]);
	int16_t osziHight = oszi[1]*2;
	tftDrawFastVLine(timepos, (oszi[0]-oszi[1]), osziHight, osziColor);
	if (Ya == Yb)
	{
		tftDrawPixel(timepos,Ya,tft_WHITE);
	}
	else
	{
		tftDrawPixel(timepos,Ya,aColor);
		tftDrawPixel(timepos,Yb,bColor);
	}
	if (++timepos > oszi[2] )
	{
		timepos = 0;
		//lcd7735_fillRect(0, oszi[0]-oszi[1], oszi[2]+1, 2*oszi[1], ST7735_GREY);
	}
	tftDrawFastVLine(timepos, oszi[0], oszi[1]/2, tft_RED);
	return timepos;
}
