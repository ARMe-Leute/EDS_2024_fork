/**
 ******************************************************************************
 * @file	graphics.c
 * @author	Prof Flaemig <https://github.com/tflaemig68/>
 * @brief	Graphics for TFT Display V1.1
 * @date	Aug 2025
 ******************************************************************************
 * @attention Graphic function for TFT Display
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 *
 ******************************************************************************
 */


#include <stdio.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <ST7735.h>
#include <graphics.h>


/**
 * @brief Init hardware pins for I2C
 */
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
    i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_50);
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
/**
 * @brief Init LED pins for RGB
 */
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

/**
 * @brief init the used IO-hardware
 */
void BALOsetup(void)
{
    // Configuration LED Pin
   ledActivate();
   i2cActivate();
}

/**
 * @brief initialize the Scope by setting the parameters
 * @param Scope - pointer to Oszi
 * @param PosX - Oszi X-Position on TFT Display for Oszi
 * @param PosY - Oszi Y-Position on TFT Display for Oszi Y = Zero default: 73
 * @param AmpY - Oszi Y-Amplitude on TFT Display for Oszi Amplitude default: 47
 * @param TimeX - Oszi Time Length on TFT Display for Oszi  default: 159 (maximum)
 * @param ColBG - BackgroundColor default: tft_GREY
 * @param ColA	- Color of A Channel Line default: tft_YELLOW;
 * @param ColB	- Color of B Channel Line default: tft_MAGENTA;
 *
 */
void OSZIinit(Scope_t* Scope, int16_t PosX, int16_t PosY, int16_t AmpY,int16_t TimeX, uint16_t ColBG, uint16_t ColA, uint16_t ColB)
{
	Scope->PosX = PosX;
	Scope->PosY = PosY;
	Scope->AmpY = AmpY;
	Scope->TimeX = TimeX;
	Scope->TimePos =0;		//! Set Current Time Pos to 0
	Scope->ColBG = ColBG;
	Scope->ColA = ColA;
	Scope->ColB = ColB;
}
/**
 * @brief run procedure for display A and B Line on this Scope
 */
int16_t OSZIrun(Scope_t* Oszi, float *AlphaBeta)
{
	// Variables for Oszi Function
	int16_t oszi[4] = {Oszi->PosY,Oszi->AmpY,Oszi->TimeX,Oszi->PosX};			//oszi ypos-Zero Level, y-amplitude, t_lenght
	uint16_t osziColor = Oszi->ColBG;
	uint16_t aColor = Oszi->ColA;
	uint16_t bColor = Oszi->ColB;

	int16_t timepos = Oszi->TimePos;

	// kleines Oszi als Zeitmitschrieb
		if (AlphaBeta[0] > 1) {AlphaBeta[0]= 1;}
		if (AlphaBeta[0] < -1) {AlphaBeta[0]= -1;}
		if (AlphaBeta[1] > 1) {AlphaBeta[1]= 1;}
		if (AlphaBeta[1] < -1) {AlphaBeta[1]= -1;}
		int16_t Ya = oszi[0] - (int16_t)((oszi[1]-1) * AlphaBeta[0]);			// - ST7735 y = 0 upper line inverter direct to y Scale
		int16_t Yb = oszi[0] - (int16_t)((oszi[1]-1) * AlphaBeta[1]);
		int16_t osziHight = oszi[1]*2;
		tftDrawFastVLine(timepos+oszi[3], (oszi[0]-oszi[1]), osziHight, osziColor);
		if (Ya == Yb)
		{
			tftDrawPixel(timepos+oszi[3],Ya,tft_WHITE);
		}
		else
		{
			tftDrawPixel(timepos+oszi[3],Ya,aColor);
			tftDrawPixel(timepos+oszi[3],Yb,bColor);
		}
		if (++timepos > oszi[2] )
		{
			timepos = 0;
			//lcd7735_fillRect(0, oszi[0]-oszi[1], oszi[2]+1, 2*oszi[1], osziColor);  // Clear all
		}
		tftDrawFastVLine(timepos+oszi[3], oszi[0], oszi[1]/2, tft_RED);
		Oszi->TimePos = timepos;
		return timepos;
}

/**
 * @brief define prototype structure for the Scope
 * the parameters are default values for a full screen Scope with enough space above for two text lines
 * Display Size Height x Width 128 x 160
 */
const Scope_t Scope = {
	.PosX = 0,			//! Oszi X-Position on TFT Display for Oszi  default: 0
	.PosY = 63,			//! Oszi Y-Position on TFT Display for Oszi Y = Zero default: 63
	.AmpY = 63,			//! OSZI Y-AMPLITUDE ON TFT DISPLAY FOR OSZI AMPLITUDE DEFAULT: 63 =128/2
	.TimeX = 159,		//! Oszi Time Length on TFT Display for Oszi  default: 159 (maximum)
	.TimePos = 0,		//! Current Time Pos
	.ColBG = tft_GREY,		//! BackgroundColor default: tft_GREY;
	.ColA = tft_YELLOW,		//! Color of A Channel Line default: tft_YELLOW;
	.ColB = tft_MAGENTA,	//! Color of A Channel Line default: tft_MAGENTA;
	.init = OSZIinit,
	.run = OSZIrun,
};

/**
 * old fashioned routine for a Scope
 * @param AlphaBeta - Pointer of the value Array[2]
 *
 */
uint16_t AlBeOszi(float *AlphaBeta)
{
	// Variables for Oszi Function
	const int16_t oszi[3] = {73,47,159};			//oszi ypos-Zero Level, y-amplitude, t_lenght
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
