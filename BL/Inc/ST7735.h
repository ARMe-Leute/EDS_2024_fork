
/***************************************************
  Code has been ported from Arduino Adafruit library.
  Copyrigths below:

  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ADAFRUIT_ST7735H_
#define _ADAFRUIT_ST7735H_

//#include "stm32f30x.h"
#include <stm32f4xx.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalSysTick.h>
#include <BALO.h>


/*****************************************************************************************
Hardware Configuration
******************************************************************************************/
#ifndef __VCP_HW_CONFIG__
#define __VCP_HW_CONFIG__

/***********************************************************/
#ifdef PIGGYBAG
	//Define Display Piggyback

	#define ST7735_SPI_PORT		GPIOA
	#define ST7735_CLK			PIN5
	#define ST7735_MOSI			PIN7

	//RST  PA9	Piggyback

	#define ST7735_RST_PORT		GPIOA
	#define ST7735_RST			PIN9
	#define ST7735_RST1  gpioSetPin(GPIOA, PIN9)
	#define ST7735_RST0  gpioResetPin(GPIOA, PIN9)


	//A0   PB7	Piggyback
	#define ST7735_DC_PORT		GPIOC
	#define ST7735_DC			PIN7
	#define ST7735_DC1   gpioSetPin(GPIOC, PIN7)
	#define ST7735_DC0	  gpioResetPin(GPIOC, PIN7)


	//CS   PB6	Piggyback
	#define ST7735_CS_PORT		GPIOB
	#define ST7735_CS			PIN6
	#define ST7735_CS1   gpioSetPin(GPIOB, PIN6)
	#define ST7735_CS0   gpioResetPin(GPIOB, PIN6)

#endif

#ifdef BALA2024
//Define Display BALA2042_V2.0

/****
 * RST - Reset 			PB4
 * DC  - Data/Command	PB5
 * CS  - ChipSelect		PA15
 * MOSI -				PA7
 * CLK					PA5
 *
 */

	#define ST7735_SPI_PORT		GPIOA
	#define ST7735_CLK			PIN5
	#define ST7735_MOSI			PIN7

	//RST  PB4	BALA2042_V2.0

	#define ST7735_RST_PORT		GPIOB
	#define ST7735_RST			PIN4
	#define ST7735_RST1  gpioSetPin(GPIOB, PIN4)
	#define ST7735_RST0  gpioResetPin(GPIOB, PIN4)

	//A0   PB5	BALA2042_V2.0
	#define ST7735_DC_PORT		GPIOB
	#define ST7735_DC			PIN5
	#define ST7735_DC1   gpioSetPin(GPIOB, PIN5)
	#define ST7735_DC0	  gpioResetPin(GPIOB, PIN5)

	//CS   PA15	BALA2042_V2.0
	#define ST7735_CS_PORT		GPIOA
	#define ST7735_CS			PIN15
	#define ST7735_CS1   gpioSetPin(GPIOA, PIN15)
	#define ST7735_CS0   gpioResetPin(GPIOA, PIN15)

#endif
 /***********************************************************/


extern void spiInit(void);
extern void tftSPISenddata(const uint8_t cmd);
extern void tftSPISenddata16(const uint16_t data);
extern void tftSendCmd(const uint8_t cmd);
extern void tftSendData(const uint8_t data);

extern uint32_t 	ST7735_Timer;

#endif /* __VCP_HW_CONFIG__ */

/*****************************************************************************************
Hardware Configuration end
******************************************************************************************/


/*****************************************************************************************
Fonts
******************************************************************************************/

//include used Fonts
#define __DEFAULT_FONTS__ //all Fonts
//#define __Small_Fonts__
//#define __Big_Fonts__
//#define __SevenSegNum_Fonts__


#ifdef __DEFAULT_FONTS__

extern const unsigned char SmallFont[];
extern const unsigned char BigFont[];
extern const unsigned char SevenSegNumFont[];

#endif /* __DEFAULT_FONTS__ */


#ifdef __Small_Fonts__

extern const unsigned char SmallFont[];

#endif /* __Small_Fonts__ */


#ifdef __Big_Fonts__

extern const unsigned char BigFont[];

#endif /* __Big_Fonts__ */


#ifdef __SevenSegNum_Fonts__

extern const unsigned char SevenSegNumFont[];

#endif /* __SevenSegNum_Fonts__ */


/*****************************************************************************************
Fonts end
******************************************************************************************/


// some flags for initR() :(
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB   0x2

#define ST7735_TFTWIDTH  128
#define ST7735_TFTHEIGHT 160

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// 5-6-5 bits colors samples
#define	tft_BLACK   0x0000
#define	tft_BLUE    0x001F
#define	tft_RED     0xF800
#define	tft_GREEN   0x07E0
#define tft_CYAN    0x07FF
#define tft_MAGENTA 0xF81F
#define tft_YELLOW  0xFFE0
#define tft_GREY	0b0001100100100011
#define tft_WHITE   0xFFFF

#define INVERT_ON		1
#define INVERT_OFF		0

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

#define PORTRAIT	0
#define LANDSCAPE	1
#define PORTRAIT_FLIP	2
#define LANDSCAPE_FLIP	3

#define CENTER	-1
#define RIGHT	-2

#define bitmapdatatype uint16_t *

extern void delayms(uint32_t delay_value);

// Initialization for ST7735R screens (green or red tabs)
extern void tftInitR(uint8_t options);
extern uint8_t tftGetWidth(void);
extern uint8_t tftGetHeight(void);

extern void tftInvertDisplay(const uint8_t mode);
extern void tftSetRotation(uint8_t m);
extern void tftFillScreen(uint16_t color);
// Pass 8-bit (each) R,G,B, get back 16-bit packed color

extern uint16_t tftColor565(uint8_t r, uint8_t g, uint8_t b);
extern void tftFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
extern void tftDrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
extern void tftDrawPixel(int16_t x, int16_t y, uint16_t color);
extern void tftDrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
extern void tftSetAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
extern void tftPushColor(uint16_t color); // CAUTION!! can't be used separately
extern void tftDrawFastLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
extern void tftDrawRect(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2, uint16_t color);
extern void tftDrawCircle(int16_t x, int16_t y, int radius, uint16_t color);
extern void tftFillCircle(int16_t x, int16_t y, int radius, uint16_t color);
extern void tftDrawBitmap(int x, int y, int sx, int sy, bitmapdatatype data, int scale);
extern void tftDrawBitmapRotate(int x, int y, int sx, int sy, bitmapdatatype data, int deg, int rox, int roy);
extern void tftSetFont(uint8_t* font);
extern void tftSetColor(uint16_t FontColor, uint16_t BackColor);

extern void tftPrintInt(int value,int x, int y, int deg);
extern void tftPrintLong(long value,int x, int y, int deg);
extern void tftPrintFloat(float value,int x, int y, int deg);
extern void tftPrintDouble(double value,int x, int y, int deg);
extern void tftPrint(char *st, int x, int y, int deg);
extern void tftPrintColor(char *st, int x, int y, uint16_t FontColor);
#endif
