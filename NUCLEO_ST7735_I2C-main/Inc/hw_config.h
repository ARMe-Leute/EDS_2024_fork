#ifndef __VCP_HW_CONFIG__
#define __VCP_HW_CONFIG__

//#include "stm32f401xc.h"
#include <stm32f4xx.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
//


#define DELAY_red      (900)
#define DELAY_green    (1500)
#define DELAY_blue     (700)

#define 	LED_GPIO		GPIOB
#define 	LED_red			PIN3
#define		LED_green		PIN4
#define		LED_blue		PIN10


#define LED_red_on 		gpioResetPin(LED_GPIO, LED_red)
#define LED_red_off		gpioSetPin(LED_GPIO, LED_red)
#define LED_green_on	gpioResetPin(LED_GPIO, LED_green)
#define LED_green_off	gpioSetPin(LED_GPIO, LED_green)
#define LED_blue_on		gpioResetPin(LED_GPIO, LED_blue)
#define LED_blue_off	gpioSetPin(LED_GPIO, LED_blue)



/****
 * RST - Reset 			PA9
 * DC  - Data/Command	PC7
 * CS  - ChipSelect		PB6
 * MOSI -				PA7
 * CLK					PA5
 *
 */

#define ST7735_SPI_PORT		GPIOA
#define ST7735_CLK			PIN5
#define ST7735_MOSI			PIN7

//RST  PA9

#define ST7735_RST_PORT		GPIOA
#define ST7735_RST			PIN9
#define ST7735_RST1  gpioSetPin(GPIOA, PIN9)
#define ST7735_RST0  gpioResetPin(GPIOA, PIN9)


//A0   PB7
#define ST7735_DC_PORT		GPIOC
#define ST7735_DC			PIN7
#define ST7735_DC1   gpioSetPin(GPIOC, PIN7)
#define ST7735_DC0	  gpioResetPin(GPIOC, PIN7)


//CS   PB6
#define ST7735_CS_PORT		GPIOB
#define ST7735_CS			PIN6
#define ST7735_CS1   gpioSetPin(GPIOB, PIN6)
#define ST7735_CS0   gpioResetPin(GPIOB, PIN6)





extern void lcd7735_setup(void);
extern void lcd7735_senddata(const uint8_t cmd);
extern void lcd7735_senddata16(const uint16_t data);
extern void lcd7735_sendCmd(const uint8_t cmd);
extern void lcd7735_sendData(const uint8_t data);

extern uint32_t 	ST7735_Timer;

#endif /* __VCP_HW_CONFIG__ */

