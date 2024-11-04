/****************************************************************************************
*
* Hardware configuration and low level functions
*
* The idea of HW initialization and configuration have taken on from
* http://vg.ucoz.ru/load/stm32_ickhodnye_teksty_programm_na_si/stm32_biblioteka_podkljuchenija_displeja_na_kontrollere_st7735_dlja_stm32f4/16-1-0-41
****************************************************************************************/
#include <stdio.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include "hw_config.h"

static SPI_TypeDef  *spi  = SPI1;

void delay_ms(uint32_t delay_value) {
    //TimingDelay = delay_value;
	LED_blue_on;		// for Debug only
	//systickSetTicktime(&ST7735_Timer, delay_value);
	//while ((isSystickExpired(ST7735_Timer))== 0) { }
	systickDelay(&ST7735_Timer, delay_value);
	LED_blue_off;			//for Debug only
}

// Send byte via SPI to controller
void lcd7735_senddata(const uint8_t data)
{
	spiWriteByte(spi, ST7735_CS_PORT, ST7735_CS, data);
}


// Send byte via SPI to controller
void lcd7735_senddata16(const uint16_t data)
{
	spiWriteWord(spi, ST7735_CS_PORT, ST7735_CS, data);
}

// Send control command to controller
void lcd7735_sendCmd(const uint8_t cmd)
{
	ST7735_DC0;
    lcd7735_senddata(cmd);
}

// Send parameters o command to controller
void lcd7735_sendData(const uint8_t data)
{
    ST7735_DC1;
    lcd7735_senddata(data);
}

// Init hardware
void lcd7735_setup(void)
{
    // Configuration LED Pin
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


    /* Declartion of SPI & IO Pins für ST7735-Port
     *
     *
     */

    gpioSelectPort(ST7735_RST_PORT);
    gpioSelectPinMode(ST7735_RST_PORT, ST7735_RST, OUTPUT);		// RESET
    gpioSelectPort(ST7735_DC_PORT);
    gpioSelectPinMode(ST7735_DC_PORT, ST7735_DC, OUTPUT);		// DATA/Command

    gpioInitPort(ST7735_CS_PORT);
    gpioSelectPinMode(ST7735_CS_PORT,ST7735_CS, OUTPUT);        // CS
    gpioSelectPushPullMode(ST7735_CS_PORT, ST7735_CS, PULLUP);

    gpioInitPort(ST7735_SPI_PORT);
    gpioSelectPinMode(ST7735_SPI_PORT, ST7735_CLK, ALTFUNC);     // SPI1 Clock
    gpioSelectAltFunc(ST7735_SPI_PORT, ST7735_CLK, AF5);
    gpioSelectPinMode(ST7735_SPI_PORT, ST7735_MOSI, ALTFUNC);    // SPI1 MOSI
    gpioSelectAltFunc(ST7735_SPI_PORT, ST7735_MOSI, AF5);

    // Init SPI1
    spiSelectSPI(spi);

    spiInitSPI(spi, CLK_DIV_16, SPI_DATA_8_BIT, SSM_ON, SSI_LVL_HIGH, MASTER, SPI_PHASE_EDGE_1, SPI_IDLE_LOW);

}


