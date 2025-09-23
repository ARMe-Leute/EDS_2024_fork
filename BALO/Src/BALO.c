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
#include <mcalADC.h>
#include <ST7735.h>
#include <BALO.h>







analogCh_t analogCh;




// Init hardware

void ADCInit(GPIO_TypeDef *port, PIN_NUM_t pin, ADC_TypeDef *adc, ADC_CHANNEL_t chnList[], size_t listSize, ADC_RESOLUTION_t resolution) {
    if ((port == NULL) || (adc == NULL)) {
        return; 											// Safety check if port and adc pointer have been initialized correct
    }
    gpioSelectPort(port); 									// Activate GPIO clock
    adcSelectADC(adc); 										// Activate ADC clock
    gpioSelectPinMode(port, pin, ANALOG);					// Setting the GPIO Pin to analog mode
    gpioSelectPushPullMode(port, pin, NO_PULLUP_PULLDOWN);	// Disable Pull-up or Pull-down resistors
    adcDisableADC(adc);										// Disable the ADC to configure it
    adcSetChannelSequence(adc, chnList, listSize);			// Select the ADC-Channel
    adcSetResolution(adc, resolution);						// Select the ADC Resolution
    adcEnableADC(adc);										// Enable the ADC again after configuration
}



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
    i2cInitI2C(i2c2, IC2_DUTY_CYCLE_16_9, 15, I2C_CLOCK_200);
    i2cEnableDevice(i2c2);                        // MCAL I2C2 activ
#endif /* BALA2024 */
}


void adcActivate(void)
{
    GPIO_TypeDef 			*port  = GPIOA;							// Port which is used for the analog signal
#ifdef BALA2024
    PIN_NUM_t				pinA1   = PIN1;							// Pin which is used for the analog signal
#else
    PIN_NUM_t				pinA1   = PIN0;							// Pin which is used for the analog signal
#endif

    ADC_TypeDef				*adc   = ADC1;							// ADC which is used
    ADC_RESOLUTION_t 		resolution = ADC_RES_12BIT;				// Resolution of the ADC
    //ADC_CHN_16,				TempSensor
    ADC_CHANNEL_t chnList[] = { ADC_CHN_1};				// List of ADC channels PA1 BatteryVoltage, Intern CPU Temperature

    size_t         listSize = sizeof(chnList) / sizeof(chnList[0]);	// Calculate number of channel-list elements

    ADCInit(port, pinA1, adc, chnList, listSize, resolution);	// Initialization of the GPIO and the ADC
    adcStartConversion(adc);
}

/* Reading the ADC Value, restart conversion and returning the calculated Voltage*/
BatStat_t getBatVolt(analogCh_t* pADChn)
{
    const float ratioCh1 = 0.00888;  // coefficient for the ratio 11:1 (R1+R2)/R1 multiplied with 0.00080586 R1 =1k; R2 =10k
   /*
    * Variable for IC Temp measurement
    *  const float ratioCh16 = 0.00080586;
    const float vtemp25 = 1.43;
    const float ratioTemp =	22.222;
    uint16_t ADCTemp;
   */
    uint16_t ADCValue;
	if (pADChn->adc == NULL) {
        return 0; // Safety check
    }


	if (!adcIsConversionFinished(pADChn->adc))
   	{
    	adcStartConversion(pADChn->adc);
	}
    else
    {
       	ADCValue = adcGetConversionResult(pADChn->adc);
    	//ADCTemp = adcGetConversionResult(adc);
    	adcStartConversion(pADChn->adc);
    }
	//float cpuTemp = ratioTemp*(vtemp25-(float)ADCTemp*ratioCh16) + 25; // temp = (1.43 - Vtemp) / 4.5 + 25.00
    pADChn->BatVolt = ADCValue * ratioCh1;

    if (pADChn->BatVolt > halfBatVolt)
    {
    	pADChn->BatStatus=okBat;
    }
    else
    {
    	if (pADChn->BatVolt > emptyBatVolt)
    	{ pADChn->BatStatus=halfBat; }
    	else
    	{ pADChn->BatStatus=emptyBat; }
    }
    return pADChn->BatStatus;
}
/* end of BatteryVoltage.c */





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

/*
 * Darstellung eines byte Value als hexadezimalen String mit zwei ascii Symbolen.
 * return: pointer auf den String hex[2]
 */
uint8_t *convDecByteToHex(uint8_t byte)
{
    static  uint8_t hex[2] = { 0 };

    uint8_t temp;

    temp = byte % 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[1] = temp;

    temp = byte / 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[0] = temp;

    return hex;
}

/* scanAdr. 7Bit Adresse value
 * return	0 if no device found on scanAdr
 *			if yes  return the scanAdr.
 *			and display on the ST7735 Display
 *
 *
 */
uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr)
{
	uint8_t 	*outString2 = (uint8_t *) "Addr at: \0";
	uint8_t     port, *result;
#define yPosBase 18
	uint8_t foundAddr = 0;
	static int xPos[2] = {0,100};
	static int yPos[2] = {yPosBase, yPosBase};

	if (i2c == I2C1)
    {
	   port = 0;
    }
    else
    {
	   port = 1;
    }
    if (scanAddr == 0)
    {
    yPos[0] = yPosBase;
    yPos[1] = yPosBase;
    }

	foundAddr = i2cFindSlaveAddr(i2c, scanAddr);
	if (yPos[port] == 0)
	{
		tftPrint((char *)outString2,xPos[port],yPos[port],0);
		yPos[port] = 66;
	}
	result = convDecByteToHex(scanAddr);
	if (foundAddr != 0)
	{
		//outString = outString2;
		tftPrint((char *)result,xPos[port],yPos[port],0);
		yPos[port] = (int) 14 + yPos[port];
		if (yPos[port] > 100)
		{
			yPos[port] = yPosBase;
		}
	}
	else
	{
	//	tftPrint((char *)result,xPos,14,0);
	}
	return foundAddr;
}

// Tiefpassfilterung der drei Richtungsvektoren xyz
void low_pass(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp)
{
	static long _sto_xyz[3];
	uint8_t i;
	for (i=1;i<=3;i++)
	{
  	 	_sto_xyz[i] += (long) raw_data[i] - (filt_data[i] = _sto_xyz[i]/_tp);
	}
}
