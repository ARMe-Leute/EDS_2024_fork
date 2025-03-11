#ifndef BALO_H_
#define BALO_H_

// use define BALA2024 for HW DemoBoard Balancer V1.0
#define BALA2024

#define DELAY_red      (900)
#define DELAY_green    (1500)
#define DELAY_blue     (700)


#ifdef BALA2024
// LED_red_pin
#define 	LED_GPIO		GPIOB
#define 	LED_red			PIN15
#define		LED_green		PIN14
#define		LED_blue		PIN13

// default HW PiggyBag with TFT RotBPush Buttom V1.0
#else
#define 	LED_GPIO		GPIOB
#define 	LED_red			PIN3
#define		LED_green		PIN4
#define		LED_blue		PIN10
#endif

#define LED_red_on 		gpioResetPin(LED_GPIO, LED_red)
#define LED_red_off		gpioSetPin(LED_GPIO, LED_red)
#define LED_green_on	gpioResetPin(LED_GPIO, LED_green)
#define LED_green_off	gpioSetPin(LED_GPIO, LED_green)
#define LED_blue_on		gpioResetPin(LED_GPIO, LED_blue)
#define LED_blue_off	gpioSetPin(LED_GPIO, LED_blue)

extern void BALOsetup(void);
extern void i2cActivate(void);
extern void ledActivate(void);


#define halfBatVolt  (float)14
#define emptyBatVolt (float)13


typedef enum
{
	okBat = 0,
	halfBat,
	emptyBat,
	underVolt,
	overVolt
} BatStat_t;

typedef struct analogCh
{
	ADC_TypeDef *adc;
	float BatVolt;
	BatStat_t BatStatus;
	float ratioBatVolt;
	float CpuTemp;
} analogCh_t;



extern void adcActivate(void);
extern BatStat_t getBatVolt(analogCh_t* pADChn);

extern uint16_t AlBeOszi(float *AlphaBeta);

#endif /* BALO_H_ */
