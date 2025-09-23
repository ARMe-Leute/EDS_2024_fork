#ifndef ADCBAT_H_
#define ADCBAT_H_

#include <stm32f4xx.h>
#include <mcalGPIO.h>

#define DELAY_red      (900)
#define DELAY_green    (1500)
#define DELAY_blue     (700)

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


typedef struct
{
	ADC_TypeDef *adc;
	float BatVolt;
	BatStat_t BatStatus;
	float ratioBatVolt;
	float CpuTemp;
} analogCh_t;

extern analogCh_t analogCh;

extern void activateADC(PIN_NUM_t pinA1);
extern BatStat_t getBatVolt(analogCh_t* pADChn);

extern uint16_t AlBeOszi(float *AlphaBeta);

extern uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr);
extern uint8_t *convDecByteToHex(uint8_t byte);

// Tiefpassfilterung der drei Richtungsvektoren xyz
//extern void low_pass(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp);



#endif /* ADCBAT_H_ */
