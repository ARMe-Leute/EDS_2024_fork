#ifndef ADC_H_
#define ADC_H_

#include <stm32f4xx.h>
#include <mcalGPIO.h>

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

#endif /* ADC_H_ */
