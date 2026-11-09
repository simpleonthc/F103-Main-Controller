/*
 * buzzer.h
 *
 *  Created on: Nov 8, 2026
 *      Author: pk
 */
#include "main.h"

#ifndef STM32_BUZZER_BUZZER_H_
#define STM32_BUZZER_BUZZER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	TIM_HandleTypeDef *timer;
	__IO uint32_t channel;
	uint32_t timerClockFreqHz;
} Buzzer_InitTypeDef;

typedef struct
{
	Buzzer_InitTypeDef Init;
	uint8_t value;
	uint16_t blinkFreqHz;
	uint16_t clock;
	uint16_t clockPeriod;
} Buzzer_HandleTypeDef;

void Buzzer_Note(Buzzer_HandleTypeDef* handle, uint32_t noteFreq);
void Buzzer_NoNote(Buzzer_HandleTypeDef* handle);
void Buzzer_Init(Buzzer_HandleTypeDef* handle, Buzzer_InitTypeDef* config);
void Buzzer_Start(Buzzer_HandleTypeDef* handle);


#ifdef __cplusplus
}
#endif



#endif /* STM32_BUZZER_BUZZER_H_ */
