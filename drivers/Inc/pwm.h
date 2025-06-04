/*
 * timer.h
 *
 *  Created on: Mar 25, 2024
 *      Author: minhh
 */


#include "stm32f103xx.h"
#include <stdint.h>

#define OC1M_POS 4
#define OC2M_POS 12
#define OC3M_POS 4
#define OC4M_POS 12

#define CC1E_POS 0
#define CC2E_POS 4
#define CC3E_POS 8
#define CC4E_POS 12

#define CC1P_POS 1
#define CC2P_POS 5
#define CC3P_POS 9
#define CC4P_POS 13

#define MOE_POS 15
#define CEN_POS 0
#define CMS_POS 5
#define DIR_POS 4
#define ARPE_POS 7

#define EDGE_ALIGN 0
#define CENTER_ALIGH_MODE_1 1
#define CENTER_ALIGH_MODE_2 2
#define CENTER_ALIGH_MODE_3 3

#define COUNTER_UP 1
#define COUNTER_DOWN 2

#define PWM_MODE_1 6
#define PWM_MODE_2 7
#define PRELOAD_ENA 1
#define PRELOAD_DIS 0
#define POLARITY_HIGH 1
#define POLARITY_LOW 0

#define PWM_PIN_A0 1
#define PWM_PIN_A1 2
#define PWM_PIN_A2 3
#define PWM_PIN_A3 4

#define PWM_PIN_A6 5
#define PWM_PIN_A7 6
#define PWM_PIN_B0 7
#define PWM_PIN_B1 8

#define PWM_PIN_B6 9
#define PWM_PIN_B7 10
#define PWM_PIN_B8 11
#define PWM_PIN_B9 12

#define PWM_PIN_A8 13
#define PWM_PIN_A9 14
#define PWM_PIN_A10 15
#define PWM_PIN_A11 16

typedef struct{
	uint8_t pinNumber;
	uint8_t pwmMode;
	uint8_t polarity;
	uint8_t alighMode;
	uint8_t counterMode;
	uint8_t preloadCondition;
	uint32_t pwmFrequency;
	uint32_t currentClockFreq;
	uint16_t prescale;
}PWM_Config_t;

typedef struct{
	TIMER_RegDef_t* pTimerX;
	PWM_Config_t pTimerConfig;
}PWM_Handle_t;

void Timer_ClockControl(TIMER_RegDef_t* pTimerX, uint8_t conditon);
void PWM_Init(PWM_Handle_t* pTimerHandle);
void PWM_SetDuty(PWM_Handle_t* pTimerHandle, uint8_t* duty);
