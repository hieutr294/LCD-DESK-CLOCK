/*
 * interrupts.h
 *
 *  Created on: Mar 27, 2024
 *      Author: minhh
 */

#ifndef INC_INTERRUPTS_H_
#define INC_INTERRUPTS_H_

#include <stdint.h>
#include "stm32f103xx.h"

#define RISING 1
#define FALLING 2
#define RISING_FALLING 3

typedef struct{
	uint8_t edgeTrigger;
	uint8_t interruptPort;
	uint8_t interruptPinNumber;
	uint8_t priority;
}Interrupts_Config_t;

void Interrupt_Config(Interrupts_Config_t* pInterruptsConfig);
void Interrupts_Handling(Interrupts_Config_t* pInterruptsConfig, void* pFunc());

#endif /* INC_INTERRUPTS_H_ */
