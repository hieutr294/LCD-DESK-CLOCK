/*
 * interrupts.c
 *
 *  Created on: Mar 27, 2024
 *      Author: minhh
 */

#include <stdint.h>
#include "stm32f103xx.h"
#include "interrupts.h"

EXTI_RegDef_t* exti = EXTI;
AFIO_RegDef_t* afio = AFIO;
NVIC_ISER_t* nvicIser = NVIC_ISER;
NVIC_IPR_t* nvicIpr = NVIC_IPR;

void __setPriority(uint8_t IRQ, uint8_t priority){
	NVIC_IPR_t* nvicIPR = (NVIC_IPR_t*)(NVIC_IPR_BASE_ADDRESS);
	uint8_t iprx = (IRQ)/4;
	uint8_t iprx_section = IRQ%4;
	uint8_t shift_amount = (8*iprx_section) + (8-4); // 8 - number of bit implement
	nvicIPR->IPR[iprx] |= (priority<<shift_amount);
}

void Interrupt_Config(Interrupts_Config_t* pInterruptsConfig){

	AFIO_PCLK_EN();

	if(pInterruptsConfig->interruptPinNumber<=3){
		afio->EXTICR[0] |= (pInterruptsConfig->interruptPort<<pInterruptsConfig->interruptPinNumber);
	}
	else if(pInterruptsConfig->interruptPinNumber<=7 && pInterruptsConfig->interruptPinNumber>=4){
		afio->EXTICR[1] |= (pInterruptsConfig->interruptPort<<pInterruptsConfig->interruptPinNumber);
	}
	else if(pInterruptsConfig->interruptPinNumber<=11 && pInterruptsConfig->interruptPinNumber>=8){
		afio->EXTICR[2] |= (pInterruptsConfig->interruptPort<<pInterruptsConfig->interruptPinNumber);
	}
	else if(pInterruptsConfig->interruptPinNumber<=15 && pInterruptsConfig->interruptPinNumber>=12){
		afio->EXTICR[3] |= (pInterruptsConfig->interruptPort<<pInterruptsConfig->interruptPinNumber);
	}

	if(pInterruptsConfig->edgeTrigger == RISING){
		exti->RTSR |= (1<<pInterruptsConfig->interruptPinNumber);
		exti->FTSR &= ~(1<<pInterruptsConfig->interruptPinNumber);
	}
	else if(pInterruptsConfig->edgeTrigger == FALLING){
		exti->FTSR |= (1<<pInterruptsConfig->interruptPinNumber);
		exti->RTSR &= ~(1<<pInterruptsConfig->interruptPinNumber);
	}
	else if(pInterruptsConfig->edgeTrigger == RISING_FALLING){
		exti->RTSR |= (1<<pInterruptsConfig->interruptPinNumber);
		exti->FTSR |= (1<<pInterruptsConfig->interruptPinNumber);
	}

	if(pInterruptsConfig->interruptPinNumber<=4 && pInterruptsConfig->interruptPinNumber>=0){
		__setPriority(pInterruptsConfig->interruptPinNumber+6,pInterruptsConfig->priority);
		nvicIser->ISER[0] |= (1<<(pInterruptsConfig->interruptPinNumber+6));
	}
	else if(pInterruptsConfig->interruptPinNumber<=9 && pInterruptsConfig->interruptPinNumber>=5){
		__setPriority(EXTI9_5_INTERRUPT,pInterruptsConfig->priority);
		nvicIser->ISER[0] |= (1<<EXTI9_5_INTERRUPT);
	}
	else if(pInterruptsConfig->interruptPinNumber<=15 && pInterruptsConfig->interruptPinNumber>=10){
		__setPriority(EXTI15_10_INTERRUPT,pInterruptsConfig->priority);
		nvicIser->ISER[1] |= (1<<(EXTI15_10_INTERRUPT-32));
	}

	exti->IMR |= (1<<pInterruptsConfig->interruptPinNumber);
}

void Interrupts_Handling(Interrupts_Config_t* pInterruptsConfig, void* pFunc()){
	pFunc();
	exti->PR|=(1<<(pInterruptsConfig->interruptPinNumber+6));
}
