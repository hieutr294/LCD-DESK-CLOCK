/*
 * encoder.c
 *
 *  Created on: Jun 3, 2025
 *      Author: minhh
 */

#include "stm32f103xx.h"
#include "encoder.h"
#include "gpio.h"

void __gpioConfig(uint8_t pinNumber, GPIO_RegDef_t* gpioX){
	GPIO_Handle_t gpio;
	gpio.pGPIOX = gpioX;
	gpio.GPIO_PinConfig.pinNumber = pinNumber;
	gpio.GPIO_PinConfig.pinMode = INPUT_PUPD;
	GPIO_ClockControl(gpio.pGPIOX, ENABLE);
	GPIO_Init(&gpio);
}

void __timerConfig(TIMER_RegDef_t* timerX, uint8_t smsValue, uint8_t polarity, uint8_t channelToCount, uint16_t maxValue){
	if(timerX == TIM2){
		TIM2_PCLK_EN();
		timerX->SMCR |= (smsValue<<0);
		if(channelToCount==COUNT_ON_C1){
			timerX->CCER |= (polarity<<1);
			timerX->CCMR1 |= (1<<0);
			timerX->SMCR |= (5<<4);
		}else if(channelToCount==COUNT_ON_C2){
			timerX->CCER |= (polarity<<5);
			timerX->CCMR1 |= (1<<8);
			timerX->SMCR |= (6<<4);
		}else if(channelToCount==COUNT_ON_BOTH){
			timerX->CCER |= (polarity<<1);
			timerX->CCER |= (polarity<<5);
			timerX->CCMR1 |= (1<<0);
			timerX->CCMR1 |= (1<<8);
			timerX->SMCR |= (7<<4);
//			timerX->SMCR |= (6<<4);
		}
		timerX->ARR = maxValue;
		timerX->CR1 |= (1<<0);
	}else if(timerX == TIM3){
		TIM3_PCLK_EN();
		timerX->SMCR |= (smsValue<<0);
		if(channelToCount==COUNT_ON_C1){
			timerX->CCER |= (polarity<<1);
			timerX->CCMR1 |= (1<<0);
			timerX->SMCR |= (5<<4);
		}else if(channelToCount==COUNT_ON_C2){
			timerX->CCER |= (polarity<<5);
			timerX->CCMR1 |= (1<<8);
			timerX->SMCR |= (6<<4);
		}else if(channelToCount==COUNT_ON_BOTH){
			timerX->CCER |= (polarity<<1);
			timerX->CCER |= (polarity<<5);
			timerX->CCMR1 |= (1<<0);
			timerX->CCMR1 |= (1<<8);
			timerX->SMCR |= (5<<4);
			timerX->SMCR |= (6<<4);
		}
		timerX->ARR = maxValue;
		timerX->CR1 |= (1<<0);
	}else if(timerX == TIM4){
		TIM4_PCLK_EN();
		timerX->SMCR |= (smsValue<<0);
		if(channelToCount==COUNT_ON_C1){
			timerX->CCER |= (polarity<<1);
			timerX->CCMR1 |= (1<<0);
			timerX->SMCR |= (5<<4);
		}else if(channelToCount==COUNT_ON_C2){
			timerX->CCER |= (polarity<<5);
			timerX->CCMR1 |= (1<<8);
			timerX->SMCR |= (6<<4);
		}else if(channelToCount==COUNT_ON_BOTH){
			timerX->CCER |= (polarity<<1);
			timerX->CCER |= (polarity<<5);
			timerX->CCMR1 |= (1<<0);
			timerX->CCMR1 |= (1<<8);
			timerX->SMCR |= (5<<4);
			timerX->SMCR |= (6<<4);
		}
		timerX->ARR = maxValue;
		timerX->CR1 |= (1<<0);
	}
}

void EncoderInit(Encoder_Config_t* config){
	if(config->channelToCount == COUNT_ON_C1){
		if(config->timerX == TIM2){
			__gpioConfig(0,GPIOA);
			__timerConfig(config->timerX, 2, config->inputPolarity, COUNT_ON_C1, config->maxValue);
		}else if(config->timerX == TIM3){
			__gpioConfig(6,GPIOA);
			__timerConfig(config->timerX, 2, config->inputPolarity, COUNT_ON_C1, config->maxValue);
		}else if(config->timerX == TIM4){
			__gpioConfig(6,GPIOB);
			__timerConfig(config->timerX, 2, config->inputPolarity, COUNT_ON_C1, config->maxValue);
		}
	}else if(config->channelToCount == COUNT_ON_C2){
		if(config->timerX == TIM2){
			__gpioConfig(1,GPIOA);
			__timerConfig(config->timerX, 1, config->inputPolarity, COUNT_ON_C2, config->maxValue);
		}else if(config->timerX == TIM3){
			__gpioConfig(7,GPIOA);
			__timerConfig(config->timerX, 1, config->inputPolarity, COUNT_ON_C2, config->maxValue);
		}else if(config->timerX == TIM4){
			__gpioConfig(7,GPIOB);
			__timerConfig(config->timerX, 1, config->inputPolarity, COUNT_ON_C2, config->maxValue);
		}
	}else if(config->channelToCount == COUNT_ON_BOTH){
		if(config->timerX == TIM2){
			__gpioConfig(0,GPIOA);
			__gpioConfig(1,GPIOA);
			__timerConfig(config->timerX, 3, config->inputPolarity, COUNT_ON_BOTH, config->maxValue);
		}else if(config->timerX == TIM3){
			__gpioConfig(6,GPIOA);
			__gpioConfig(7,GPIOA);
			__timerConfig(config->timerX, 3, config->inputPolarity, COUNT_ON_BOTH, config->maxValue);
		}else if(config->timerX == TIM4){
			__gpioConfig(6,GPIOB);
			__gpioConfig(7,GPIOB);
			__timerConfig(config->timerX, 3, config->inputPolarity, COUNT_ON_BOTH, config->maxValue);
		}
	}
}
