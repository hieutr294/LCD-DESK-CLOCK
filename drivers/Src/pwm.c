/*
 * timer.c
 *
 *  Created on: Mar 25, 2024
 *      Author: minhh
 */

#include <pwm.h>
#include <stdint.h>
#include "stm32f103xx.h"
#include "gpio.h"
uint8_t channel;
void Timer_ClockControl(TIMER_RegDef_t* pTimerX, uint8_t condition){
	if(condition==ENABLE){
		if(pTimerX==TIM1){
			TIM1_PCLK_EN();
		}
		else if(pTimerX==TIM2){
			TIM2_PCLK_EN();
		}
//		else if(pTimerX==TIM3){
//			TIM3_PCLK_EN();
//		}
//		else if(pTimerX==TIM4){
//			TIM4_PCLK_EN();
//		}
		else if(pTimerX==TIM8){
			TIM8_PCLK_EN();
		}
	}
	else if(condition==DISABLE){
		if(pTimerX==TIM1){
			TIM1_PCLK_DI();
		}
		else if(pTimerX==TIM2){
			TIM2_PCLK_DI();
		}
//		else if(pTimerX==TIM3){
//			TIM3_PCLK_DI();
//		}
//		else if(pTimerX==TIM4){
//			TIM4_PCLK_DI();
//		}
		else if(pTimerX==TIM8){
			TIM8_PCLK_DI();
		}
	}
}

void PWM_Init(PWM_Handle_t* pTimerHandle){
	GPIO_Handle_t gpio;
	uint32_t timerClock = 0;
	uint32_t arrValue = 0;

	switch(pTimerHandle->pTimerConfig.pinNumber){
		case 1: //PIN A0
			gpio.GPIO_PinConfig.pinNumber = 0;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 1;
			break;
		case 2: //PIN A1
			gpio.GPIO_PinConfig.pinNumber = 1;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 2;
			break;
		case 3: //PIN A2
			gpio.GPIO_PinConfig.pinNumber = 2;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 3;
			break;
		case 4: //PIN A3
			gpio.GPIO_PinConfig.pinNumber = 3;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 4;
			break;
		case 5: //PIN A6
			gpio.GPIO_PinConfig.pinNumber = 6;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 1;
			break;
		case 6: //PIN A7
			gpio.GPIO_PinConfig.pinNumber = 7;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 2;
			break;
		case 7: //PIN B0
			gpio.GPIO_PinConfig.pinNumber = 0;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOB;
			channel = 3;
			break;
		case 8: //PIN B1
			gpio.GPIO_PinConfig.pinNumber = 1;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOB;
			channel = 4;
			break;
		case 9: //PIN B6
			gpio.GPIO_PinConfig.pinNumber = 6;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOB;
			channel = 1;
			break;
		case 10: //PIN B7
			gpio.GPIO_PinConfig.pinNumber = 7;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOB;
			channel = 2;
			break;
		case 11: //PIN B8
			gpio.GPIO_PinConfig.pinNumber = 8;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOB;
			channel = 3;
			break;
		case 12: //PIN B9
			gpio.GPIO_PinConfig.pinNumber = 9;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOB;
			channel = 4;
			break;
		case 13: //PIN A8
			gpio.GPIO_PinConfig.pinNumber = 8;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 1;
			break;
		case 14: //PIN A9
			gpio.GPIO_PinConfig.pinNumber = 9;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 2;
			break;
		case 15: //PIN A10
			gpio.GPIO_PinConfig.pinNumber = 10;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 3;
			break;
		case 16: //PIN A11
			gpio.GPIO_PinConfig.pinNumber = 11;
			gpio.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
			gpio.pGPIOX = GPIOA;
			channel = 4;
			break;
	}

	GPIO_ClockControl(gpio.pGPIOX, ENABLE);
	GPIO_Init(&gpio);

	switch(channel){
		case 1:
			pTimerHandle->pTimerX->CCER |= (1<<CC1E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC1P_POS);
			pTimerHandle->pTimerX->CCMR1 |= (pTimerHandle->pTimerConfig.pwmMode<<OC1M_POS);
			break;
		case 2:
			pTimerHandle->pTimerX->CCMR1 |= (pTimerHandle->pTimerConfig.pwmMode<<OC2M_POS);
			pTimerHandle->pTimerX->CCER |= (1<<CC2E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC2P_POS);
			break;
		case 3:
			pTimerHandle->pTimerX->CCMR2 |= (pTimerHandle->pTimerConfig.pwmMode<<OC3M_POS);
			pTimerHandle->pTimerX->CCER |= (1<<CC3E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC3P_POS);
			break;
		case 4:
			pTimerHandle->pTimerX->CCMR2 |= (pTimerHandle->pTimerConfig.pwmMode<<OC4M_POS);
			pTimerHandle->pTimerX->CCER |= (1<<CC4E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC4P_POS);
			break;
	}

	if(pTimerHandle->pTimerConfig.alighMode == CENTER_ALIGH_MODE_1){
		pTimerHandle->pTimerX->CR1 |= (CENTER_ALIGH_MODE_1<<CMS_POS);
	}
	else if(pTimerHandle->pTimerConfig.alighMode == CENTER_ALIGH_MODE_2){
		pTimerHandle->pTimerX->CR1 |= (CENTER_ALIGH_MODE_2<<CMS_POS);
	}
	else if(pTimerHandle->pTimerConfig.alighMode == CENTER_ALIGH_MODE_3){
		pTimerHandle->pTimerX->CR1 |= (CENTER_ALIGH_MODE_3<<CMS_POS);
	}
	else if(pTimerHandle->pTimerConfig.alighMode == EDGE_ALIGN){
		pTimerHandle->pTimerX->CR1 &= ~(CENTER_ALIGH_MODE_3<<CMS_POS);
	}

	if(pTimerHandle->pTimerConfig.counterMode == COUNTER_UP){
		pTimerHandle->pTimerX->CR1 &= ~(1<<DIR_POS);
	}
	else if(pTimerHandle->pTimerConfig.counterMode == COUNTER_DOWN){
		pTimerHandle->pTimerX->CR1 |= (1<<DIR_POS);
	}
	if(pTimerHandle->pTimerConfig.preloadCondition==PRELOAD_ENA){
		pTimerHandle->pTimerX->CR1 |= (1<<ARPE_POS);
	}
	else if(pTimerHandle->pTimerConfig.preloadCondition==PRELOAD_DIS){
		pTimerHandle->pTimerX->CR1 &= ~(1<<ARPE_POS);
	}

//	timerClock = pTimerHandle->pTimerConfig.currentClockFreq/(pTimerHandle->pTimerConfig.prescale+1);
//	arrValue = timerClock/pTimerHandle->pTimerConfig.pwmFrequency;
	/*
	 *
	 * 				Timer sẽ đếm từ 0 đến giá trị trong ARR sau đó sẽ reset
	 * 				vậy thời gian để đếm từ 0 -> ARR sẽ bằng ARR*(1/tần số của timer)
	 * 				mỗi một chu kì (1/tần số timer) thì biến đếm sẽ tăng lên 1
	 * 				Giá trị PSC dùng để chia nhỏ tần số của timer
	 * 				Giữ PSC nhỏ nhất có thể và ARR lớn và cả 2 cùng là số nguyên để tấn số pwm chính xác
	 * 				để điều chỉnh độ rộng xung (duty cycle) cần ghi giá trị vào CCR
	 *
	 * 				       Duty cyle
	 * 				CCR = ------------*(ARR+1)
	 * 						  100
	 *
	 * 				Giả sử ARR là 100 nếu muốn chỉnh duty cycle là 50% thì giá trị CCR là 50
	 *
	 * 							 Ftimer
	 *				Fpwm = -------------------
	 *						(PSC+1)*(ARR+1)
	 *
	 *
	 * 						 	Ftimer
	 * 				ARR = (----------------------) - 1
	 * 						(Prescale+1)*Fpwm
	 * */
	arrValue = ((pTimerHandle->pTimerConfig.currentClockFreq)/((pTimerHandle->pTimerConfig.prescale+1)*pTimerHandle->pTimerConfig.pwmFrequency));

	pTimerHandle->pTimerX->PSC = pTimerHandle->pTimerConfig.prescale;
	pTimerHandle->pTimerX->ARR = arrValue-1;

	pTimerHandle->pTimerX->BDTR |= (1<<MOE_POS);

	pTimerHandle->pTimerX->CR1 |= (1<<CEN_POS);
}

void PWM_SetDuty(PWM_Handle_t* pTimerHandle, uint8_t* duty){
	pTimerHandle->pTimerX->CCR[channel-1] = ((*duty)*((uint32_t)(pTimerHandle->pTimerX->ARR)+1))/100;
}
