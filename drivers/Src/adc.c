/*
 * adc.c
 *
 *  Created on: Mar 18, 2024
 *      Author: minhh
 */


#include <stdint.h>
#include "stm32f103xx.h"
#include "adc.h"
#include "gpio.h"

void ADC_ClockControl(ADC_RegDef_t* pADCX, uint8_t condition){
	if(condition==ENABLE){
		if(pADCX==ADC1){
			ADC1_PCLK_EN();
		}else if(pADCX==ADC2){
			ADC2_PCLK_EN();
		}else if(pADCX==ADC3){
			ADC3_PCLK_EN();
		}
	}else{
		if(pADCX==ADC1){
			ADC1_PCLK_DI();
		}else if(pADCX==ADC2){
			ADC2_PCLK_DI();
		}else if(pADCX==ADC3){
			ADC3_PCLK_DI();
		}
	}
}

void ADC_Init(ADC_Handle_t* pADCHandle){
	GPIO_Handle_t channelPin;
	if(pADCHandle->pADCConfig.conversionMode == SCAN_MODE){
		pADCHandle->pADCX->CR2 |= (1<<CONT_POS);
		pADCHandle->pADCX->CR1 |= (1<<SCAN_POS);
	}else if(pADCHandle->pADCConfig.conversionMode == SINGLE_MODE){
		pADCHandle->pADCX->CR2 &= ~(1<<CONT_POS);
		pADCHandle->pADCX->CR1 &= ~(1<<SCAN_POS);
	}

	if(pADCHandle->pADCConfig.channelGroup == REGULAR_MODE){

		pADCHandle->pADCX->SQR1 = ((pADCHandle->pADCConfig.numberChannel-1)<<REGULAR_L_POS);

		for(int i = 0; i < pADCHandle->pADCConfig.numberChannel; i++){
			if(i+1>=1 && i+1<=6){
				pADCHandle->pADCX->SQR3 |= (pADCHandle->pADCConfig.channel[i]<<(5*i));
				pADCHandle->pADCX->SMPR2 |= (pADCHandle->pADCConfig.sampleTime<<(3*i));
			}else if(i+1>=7 && i+1<=12){
				pADCHandle->pADCX->SQR2 |= (pADCHandle->pADCConfig.channel[i]<<(5*((i+1)-7)));
				if(i+1<10){
					pADCHandle->pADCX->SMPR2 |= (pADCHandle->pADCConfig.sampleTime<<(3*i));
				}else{
					pADCHandle->pADCX->SMPR1 |= (pADCHandle->pADCConfig.sampleTime<<(3*(i-10)));
				}
			}else if(i+1>=13 && i+1<=16){
				pADCHandle->pADCX->SQR1 |= (pADCHandle->pADCConfig.channel[i]<<(5*(i-13)));
				pADCHandle->pADCX->SMPR1 |= (pADCHandle->pADCConfig.sampleTime<<(3*(i-10)));
			}
		}

	}else if(pADCHandle->pADCConfig.channelGroup == INJECTED_MODE){

		pADCHandle->pADCX->JSQR |= ((pADCHandle->pADCConfig.numberChannel-1)<<INJECTED_L_POS);

		for(int i = 0; i < pADCHandle->pADCConfig.numberChannel; i++){
			pADCHandle->pADCX->SQR3 |= (i<<(5*i));
			pADCHandle->pADCX->SMPR2 |= (pADCHandle->pADCConfig.sampleTime<<(3*i));
		}

	}

	if(pADCHandle->pADCConfig.dmaEnable == ENABLE){
		pADCHandle->pADCX->CR2 |= (1<<DMA_POS);
	}else if(pADCHandle->pADCConfig.dmaEnable == DISABLE){
		pADCHandle->pADCX->CR2 &= ~(1<<DMA_POS);
	}

	for(int i = 0; i < pADCHandle->pADCConfig.numberChannel; i++){
		if(pADCHandle->pADCConfig.channel[i]<=8){
			channelPin.pGPIOX = GPIOA;
			channelPin.GPIO_PinConfig.pinMode = ANALOG_INPUT;
			channelPin.GPIO_PinConfig.pinNumber = pADCHandle->pADCConfig.channel[i];
			GPIO_ClockControl(GPIOA,ENABLE);
			GPIO_Init(&channelPin);
		}else if(pADCHandle->pADCConfig.channel[i]<=11){
			channelPin.pGPIOX = GPIOB;
			channelPin.GPIO_PinConfig.pinMode = ANALOG_INPUT;
			channelPin.GPIO_PinConfig.pinNumber = pADCHandle->pADCConfig.channel[i];
			GPIO_ClockControl(GPIOB,ENABLE);
			GPIO_Init(&channelPin);
		}else if(pADCHandle->pADCConfig.channel[i]<=16){
			channelPin.pGPIOX = GPIOC;
			channelPin.GPIO_PinConfig.pinMode = ANALOG_INPUT;
			channelPin.GPIO_PinConfig.pinNumber = pADCHandle->pADCConfig.channel[i];
			GPIO_ClockControl(GPIOC,ENABLE);
			GPIO_Init(&channelPin);
		}
	}
}

void ADC_Start(ADC_Handle_t* pADCHandle){
	if(pADCHandle->pADCConfig.channelGroup == REGULAR_MODE){
		pADCHandle->pADCX->SR = 0;
		pADCHandle->pADCX->CR2 |= (1<<ADON_POS);
		pADCHandle->pADCX->CR2 |= (1<<ADON_POS);
		pADCHandle->pADCX->CR2 |= (1<<CAL_POS);
		while(((pADCHandle->pADCX->CR2 >> CAL_POS) & 1) != 0 );
		pADCHandle->pADCX->CR2 |= (1<<SWSTART_POS);
	}else if(pADCHandle->pADCConfig.channelGroup == INJECTED_MODE){
		pADCHandle->pADCX->CR2 |= (1<<ADON_POS);
		pADCHandle->pADCX->CR2 |= (1<<ADON_POS);
		pADCHandle->pADCX->CR2 |= (1<<CAL_POS);
		while((pADCHandle->pADCX->CR2 & ~(1<<CAL_POS) >> CAL_POS) != 0 );
		pADCHandle->pADCX->CR2 |= (1<<JSWSTART_POS);
	}
}

uint16_t ADC_ReadData(ADC_Handle_t* pADCHandle){
	return (pADCHandle->pADCX->DR);
}
