/*
 * uart.c
 *
 *  Created on: Jun 9, 2024
 *      Author: minhh
 */
#include "stm32f103xx.h"
#include <stdint.h>
#include "uart.h"
#include "gpio.h"


static uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTX, uint8_t FlagName){
	if(pUSARTX->SR && FlagName){
		return FLAG_SET;
	}else{
		return FLAG_RESET;
	}
}

void USART_ClockControl(USART_RegDef_t* pUSARTX){
	if(pUSARTX==USART1){
		USART1_PCLK_EN();
	}else if(pUSARTX==USART2){
		USART2_PCLK_EN();
	}else if(pUSARTX==USART3){
		USART3_PCLK_EN();
	}
}

void __UART_GPIO_PIN(){
	GPIO_Handle_t txPin;
	GPIO_Handle_t rxPin;

	txPin.GPIO_PinConfig.pinNumber = 9;
	txPin.GPIO_PinConfig.pinMode = GPIO_MODE_AF_PUSH_PULL_50MHZ;
	txPin.pGPIOX = GPIOA;

	rxPin.GPIO_PinConfig.pinNumber = 10;
	rxPin.GPIO_PinConfig.pinMode = INPUT_PUPD;
	rxPin.pGPIOX = GPIOA;

	GPIO_ClockControl(GPIOA, ENABLE);
	GPIO_Init(&txPin);
	GPIO_Init(&rxPin);
}

void USART_SetBaud(USART_Handle_t* pUSARTHandle){
	float usartDiv = (float)(((float)8000000)/((float)16*(float)pUSARTHandle->pConfig.USART_Baud))*100;
	uint16_t mantissa = (uint16_t)usartDiv/100;
	uint8_t fraction = (uint8_t)(16*((float)((uint16_t)usartDiv%100)/100));

	pUSARTHandle->pUSARTX->BRR |= (mantissa<<4);
	pUSARTHandle->pUSARTX->BRR |= (fraction<<0);
}

void USART_Init(USART_Handle_t* pUSARTHandle){
	__UART_GPIO_PIN();

	if(pUSARTHandle->pConfig.USART_Mode == USART_MODE_ONLY_RX){
		pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_RE);
		pUSARTHandle->pUSARTX->CR1 &= ~(1<<CR1_TE);
	}else if(pUSARTHandle->pConfig.USART_Mode == USART_MODE_ONLY_TX){
		pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_TE);
		pUSARTHandle->pUSARTX->CR1 &= ~(1<<CR1_RE);
	}else if(pUSARTHandle->pConfig.USART_Mode == USART_MODE_TXRX){
		pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_RE);
		pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_TE);
	}

	if(pUSARTHandle->pConfig.USART_ParityControl != USART_PARITY_DISABLE){
		pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_PCE);
		if(pUSARTHandle->pConfig.USART_ParityControl == USART_PARITY_EN_EVEN){
			pUSARTHandle->pUSARTX->CR1 &= ~(1<<CR1_PS);
		}else if(pUSARTHandle->pConfig.USART_ParityControl == USART_PARITY_EN_ODD){
			pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_PS);
		}
	}else{
		pUSARTHandle->pUSARTX->CR1 &= ~(1<<CR1_PCE);
	}

	if(pUSARTHandle->pConfig.USART_WordLength == USART_WORDLEN_8BITS){
		pUSARTHandle->pUSARTX->CR1 &= ~(1<<CR1_M);
	}else if(pUSARTHandle->pConfig.USART_WordLength == USART_WORDLEN_9BITS){
		pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_M);
	}

	pUSARTHandle->pUSARTX->CR2 |= (pUSARTHandle->pConfig.USART_NoOfStopBits<<CR2_STOP);

	if(pUSARTHandle->pConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_NONE){
		pUSARTHandle->pUSARTX->CR3 &= ~(1<<CR3_CTSE);
		pUSARTHandle->pUSARTX->CR3 &= ~(1<<CR3_RTSE);
	}else if(pUSARTHandle->pConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		pUSARTHandle->pUSARTX->CR3 |= (1<<CR3_CTSE);
		pUSARTHandle->pUSARTX->CR3 &= ~(1<<CR3_RTSE);
	}else if(pUSARTHandle->pConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		pUSARTHandle->pUSARTX->CR3 |= (1<<CR3_RTSE);
		pUSARTHandle->pUSARTX->CR3 &= ~(1<<CR3_CTSE);
	}else if(pUSARTHandle->pConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		pUSARTHandle->pUSARTX->CR3 |= (1<<CR3_RTSE);
		pUSARTHandle->pUSARTX->CR3 |= (1<<CR3_CTSE);
	}

	USART_SetBaud(pUSARTHandle);
	pUSARTHandle->pUSARTX->CR3 |= (1<<CR3_DMAR);//Enable dma
	pUSARTHandle->pUSARTX->CR1 |= (1<<CR1_UE);
}

void USART_SendData(USART_Handle_t* pUSARTHandle, uint16_t* pBuffer, uint32_t len){

	for(int i = 0; i < len; i++){
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTX, SR_TXE));
		if(pUSARTHandle->pConfig.USART_WordLength == USART_WORDLEN_9BITS){
			pUSARTHandle->pUSARTX->DR = (*pBuffer & (uint16_t)0x01ff);
			if(pUSARTHandle->pConfig.USART_ParityControl == USART_PARITY_DISABLE){
				pBuffer++;
				pBuffer++;
			}else{
				pBuffer++;
			}

		}else{
			pUSARTHandle->pUSARTX->DR = (*pBuffer & (uint8_t)0xff);
			pBuffer++;
		}
	}
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTX, SR_TC));
}

void USART_ReciveData(USART_Handle_t* pUSARTHandle, uint8_t* pBuffer, uint8_t numberOfdata){
	uint32_t dummyRead = 0;
	for(int j = 0; j < numberOfdata; j++){
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTX, SR_RXNE));
//		if (USART_GetFlagStatus(pUSARTHandle->pUSARTX, SR_ORE)) {
//			volatile uint32_t tmp;
//			tmp = pUSARTHandle->pUSARTX->SR;  // Đọc SR
//			tmp = pUSARTHandle->pUSARTX->DR;  // Đọc DR để xóa lỗi
//			(void)tmp;  // Tránh cảnh báo
//		}
		*pBuffer = (pUSARTHandle->pUSARTX->DR);
		pBuffer++;
	}
	(void)dummyRead;
}
