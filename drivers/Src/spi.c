/*
 * spi.c
 *
 *  Created on: Feb 27, 2024
 *      Author: minhh
 */

#include "stm32f103xx.h"
#include <stdint.h>
#include "spi.h"
#include "nvic.h"
#include "stddef.h"

static void spi_tx_handling(SPI_Handle_t *pSPIHandle);
static void spi_rx_handling(SPI_Handle_t *pSPIHandle);

void SPI_ClockControl(SPI_RegDef_t *pSPIX, uint8_t condition){
	if(condition==ENABLE){
		if(pSPIX == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIX == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIX == SPI3){
			SPI3_PCLK_EN();
		}

	}else if(condition==DISABLE){
		pSPIX->CR1 &= (uint32_t)~(1<<SPI_CR1_SPE);
		if(pSPIX == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIX == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIX == SPI3){
			SPI3_PCLK_DI();
		}
	}

}

void SPI_Init(SPI_Handle_t *pSPIHandle){

//	uint32_t tempReg = pSPIHandle->pSPIX->CR1;
	pSPIHandle->pSPIX->CR1 |= (uint32_t)(pSPIHandle->pSPIConfig.SPI_CPHA<<SPI_CR1_CPHA);
	pSPIHandle->pSPIX->CR1 |= (uint32_t)(pSPIHandle->pSPIConfig.SPI_CPOL<<SPI_CR1_CPOL);
	pSPIHandle->pSPIX->CR1 |= (uint32_t)(pSPIHandle->pSPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR);
	pSPIHandle->pSPIX->CR1 |= (uint32_t)(pSPIHandle->pSPIConfig.SPI_BaudRate<<SPI_CR1_BR);
	pSPIHandle->pSPIX->CR1 |= (uint32_t)(pSPIHandle->pSPIConfig.SPI_SSM<<SPI_CR1_SSM);
	pSPIHandle->pSPIX->CR1 |= (uint32_t)(pSPIHandle->pSPIConfig.SPI_DFF<<SPI_CR1_DFF);
	pSPIHandle->pSPIX->CR2 |= (uint32_t)(1<<SPI_SSOE);

	if(pSPIHandle->pSPIConfig.SPI_BusConfig == SPI_BUS_FULL_DUPLEX){
		pSPIHandle->pSPIX->CR1 &= (uint32_t)~(1 << SPI_CR1_BIDI_MODE);
	}else if(pSPIHandle->pSPIConfig.SPI_BusConfig == SPI_BUS_HALF_DUPLEX){
		pSPIHandle->pSPIX->CR1 |= (1 << SPI_CR1_BIDI_MODE);
	}else if(pSPIHandle->pSPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX){
		pSPIHandle->pSPIX->CR1 &= (uint32_t)~(1 << SPI_CR1_BIDI_MODE);
		pSPIHandle->pSPIX->CR1 |= (1 << SPI_CR1_RX_ONLY);
	}
	nvicEnable(SPI1_GLOBAL_INTERRUPT);
	SPI_SSIConfig(SPI1,DISABLE);
	pSPIHandle->pSPIX->CR1 |= (1<<SPI_CR1_SPE);
}

void SPI_DeInit(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIX->CR1 = 0x0000;
	pSPIHandle->pSPIX->CR2 = 0x0000;
	pSPIHandle->pSPIX->DR = 0x0000;
	pSPIHandle->pSPIX->CRCPR = 0x0007;
	pSPIHandle->pSPIX->RXCRCR = 0x0000;
	pSPIHandle->pSPIX->TXCRCR = 0x0000;
	pSPIHandle->pSPIX->I2SCFGR = 0x0000;
	pSPIHandle->pSPIX->I2SPR = 0x0002;
}

void SPI_SendData(SPI_RegDef_t *pSPIX, uint8_t *pTxBuffer, int32_t len){
	uint32_t txeFlag = getBits(pSPIX->SR, SPI_SR_TXE);
	while(len > 0){
		while(txeFlag==0);
		if((pSPIX->CR1 & (1<<11))){
			pSPIX->DR = *((uint16_t*)(pTxBuffer));
			len--;
			len--;
			(uint16_t*)(pTxBuffer)++;
		}else{
			pSPIX->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIX, uint8_t condition){
	if(condition==ENABLE){
		pSPIX->CR1 |= (1<<SPI_CR1_SSI);
	}else if(condition==DISABLE){
		pSPIX->CR1 &= (uint32_t)~(1<<SPI_CR1_SSI);
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIX->CR2 |= ( 1 << SPI_TXEIE );

	}


	return state;
}

void SPI_IRQHandling(SPI_Handle_t* pSPIHandle){
	uint32_t txeFlag = getBits(pSPIHandle->pSPIX->SR, SPI_SR_TXE);
	uint32_t rxneFlag = getBits(pSPIHandle->pSPIX->SR, SPI_SR_RXNE);
	if(txeFlag){
		spi_tx_handling(pSPIHandle);
	}
	else if(rxneFlag){
		spi_rx_handling(pSPIHandle);
	}
}


static void spi_tx_handling(SPI_Handle_t *pSPIHandle){

	if((pSPIHandle->pSPIX->CR1 & (1<<11))){
		pSPIHandle->pSPIX->DR = *((uint16_t*)(pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)(pSPIHandle->pTxBuffer)++;
	}else{
		pSPIHandle->pSPIX->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen){
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}

static void spi_rx_handling(SPI_Handle_t *pSPIHandle){

}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle){
	uint8_t temp;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIX->DR;
		temp = pSPIHandle->pSPIX->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIX->CR2 &= ~(1<<SPI_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

}

uint32_t getBits(uint32_t reg, uint8_t bitNumber){

    uint32_t temp = reg;
    uint32_t result = (temp >> bitNumber) & 1;

    return result;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//This is a weak implementation . the user application may override this function.
}
