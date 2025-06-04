/*
 * dma.c
 *
 *  Created on: Mar 20, 2024
 *      Author: minhh
 */

#include <stdint.h>
#include "dma.h"
#include "stm32f103xx.h"
#include "nvic.h"

void DMA_ClockControl(DMA_RegDef_t* pDMAX, uint8_t condition){
	if(condition==ENABLE){
		if(pDMAX==DMA1){
			DMA1_PCLK_EN();
		}else if(pDMAX==DMA2){
			DMA2_PCLK_EN();
		}
	}else{
		if(pDMAX==DMA1){
			DMA1_PCLK_DI();
		}else if(pDMAX==DMA2){
			DMA2_PCLK_DI();
		}
	}
}

uint32_t __DMAChannelAddressCal(uint32_t* address, uint8_t channel){
	return ((uint32_t)address)+20*(channel-1);
}

void DMA_Init(DMA_Handle_t* pDMAHandle){

	uint32_t* newCCRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CCR,pDMAHandle->pDMAConfig.channel);
	uint32_t* newCNDTRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CNDTR,pDMAHandle->pDMAConfig.channel);

	if(pDMAHandle->pDMAConfig.mode == MEM2MEM){
		*newCCRAddress |= (1<<MEM2MEM_POS);
	}
	else if(pDMAHandle->pDMAConfig.mode == PERI2MEM){
		*newCCRAddress &= ~(1<<MEM2MEM_POS);
	}

	*newCCRAddress |= (pDMAHandle->pDMAConfig.channelPriority<<PL_POS);

	if(pDMAHandle->pDMAConfig.circularMode == CIRC_ENA){
		*newCCRAddress |= (pDMAHandle->pDMAConfig.circularMode<<CIRC_POS);
	}
	else if(pDMAHandle->pDMAConfig.circularMode == CIRC_DIS){
		*newCCRAddress &= ~(1<<CIRC_POS);
	}

	if(pDMAHandle->pDMAConfig.dataDirection == READ_FROM_MEM){
		*newCCRAddress |= (pDMAHandle->pDMAConfig.dataDirection<<DIR_POS);
	}
	else if(pDMAHandle->pDMAConfig.dataDirection == READ_FROM_PERI){
		*newCCRAddress &= ~(1<<DIR_POS);
	}

	if(pDMAHandle->pDMAConfig.memIncrement == MINC_ENA){
		*newCCRAddress |= (1<<MINC_POS);
	}
	else if(pDMAHandle->pDMAConfig.memIncrement == MINC_DIS){
		*newCCRAddress &= ~(1<<MINC_POS);
	}

	if(pDMAHandle->pDMAConfig.peripheralIncrement == PINC_ENA){
		*newCCRAddress |= (1<<PINC_POS);
	}
	else if(pDMAHandle->pDMAConfig.peripheralIncrement == PINC_DIS){
		*newCCRAddress &= ~(1<<PINC_POS);
	}

	if(pDMAHandle->pDMAConfig.interupt == TRANSFER_COMPLETE_INTERUPT){
		*newCCRAddress |= (1<<TCIE_POS);
		nvicEnable(DMA1_CHANNEL5);
		nvicSetPriority(DMA1_CHANNEL5, 22);
	}
	else if(pDMAHandle->pDMAConfig.peripheralIncrement == HALF_TRANSFER_INTERUPT){
		*newCCRAddress |= (1<<HTIE_ENA);
		nvicEnable(DMA1_CHANNEL5);
		nvicSetPriority(DMA1_CHANNEL5, 22);
	}
}

void DMA_EnableChannel(DMA_Handle_t* pDMAHandle){
	uint32_t* newCCRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CCR,pDMAHandle->pDMAConfig.channel);
	*newCCRAddress |= (1<<EN_POS);
}

void DMA_DisableChannel(DMA_Handle_t* pDMAHandle){
	uint32_t* newCCRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CCR,pDMAHandle->pDMAConfig.channel);
	*newCCRAddress &= ~(1<<EN_POS);
}

void DMA_SetReciveMemoryAddress(DMA_Handle_t* pDMAHandle, uint8_t memSize, uint8_t* memoryAddress){
	uint32_t* newCCRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CCR,pDMAHandle->pDMAConfig.channel);
	uint32_t* newCMARAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CMAR,pDMAHandle->pDMAConfig.channel);

	*newCCRAddress |= (memSize<<MSIZE_POS);
	*newCMARAddress = memoryAddress;
}

void DMA_SetPeripheralAddress(DMA_Handle_t* pDMAHandle, uint8_t periSize, uint32_t* peripheralAddress){
	uint32_t* newCCRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CCR,pDMAHandle->pDMAConfig.channel);
	uint32_t* newCPARAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CPAR,pDMAHandle->pDMAConfig.channel);

	*newCCRAddress |= (periSize<<PSIZE_POS);
	*newCPARAddress = peripheralAddress;
}

void DMA_SetNumberOfByteData(DMA_Handle_t* pDMAHandle, uint8_t size){
	uint32_t* newCNDTRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CNDTR,pDMAHandle->pDMAConfig.channel);
	*newCNDTRAddress = size;
}

void DMA_DataFlow(DMA_Handle_t* pDMAHandle, volatile uint32_t* from, volatile uint8_t* to, uint32_t numberOfByte){
	uint32_t* newCCRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CCR,pDMAHandle->pDMAConfig.channel);
	uint32_t* newCPARAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CPAR,pDMAHandle->pDMAConfig.channel);
	uint32_t* newCNDTRAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CNDTR,pDMAHandle->pDMAConfig.channel);
	uint32_t* newCMARAddress = (uint32_t*)__DMAChannelAddressCal(&pDMAHandle->pDMAX->CMAR,pDMAHandle->pDMAConfig.channel);
	*newCNDTRAddress = numberOfByte;
	*newCCRAddress |= (0<<PSIZE_POS);
	*newCCRAddress |= (0<<MSIZE_POS);
	*newCPARAddress = (uint32_t)from;
	*newCMARAddress = (uint32_t)to;
}
