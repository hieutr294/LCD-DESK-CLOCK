/*
 * dma.h
 *
 *  Created on: Mar 20, 2024
 *      Author: minhh
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include <stdint.h>
#include "stm32f103xx.h"

#define EN_POS 0
#define TCIE_POS 1
#define HTIE_POS 2
#define TEIE_POS 3
#define DIR_POS 4
#define CIRC_POS 5
#define PINC_POS 6
#define MINC_POS 7
#define PSIZE_POS 8
#define MSIZE_POS 10
#define PL_POS 12
#define MEM2MEM_POS 14

#define MEM2MEM 1
#define PERI2MEM 2

#define LOW_PRIORITY 0
#define MEDIUM_PRIORITY 1
#define HIGH_PRIORITY 2
#define VERY_HIGH_PRIORITY 3

#define MEMSIZE_8 0
#define MEMSIZE_16 1
#define MEMSIZE_32 2

#define PSIZE_8 0
#define PSIZE_16 1
#define PSIZE_32 2

#define MINC_ENA 1
#define PINC_ENA 1
#define MINC_DIS 0
#define PINC_DIS 0

#define CIRC_ENA 1
#define CIRC_DIS 0

#define READ_FROM_MEM 1
#define READ_FROM_PERI 0

#define TEIE_ENA 1
#define HTIE_ENA 1
#define TCIE_ENA 1

#define TRANSFER_COMPLETE_INTERUPT 1
#define HALF_TRANSFER_INTERUPT 1

#define ENABLE_CHANNEL 1

typedef struct{
	uint8_t channel; // 7 channels for DMA1 and 5 for DMA2
	uint8_t mode; // MEM2MEM or PERI2MEM
	uint8_t channelPriority; // Low -> medium -> high -> very high
	uint8_t memIncrement; // MINC_ENA use for array
	uint8_t peripheralIncrement; // PINC_ENA
	uint8_t circularMode; // use to handle circular buffer and continous data flows ( page 278 )
	uint8_t dataDirection; // READ_FROM_MEM or READ_FROM_PERI
	uint8_t interupt;
}DMA_Config_t;

typedef struct{
	DMA_RegDef_t* pDMAX;
	DMA_Config_t pDMAConfig;
}DMA_Handle_t;

void DMA_ClockControl(DMA_RegDef_t* pDMAX, uint8_t condition);

void DMA_Init(DMA_Handle_t* pDMAHandle);

void DMA_SetReciveMemoryAddress(DMA_Handle_t* pDMAHandle, uint8_t memSize, uint8_t* memoryAddress);

void DMA_SetPeripheralAddress(DMA_Handle_t* pDMAHandle, uint8_t periSize, uint32_t* peripheralAddress);

void DMA_EnableChannel(DMA_Handle_t* pDMAHandle);

void DMA_DisableChannel(DMA_Handle_t* pDMAHandle);

void DMA_DataFlow(DMA_Handle_t* pDMAHandle, volatile uint32_t* from,volatile uint8_t* to, uint32_t size);

void DMA_SetNumberOfByteData(DMA_Handle_t* pDMAHandle, uint8_t size);
#endif /* INC_DMA_H_ */
