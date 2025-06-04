/*
 * spi.h
 *
 *  Created on: Feb 27, 2024
 *      Author: minhh
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32f103xx.h"
#include <stdint.h>


#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSB_FIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RX_ONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRC_NEXT				12
#define SPI_CR1_CRC_EN					13
#define SPI_CR1_BIDI_OE					14
#define SPI_CR1_BIDI_MODE				15

#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRC_ERROR				4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7

#define SPI_RXDMAEN						0
#define SPI_TXDMAEN						1
#define SPI_SSOE						2
#define SPI_ERRIE						5
#define SPI_RXNEIE						6
#define SPI_TXEIE						7

#define SPI_RXNE						0
#define SPI_TXNE						1
#define SPI_CHSIDE						2
#define SPI_UDR							3
#define SPI_CRC_ERR						4
#define SPI_MODR						5
#define SPI_OVR							6
#define SPI_BSY							7

//#define FLAG_SET						(uint8_t)1
//#define FLAG_RESET						(uint8_t)0

#define RXNE_FLAG						(1<<SPI_RXNE)
#define TXNE_FLAG						(1<<SPI_TXNE)
#define CHSIDE_FLAG						(1<<SPI_CHSIDE)
#define UDR_FLAG						(1<<SPI_UDR)
#define CRC_ERR_FLAG					(1<<SPI_CRC_ERR)
#define OVR_FLAG						(1<<SPI_OVR)
#define BSY_FLAG						(1<<SPI_BSY)
#define MODR_FLAG						(1<<SPI_MODR)

#define SPI_SLAVE 						0
#define SPI_MASTER 						1
#define SPI_CPOL_HI 					1
#define SPI_CPOL_LO 					0
#define SPI_CPHA_HI						1
#define SPI_CPHA_LO 					0

#define SPI_BAUD_PCLK_DIV2 				0
#define SPI_BAUD_PCLK_DIV4 				1
#define SPI_BAUD_PCLK_DIV8 				2
#define SPI_BAUD_PCLK_DIV16 			3
#define SPI_BAUD_PCLK_DIV32 			4
#define SPI_BAUD_PCLK_DIV64 			5
#define SPI_BAUD_PCLK_DIV128 			6
#define SPI_BAUD_PCLK_DIV256 			7

#define SPI_SSM_DI 						0
#define SPI_SSM_EN 						1

#define SPI_DFF_8						0
#define SPI_DFF_16 						1

#define SPI_BUS_FULL_DUPLEX				1
#define SPI_BUS_HALF_DUPLEX				2
#define SPI_BUS_SIMPLEX_RX				3

#define SPI_READY 						0
#define SPI_BUSY_IN_TX 					1
#define SPI_BUSY_IN_RX 					2

#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERROR				3

#define SPI_SOFTWARE_SM					1
#define SPI_HARDWARE_SM 				0

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_BaudRate;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIX;
	SPI_Config_t pSPIConfig;

	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;

	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;

uint32_t getBits(uint32_t reg, uint8_t bitNumber);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);
void SPI_SSIConfig(SPI_RegDef_t *pSPIX, uint8_t condition);
void SPI_ClockControl(SPI_RegDef_t *pSPIX, uint8_t condition);

void SPI_SendData(SPI_RegDef_t *pSPIX, uint8_t *pTxBuffer, int32_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);
#endif /* INC_SPI_H_ */
