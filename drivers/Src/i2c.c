/*
 * i2c.c
 *
 *  Created on: Apr 17, 2024
 *      Author: minhh
 */

#include <stdint.h>
#include "stm32f103xx.h"
#include "i2c.h"
#include "gpio.h"
#include "nvic.h"

volatile uint32_t sbEvent = 0;
volatile uint32_t addrEvent = 0;
volatile uint32_t txeEvent = 0;
volatile uint32_t btfEvent = 0;

static void __I2C_ConfigPin(){
	GPIO_Handle_t scl;
	GPIO_Handle_t sda;
	scl.pGPIOX = GPIOB;
	sda.pGPIOX = GPIOB;
	scl.GPIO_PinConfig.pinNumber = 6;
	scl.GPIO_PinConfig.pinMode = GPIO_MODE_AF_OPEN_DRAIN_50MHZ;
	sda.GPIO_PinConfig.pinNumber = 7;
	sda.GPIO_PinConfig.pinMode = GPIO_MODE_AF_OPEN_DRAIN_50MHZ;
	GPIO_ClockControl(GPIOB, ENABLE);
	GPIO_Init(&scl);
	GPIO_Init(&sda);
}

void I2C_ClockControl(I2C_RegDef_t* pI2Cx, uint8_t condition){
	if(condition == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
	}else if(condition == DISABLE){
		if(pI2Cx==I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
	}
}

void I2C_Init(I2C_Handle_t* pI2CHandle){
	__I2C_ConfigPin();
	//Reset i2c before start
	pI2CHandle->pI2Cx->CR1 |= (1<<CR1_SWRST);
	pI2CHandle->pI2Cx->CR1 &= ~(1<<CR1_SWRST);

	pI2CHandle->pI2Cx->CR1 |= (1<<CR1_PE);

	if(pI2CHandle->pI2CConfig.I2C_SCLSpeed == I2C_SCL_SPEED_SM){

	    /* As the datasheet
	     * Thigh = CCR * Tpclk
	     * Tlow = CCR * Tpclk
	     * Because Thigh = Tlow
	     * => Tscl = Thigh + Tlow = 2*CCR*Tpclsk
	     * convert to frequency
	     * => CCR = FPCLK / (2*FSCL)
	     *  */

	    uint32_t fPCLK = 8000000;
		uint32_t ccrValue = fPCLK/(2*pI2CHandle->pI2CConfig.I2C_SCLSpeed);

		pI2CHandle->pI2Cx->CCR &= ~(1<<CCR_FS);
		pI2CHandle->pI2Cx->CR2 |= (8<<CR2_FREQ);
		pI2CHandle->pI2Cx->CCR |= (ccrValue<<CCR_CCR);

	}else if(pI2CHandle->pI2CConfig.I2C_SCLSpeed == I2C_SCL_SPEED_FM){
		pI2CHandle->pI2Cx->CCR |= (1<<CCR_FS);
		if(pI2CHandle->pI2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2){

		    /* As the datasheet FM mode and tlow/thigh = 2
		     * Thigh = CCR * Tpclk
		     * Tlow = 2*CCR * Tpclk
		     * Because Thigh = 2*Tlow
		     * => Tscl = Thigh + Tlow = 3*CCR*Tpclsk
		     * convert to frequency
		     * => CCR = FPCLK / (3*FSCL)
		     *  */

		    uint32_t fPCLK = 8000000;
			uint32_t ccrValue = fPCLK/(3*pI2CHandle->pI2CConfig.I2C_SCLSpeed);

			pI2CHandle->pI2Cx->CCR &= ~(1<<CCR_DUTY);
			pI2CHandle->pI2Cx->CR2 |= (8<<CR2_FREQ);
			pI2CHandle->pI2Cx->CCR |= (ccrValue<<CCR_CCR);

		}else if(pI2CHandle->pI2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_16_9){

		    /* As the datasheet FM mode and tlow/thigh = 16/9
		     * Thigh = 9*CCR * Tpclk
		     * Tlow = 16*CCR * Tpclk
		     * => Tscl = 9*Thigh + 16*Tlow = 25*CCR*Tpclsk
		     * convert to frequency
		     * => CCR = FPCLK / (25*FSCL)
		     *  */

		    uint32_t fPCLK = 8000000;
			uint32_t ccrValue = fPCLK/(25*pI2CHandle->pI2CConfig.I2C_SCLSpeed);

			pI2CHandle->pI2Cx->CCR |= (1<<CCR_DUTY);
			pI2CHandle->pI2Cx->CR2 |= (8<<CR2_FREQ);
			pI2CHandle->pI2Cx->CCR |= (ccrValue<<CCR_CCR);
		}

	}

	// Config TRISE
	if(pI2CHandle->pI2CConfig.I2C_SCLSpeed == I2C_SCL_SPEED_SM){
//		uint32_t triseValue = ((8000000/100000)+1);
		pI2CHandle->pI2Cx->TRISE |= (9<<TRISE_POS);
	}else if(pI2CHandle->pI2CConfig.I2C_SCLSpeed == I2C_SCL_SPEED_FM){
		pI2CHandle->pI2Cx->TRISE |= ((((8000000/100000)*300)+1)<<TRISE_POS);
	}


//	pI2CHandle->pI2Cx->OAR1 |= ((pI2CHandle->pI2CConfig.I2C_DeviceAddress<<1)<<OAR1_ADD);
//	pI2CHandle->pI2Cx->OAR1 |= (1<<14); // bit 14 always set by software (datasheet)

}

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle,uint8_t* pBuffer, uint32_t len, uint8_t slaveAddress){
	while(len>0){
		pI2CHandle->pI2Cx->DR = *pBuffer;
		pBuffer++;
		len--;
	}
}

uint8_t I2C_CheckStatus(I2C_Handle_t* pI2CHandle){
	return pI2CHandle->pI2CConfig.I2C_RxTxState;
}

void I2C_MasterSendIT(I2C_Handle_t* pI2CHandle, uint16_t* data, uint8_t len, uint8_t address){
	uint8_t state = pI2CHandle->pI2CConfig.I2C_RxTxState;

	if(state!=I2C_BUSY_TX && state!=I2C_BUSY_RX){
		pI2CHandle->pI2CConfig.I2C_SendData = data;
		pI2CHandle->pI2CConfig.I2C_DataLength = len;
		pI2CHandle->pI2CConfig.I2C_SlaveAddress = address;
		pI2CHandle->pI2CConfig.I2C_RxTxState = I2C_BUSY_TX;

		nvicEnable(I2C1_EV_IRQ_NUMBER);
		nvicSetPriority(31, 1);

		I2C_GenerateStartCondition(I2C1);

		pI2CHandle->pI2Cx->CR2 |= (1<<CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= (1<<CR2_ITBUFEN);



	}
}

void I2C_MasterReciveIT(I2C_Handle_t* pI2CHandle, uint16_t* reciveData, uint8_t byteRecive, uint8_t address){
	uint8_t state = pI2CHandle->pI2CConfig.I2C_RxTxState;

	if(state!=I2C_BUSY_TX && state!=I2C_BUSY_RX){
		pI2CHandle->pI2CConfig.I2C_ReciveData = reciveData;
		pI2CHandle->pI2CConfig.I2C_SlaveAddress = address;
		pI2CHandle->pI2CConfig.I2C_ByteRecive = byteRecive;
		pI2CHandle->pI2CConfig.I2C_RxTxState = I2C_BUSY_RX;

		nvicEnable(I2C1_EV_IRQ_NUMBER);
		nvicSetPriority(31, 1);

		I2C_GenerateStartCondition(I2C1);

		pI2CHandle->pI2Cx->CR1 |= (1<<CR1_ACK);

		pI2CHandle->pI2Cx->CR2 |= (1<<CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= (1<<CR2_ITBUFEN);

	}
}

void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx){

	pI2Cx->CR1 |= (1<<CCR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx){
	pI2Cx->CR1 |= (1<<CCR1_STOP);
}

static void I2C_AddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t slaveAddress){
	slaveAddress = slaveAddress << 1;
	slaveAddress &= ~(1);
	pI2Cx->DR = slaveAddress;
}

static void I2C_AddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t slaveAddress){
	slaveAddress = slaveAddress << 1;
	slaveAddress |= 1;
	pI2Cx->DR = slaveAddress;
}

uint8_t I2C_GetFlagStatus(uint32_t reg, uint8_t flag){
	if(reg & flag){
		return FLAG_SET;
	}else{
		return FLAG_RESET;
	}
}

void I2C_InteruptHandling( I2C_Handle_t* pI2CHandle){
	uint32_t sr1 = pI2CHandle->pI2Cx->SR1;

	//SB event
	if(sr1 & I2C_FLAG_SB){
		if(pI2CHandle->pI2CConfig.I2C_RxTxState==I2C_BUSY_TX){
			I2C_AddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->pI2CConfig.I2C_SlaveAddress);
		}else if(pI2CHandle->pI2CConfig.I2C_RxTxState==I2C_BUSY_RX){
			I2C_AddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->pI2CConfig.I2C_SlaveAddress);
		}
	}

	//ADDR event
	else if(sr1 & I2C_FLAG_ADDR){
		if(pI2CHandle->pI2CConfig.I2C_RxTxState == I2C_BUSY_RX && pI2CHandle->pI2CConfig.I2C_ByteRecive==1){
			I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
			pI2CHandle->pI2Cx->CR1 &= ~(1 << CR1_ACK);    // Clear ACK
			pI2CHandle->pI2Cx->CR1 |= (1 << CCR1_STOP);     // Set STOP
		}else{
			I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
		}
	}

	//TXE event
	else if(sr1 & I2C_FLAG_TXE){

		if(pI2CHandle->pI2CConfig.I2C_DataLength > 0){
			pI2CHandle->pI2Cx->DR = *pI2CHandle->pI2CConfig.I2C_SendData;
			pI2CHandle->pI2CConfig.I2C_SendData++;
			pI2CHandle->pI2CConfig.I2C_DataLength--;
		} else {
			I2C_GenerateStopCondition(I2C1);
			pI2CHandle->pI2Cx->CR2 &= ~(1<<CR2_ITEVTEN);
			pI2CHandle->pI2Cx->CR2 &= ~(1<<CR2_ITBUFEN);

			pI2CHandle->pI2CConfig.I2C_RxTxState = I2C_TX_DONE;
			pI2CHandle->pI2CConfig.I2C_CallBack(pI2CHandle->pI2CConfig.I2C_RxTxState);
		}

	}

	//RXE event
	else if(sr1 & I2C_FLAG_RXE) {

	    if (pI2CHandle->pI2CConfig.I2C_ByteRecive == 1)
	    {
	        // Trường hợp chỉ nhận 1 byte
	        *(pI2CHandle->pI2CConfig.I2C_ReciveData) = pI2CHandle->pI2Cx->DR;
	        pI2CHandle->pI2CConfig.I2C_ByteRecive--;
	        pI2CHandle->pI2CConfig.I2C_RxTxState = I2C_RX_DONE;
	        pI2CHandle->pI2CConfig.I2C_CallBack(I2C_RX_DONE);
	    }

	    else if (pI2CHandle->pI2CConfig.I2C_ByteRecive > 1)
	    {
	        if (pI2CHandle->pI2CConfig.I2C_ByteRecive == 2)
	        {
	            // Khi còn 2 byte: chuẩn bị kết thúc giao tiếp
	            // Bước quan trọng: Clear ACK và set STOP

	            pI2CHandle->pI2Cx->CR1 &= ~(1 << CR1_ACK);    // Clear ACK
	            pI2CHandle->pI2Cx->CR1 |= (1 << CCR1_STOP);     // Set STOP
	        }

	        // Đọc dữ liệu từ DR vào buffer
	        *(pI2CHandle->pI2CConfig.I2C_ReciveData) = pI2CHandle->pI2Cx->DR;
	        pI2CHandle->pI2CConfig.I2C_ReciveData++;
	        pI2CHandle->pI2CConfig.I2C_ByteRecive--;

	        if (pI2CHandle->pI2CConfig.I2C_ByteRecive == 0)
	        {
	            // Đã nhận xong hết các byte
	        	pI2CHandle->pI2CConfig.I2C_RxTxState = I2C_RX_DONE;
	        }
	    }

	}

}

void I2C_ACKManage(I2C_RegDef_t* pI2Cx, uint8_t condition){
	if(condition==ENABLE){
		pI2Cx->CR1 |= (1 << CR1_ACK);
	}else if(condition==DISABLE){
		pI2Cx->CR1 &= ~(1 << CR1_ACK);
	}

}

static void I2C_ClearAddrFlag(I2C_RegDef_t* pI2Cx){
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
//	(void)dummyRead;
}

void I2C_MasterRecivePolling(I2C_Handle_t* pI2CHandle, uint16_t* reciveData, uint8_t byteRecive, uint8_t address){

	if(byteRecive>2){

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_SB));
		I2C_AddressPhaseRead(pI2CHandle->pI2Cx, address);
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_ADDR));
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
		while(byteRecive>0){
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_RXE));
			*reciveData = pI2CHandle->pI2Cx->DR;
			reciveData++;
			byteRecive--;
			if(byteRecive==2){
				I2C_ACKManage(pI2CHandle->pI2Cx, DISABLE);
	            *reciveData = pI2CHandle->pI2Cx->DR;
				reciveData++;
				byteRecive--;
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	            pI2CHandle->pI2Cx->CR1 |= (1 << CCR1_STOP);
	            *reciveData = pI2CHandle->pI2Cx->DR;
				reciveData++;
				byteRecive--;
			}
		}
	}else if(byteRecive==2){
		pI2CHandle->pI2Cx->CR1 |= (1<<CR1_POS);
		I2C_ACKManage(pI2CHandle->pI2Cx, ENABLE);
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_SB));
		I2C_AddressPhaseRead(pI2CHandle->pI2Cx, address);
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_ADDR));
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
		I2C_ACKManage(pI2CHandle->pI2Cx, DISABLE);
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_BTF));
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		*reciveData = pI2CHandle->pI2Cx->DR;
		reciveData++;
		*reciveData = pI2CHandle->pI2Cx->DR;
	}else if(byteRecive==1){
		;
	}
}

void I2C_MasterSendPolling(I2C_Handle_t* pI2CHandle, uint16_t* sendData, uint8_t byteSend, uint8_t address){
	uint8_t byteSend2 = byteSend+1;
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_SB));
	I2C_AddressPhaseWrite(pI2CHandle->pI2Cx, address);
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_ADDR));
	I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
	while(byteSend2>0){
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx->SR1, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *sendData;
		byteSend2--;
		sendData++;
	}
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}
