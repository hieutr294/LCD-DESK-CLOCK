/*
 * backup.c
 *
 *  Created on: Jun 6, 2025
 *      Author: minhh
 */
#include "stm32f103xx.h"
#include "backup.h"

void enableBackupReg(BKP_RegDef_t* bkp){
	RCC_config_t* rcc = RCC;
	PWR_RegDef_t* pwr = PWR;
	rcc->APB1ENR |= (1<<28) | (1<<27); //enable Power and backup register
 	pwr->PWR_CR |= (1<<8); // enable bit DBP in power register to access rtc and backup
}

void bkpWrite16(BKP_RegDef_t* bkp, uint16_t data, uint8_t regNumber){
	if(regNumber<10){
		bkp->DR1[regNumber] = data;
	}
	else{
		bkp->DR2[regNumber] = data;
	}
}

uint16_t bkpRead16(BKP_RegDef_t* bkp, uint8_t regNumber){
	uint16_t returnData;
	if(regNumber<10){
		returnData = bkp->DR1[regNumber];
	}
	else{
		returnData = bkp->DR2[regNumber];
	}
	return returnData;
}

void bkpWrite32(BKP_RegDef_t* bkp, uint32_t data, uint8_t regNumber){
	uint16_t lsb = data&0xffff;
	uint16_t msb = data>>16;
	if(regNumber<10){
		bkp->DR1[regNumber] = msb;
		bkp->DR1[regNumber+1] = lsb;
	}
	else{
		bkp->DR2[regNumber] = msb;
		bkp->DR2[regNumber+1] = lsb;
	}
}

uint32_t bkpRead32(BKP_RegDef_t* bkp, uint8_t regNumber){
	uint32_t returnData;
	uint16_t lsb = 0;
	uint16_t msb = 0;
	if(regNumber<10){
		msb = bkp->DR1[regNumber];
		lsb = bkp->DR1[regNumber+1];
		returnData = msb<<16|lsb;
	}
	else{
		msb = bkp->DR2[regNumber];
		lsb = bkp->DR2[regNumber+1];
		returnData = msb<<16|lsb;
	}
	return returnData;
}
