/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/*
	I2C pin
	SCL -> B6
	SDA -> B7
	Config as alternate function open-drain
 */

#include <stdint.h>
#include "stm32f103xx.h"
#include "rtc.h"
#include "backup.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

BKP_RegDef_t* bkp = BKP;
uint16_t read = 0;
uint32_t read2 = 54934569;
uint32_t value = 0;
int main(void)
{
	enableBackupReg(bkp);
	bkpWrite32(bkp, read2, 0);
	while(1){
		value = bkpRead32(bkp, 0);
	}
}
