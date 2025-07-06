/*
 * rtc.c
 *
 *  Created on: Jun 5, 2025
 *      Author: minhh
 */
#include "rtc.h"
#include "stm32f103xx.h"
#include "stdint.h"

void rtcInit(RTC_RegDef_t* rtc){
	RCC_config_t* rcc = RCC;
	PWR_RegDef_t* pwr = PWR;
	rcc->CSR |= (1<<0); // enable LSI
	rcc->APB1ENR |= (1<<28) | (1<<27); //enable Power and backup register
	pwr->PWR_CR |= (1<<8); // enable bit DBP in power register to access rtc
	rcc->BDCR |= (1<<15) | (2<<8); // turn on rtc and select LSI as clock source
	while(!(rtc->CRL&(1<<5))); // wait for rtc is not update by checking rtoff flag
	rtc->CRL |= (1<<4); // enter config mode
	rtc->CRH |= (1<<0);// enable sec interupt
	rtc->PRLL = 39999; // config PRL is 39999 because using 40kHz LSI clock (PRL+1)/LSI = 1hz -> 1 seconds
	rtc->CRL &= ~(1<<4); // exit config mode
}

void rtcSetSeconds(RTC_RegDef_t* rtc, uint32_t value){
	while(!(rtc->CRL&(1<<5))); // wait for rtc is not update by checking rtoff flag
	rtc->CRL |= (1<<4); // enter config mode
	rtc->CNTL = (uint16_t)value&0xffff;
	rtc->CNTH = (uint16_t)(value>>16);
	rtc->CRL &= ~(1<<4); // exit config mode

}

uint32_t rtcGetSeconds(RTC_RegDef_t* rtc){
	uint16_t highBit = rtc->CNTH;
	uint16_t lowBit = rtc->CNTL;
	uint32_t result = ((uint32_t)highBit << 16) | lowBit;
	return result;
}

