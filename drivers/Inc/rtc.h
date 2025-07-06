/*
 * rtc.h
 *
 *  Created on: Jun 5, 2025
 *      Author: minhh
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "stm32f103xx.h"

void rtcInit(RTC_RegDef_t* rtc);
uint32_t rtcGetSeconds(RTC_RegDef_t* rtc);
void rtcSetSeconds(RTC_RegDef_t* rtc, uint32_t value);

#endif /* INC_RTC_H_ */
