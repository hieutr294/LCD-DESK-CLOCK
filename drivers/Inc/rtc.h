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
uint16_t rtcGetSeconds(RTC_RegDef_t* rtc);

#endif /* INC_RTC_H_ */
