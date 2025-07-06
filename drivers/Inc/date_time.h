/*
 * date.h
 *
 *  Created on: Jul 6, 2025
 *      Author: minhh
 */

#ifndef INC_DATE_TIME_H_
#define INC_DATE_TIME_H_

#include "stdint.h"

typedef struct{
	uint32_t unixTime;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	uint8_t hour;
	uint8_t minute;
	uint8_t seconds;
}Date;

Date getDate(Date* date);
Date getTime(Date* date);

uint8_t getDay(Date* date);
uint8_t getMonth(Date* date);
uint32_t getYear(Date* date);

uint8_t getHour(Date* date);
uint8_t getMinute(Date* date);
uint8_t getSeconds(Date* date);

void setDay(uint8_t day);
void setMonth(uint8_t month);
void setYear(uint8_t year);

void setHour(uint8_t hour);
void setMinute(uint8_t minute);
void setSeconds(uint8_t year);

void setUnixTime(Date* date);

#endif /* INC_DATE_TIME_H_ */
