/*
 * date.c
 *
 *  Created on: Jul 6, 2025
 *      Author: minhh
 */


#include <date_time.h>
#include "stdint.h"

Date getDate(Date* date){
	uint32_t days = date->unixTime / 86400;

	// Công thức tính năm:
	uint32_t z = days + 719468;  // 719468 là offset từ 01/01/0000 đến 01/01/1970
	uint32_t era = z / 146097;
	uint32_t doe = z - era * 146097;
	uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	uint32_t y = yoe + era * 400;
	uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	uint32_t mp = (5 * doy + 2) / 153;

	date->day = doy - (153 * mp + 2) / 5 + 1;
	date->month = mp < 10 ? mp + 3 : mp - 9;
	date->year = y + (date->month <= 2);
}

Date getTime(Date* date){
	date->seconds = date->unixTime%60;
	date->minute = ((date->unixTime/60)%60);
	date->hour = ((date->unixTime/3600)%25)+7;
}
