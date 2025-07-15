/*
 * timer.h
 *
 *  Created on: Jul 7, 2025
 *      Author: minhh
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

typedef struct{

}Timer_Config;


typedef struct{
	Timer_Config config;
	TIMER_RegDef_t* TimerX;
}Timer_Handle;

void timerInit(Timer_Handle* timerHandle);


#endif /* INC_TIMER_H_ */
