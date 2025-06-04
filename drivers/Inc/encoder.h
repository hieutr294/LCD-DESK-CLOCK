/*
 * encoder.h
 *
 *  Created on: Jun 3, 2025
 *      Author: minhh
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#define COUNT_ON_C1 1
#define COUNT_ON_C2 2
#define COUNT_ON_BOTH 3

#define INVERTED 1
#define NON_INVERTED 0

typedef struct{
	TIMER_RegDef_t* timerX;
	uint8_t channelToCount;
	uint8_t inputPolarity;
	uint16_t maxValue;
}Encoder_Config_t;

void __gpioConfig(uint8_t pinNumber, GPIO_RegDef_t* gpioX);
void __timerConfig(TIMER_RegDef_t* timerX, uint8_t smsValue, uint8_t polarity, uint8_t channelToCount, uint16_t maxValue);
void EncoderInit(Encoder_Config_t* config);
#endif /* INC_ENCODER_H_ */
