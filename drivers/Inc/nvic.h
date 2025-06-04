/*
 * nvic.h
 *
 *  Created on: Apr 8, 2024
 *      Author: minhh
 */

#ifndef INC_NVIC_H_
#define INC_NVIC_H_
#define I2C1_EV_IRQ_NUMBER	31
#define DMA1_CHANNEL5	15
void nvicEnable(int IRQNumber);
void nvicDisable(uint8_t IRQNumber);
void nvicSetPriority(uint8_t IRQNumber, uint8_t priority);

#endif /* INC_NVIC_H_ */
