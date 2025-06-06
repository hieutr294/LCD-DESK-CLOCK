/*
 * backup.h
 *
 *  Created on: Jun 6, 2025
 *      Author: minhh
 */

#include "stm32f103xx.h"

#ifndef INC_BACKUP_H_
#define INC_BACKUP_H_

void enableBackupReg(BKP_RegDef_t* bkp);

void bkpWrite16(BKP_RegDef_t* bkp, uint16_t data, uint8_t regNumber);
uint16_t bkpRead16(BKP_RegDef_t* bkp, uint8_t regNumber);

void bkpWrite32(BKP_RegDef_t* bkp, uint32_t data, uint8_t regNumber);
uint32_t bkpRead32(BKP_RegDef_t* bkp, uint8_t regNumber);

#endif /* INC_BACKUP_H_ */
