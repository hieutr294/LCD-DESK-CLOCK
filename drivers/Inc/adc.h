/*
 * adc.h
 *
 *  Created on: Mar 18, 2024
 *      Author: minhh
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define AWDCH_POS 0
#define EOCIE_POS 5
#define AWDIE_POS 6
#define JEOCIE_POS 7
#define SCAN_POS 8
#define AWDSGL_POS 9
#define JAUTO_POS 10
#define DISCEN_POS 11
#define JDISCEN_POS 12
#define DISCNUM_POS 13
#define DUALMOD_POS 16
#define JAWDEN_POS 22
#define AWDEN_POS 23

#define ADON_POS 0
#define CONT_POS 1
#define CAL_POS 2
#define RST_CAL_POS 3
#define DMA_POS 8
#define ALIGN_POS 11
#define JEXTSEL_POS 12
#define JEXTTRIG_POS 15
#define EXTSEL_POS 17
#define EXTTRIG_POS 20
#define JSWSTART_POS 21
#define SWSTART_POS 22
#define TSVREFE_POS 23
#define REGULAR_L_POS 20


#define INJECTED_CHANNEL 1
#define REGULAR_CHANNEL 2
#define SCAN_MODE 3
#define SINGLE_MODE 4

#define CYCLE_1 0
#define CYCLE_7 1
#define CYCLE_13 2
#define CYCLE_28 3
#define CYCLE_41 4
#define CYCLE_55 5
#define CYCLE_71 6
#define CYCLE_239 7

#define REGULAR_MODE 1
#define INJECTED_MODE 2

#define INJECTED_L_POS 20
typedef struct{
	uint8_t conversionMode; // SCAN MODE or SINGLE MODE
	uint8_t numberChannel;
	uint8_t sampleTime;
	uint8_t* channel;
	uint8_t channelGroup; // REGULAR MODE or INJECTED MODE
	uint8_t dmaEnable;
}ADC_Config_t;

typedef struct{
	ADC_RegDef_t* pADCX;
	ADC_Config_t pADCConfig;
}ADC_Handle_t;

void ADC_Init(ADC_Handle_t* pADCHandle);
void ADC_DeInit(ADC_Handle_t* pADCHandle);
void ADC_Start(ADC_Handle_t* pADCHandle);
void ADC_ClockControl(ADC_RegDef_t* pADCX, uint8_t condition);
uint16_t ADC_ReadData(ADC_Handle_t* pADCHandle);
#endif /* INC_ADC_H_ */
