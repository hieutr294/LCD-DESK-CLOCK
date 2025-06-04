#include "stm32f103xx.h"
#include <stdint.h>

#define BRR_FRACTION 0
#define BRR_MANTISSA 4

#define SR_PE 0
#define SR_FE 1
#define SR_NE 2
#define SR_ORE 3
#define SR_IDLE 4
#define SR_RXNE 5
#define SR_TC 6
#define SR_TXE 7
#define SR_LBD 8
#define SR_CTS 9

#define CR1_SBK 0
#define CR1_RWU 1
#define CR1_RE 2
#define CR1_TE 3
#define CR1_IDLEIE 4
#define CR1_RXNEIE 5
#define CR1_TCIE 6
#define CR1_TXEIE 7
#define CR1_PEIE 8
#define CR1_PS 9
#define CR1_PCE 10
#define CR1_WAKE 11
#define CR1_M 12
#define CR1_UE 13

#define CR2_ADD 0
#define CR2_LBDL 5
#define CR2_LBDIE 6
#define CR2_LBCL 8
#define CR2_CPHA 9
#define CR2_CPOL 10
#define CR2_CLKEN 11
#define CR2_STOP 12
#define CR2_LINEN 14

#define CR3_EIE 0
#define CR3_IREN 1
#define CR3_IRLP 2
#define CR3_HDSEL 3
#define CR3_NACK 4
#define CR3_SCEN 5
#define CR3_DMAR 6
#define CR3_DMAT 7
#define CR3_RTSE 8
#define CR3_CTSE 9
#define CR3_CTSIE 10

#define GTPR_PSC 0
#define GTPR_GT 8

/* USART_MODE */
#define USART_MODE_ONLY_RX 0
#define USART_MODE_ONLY_TX 1
#define USART_MODE_TXRX 2

/* USART_BAUD */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

typedef struct{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

typedef struct{
	USART_Config_t pConfig;
	USART_RegDef_t* pUSARTX;
}USART_Handle_t;

void __UART_GPIO_PIN();
void USART_ClockControl(USART_RegDef_t* pUSARTX);
void USART_Init(USART_Handle_t* pUSARTHandle);
void USART_SetBaud(USART_Handle_t* pUSARTHandle);
static uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTX, uint8_t FlagName);
static void USART_ClearFlag(USART_RegDef_t* pUSARTX, uint8_t FlagName);

void USART_SendData(USART_Handle_t* pUSARTHandle, uint16_t* pBuffer, uint32_t len);
void USART_ReciveData(USART_Handle_t* pUSARTHandle, uint8_t* pBuffer, uint8_t numberOfdata);

