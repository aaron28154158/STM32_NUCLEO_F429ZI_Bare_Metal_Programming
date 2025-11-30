#ifndef INC_STM32F429XX_USART_DRIVER_H_
#define INC_STM32F429XX_USART_DRIVER_H_

#include "stm32f429xx.h"

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	/* ========== 以下為中斷使用 ========== */
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;


/* ===================== @USART_Mode (CR1 Register) ===================== */
#define USART_MODE_ONLY_TX		0
#define USART_MODE_ONLY_RX		1
#define USART_MODE_TXRX			2

/* ===================== @USART_Baud (BRR Register) ===================== */
#define USART_STD_BAUD_1200			1200
#define USART_STD_BAUD_2400			1200
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200		19200
#define USART_STD_BAUD_38400		38400
#define USART_STD_BAUD_57600		57600
#define USART_STD_BAUD_115200		115200
#define USART_STD_BAUD_230400		230400
#define USART_STD_BAUD_460800		460800
#define USART_STD_BAUD_921600		921600
#define USART_STD_BAUD_2M			2000000
#define USART_STD_BAUD_3M			3000000

/* ===================== @USART_ParityControl (CR1 Register) ===================== */
#define USART_PARITY_DISABLE		0
#define USART_PARITY_EN_EVEN		1
#define USART_PARITY_EN_ODD			2

/* ===================== @USART_WordLength (CR1 Register) ===================== */
#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1

/* ===================== @USART_HWFlowControl (CR3 Register) ===================== */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/* ===================== @USART_NoOfStopBits (CR2 Register) ===================== */
#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USRAT_STOPBITS_2			2
#define USRAT_STOPBITS_1_5			3

/* ===================== USART SR (狀態暫存器) 標誌 =====================  */
#define USART_PE_FLAG		        (1 << USART_SR_PE)
#define USART_FE_FLAG		        (1 << USART_SR_FE)
#define USART_NF_FLAG		        (1 << USART_SR_NF)
#define USART_ORE_FLAG		        (1 << USART_SR_ORE)
#define USART_IDLE_FLAG		        (1 << USART_SR_IDLE)
#define USART_RXNE_FLAG		        (1 << USART_SR_RXNE)
#define USART_TC_FLAG		        (1 << USART_SR_TC)
#define USART_TXE_FLAG		        (1 << USART_SR_TXE)
#define USART_LBD_FLAG		        (1 << USART_SR_LBD)
#define USART_CTS_FLAG		        (1 << USART_SR_CTS)


/* ===================== 應用程式事件狀態 =====================  */
#define USART_READY 			0
#define USART_BUSY_IN_RX 		1
#define USART_BUSY_IN_TX 		2
#define USART_EVENT_TX_CMPLT	0
#define	USART_EVENT_RX_CMPLT   	1
#define	USART_EVENT_IDLE      	2
#define	USART_EVENT_CTS       	3
#define	USART_EVENT_PE        	4
#define	USART_ERR_FE     		5
#define	USART_ERR_NE    	 	6
#define	USART_ERR_ORE    		7


/* ==================== 1. 初始化與重置 ==================== */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/* ==================== 2. 資料傳輸（阻塞模式）==================== */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxRuffer, uint32_t Len);

/* ==================== 3. 資料傳輸（中斷模式）==================== */
void USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/* ==================== 4. 中斷處理 ==================== */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/* ==================== 5. 輔助函式 ==================== */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

void USART_ApplicationCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);

#endif
