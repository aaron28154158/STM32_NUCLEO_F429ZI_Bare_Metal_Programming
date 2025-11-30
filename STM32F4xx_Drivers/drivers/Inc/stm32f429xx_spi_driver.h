#ifndef INC_STM32F429XX_SPI_DRIVER_H_
#define INC_STM32F429XX_SPI_DRIVER_H_

#include "stm32f429xx.h"

/* ==================== SPI 引腳配置結構體 ==================== */
typedef struct
{
	uint8_t SPI_DeviceMode;		/* 參考 ＠SPI_DeviceMode, 關於 SPI_CR1 的 MSTR 欄位 */
	uint8_t SPI_BusConfig;		/* 參考 ＠SPI_BusConfig, 關於 SPI_CR1 的 BIDIMODE 欄位 */
	uint8_t SPI_SclkSpeed;		/* 參考 ＠SPI_SclkSpeed, 關於 SPI_CR1 的 BR 欄位 */
	uint8_t SPI_DFF;			/* 參考 ＠SPI_DFF, 關於 SPI_CR1 的 DFF 欄位 */
	uint8_t SPI_CPOL;			/* 參考 ＠SPI_CPOL, 關於 SPI_CR1 的 CPOL 欄位 */
	uint8_t SPI_CPHA;			/* 參考 ＠SPI_CPHA, 關於 SPI_CR1 的 CPHA 欄位 */
	uint8_t SPI_SSM;			/* 參考 ＠SPI_DFF, 關於 SPI_CR1 的 SSM 欄位 */
}SPI_Config_t;

/* ==================== SPI Handle(句柄) 結構體 ==================== */
/* SPI_Handle 結構體*/
typedef struct
{
	SPI_RegDef_t *pSPIx;        /* 指向 SPIx 的基底位址 (SPI1, SPI2, SPI3 等) */
	SPI_Config_t SPIConfig;     /* 保存 SPI 參數配置 */

	/* ========== 以下為了中斷使用 ========== */
	uint8_t		 *pTxBuffer;    /* 指向發送數據緩衝區的指標 */
	uint8_t 	 *pRxBuffer;    /* 指向接收數據緩衝區的指標 */
	uint32_t	 TxLen;         /* 待發送的數據長度 */
	uint32_t	 RxLen;         /* 待接收的數據長度 */
	uint8_t 	 TxState;       /* 發送狀態：準備 或 忙碌*/
	uint8_t		 RxState;       /* 發送狀態：準備 或 忙碌*/
}SPI_Handle_t;

/* ================================================================================
*   SPI 參數設置
*  ================================================================================
*/

/* ==================== ＠SPI_DeviceMode ==================== */
#define SPI_DEVICE_MODE_MASTER		1       /* 主機模式 */
#define SPI_DEVICE_MODE_SLAVE		0       /* 從機模式 */

/* ==================== ＠SPI_BusConfig ==================== */
#define SPI_BUS_CONFIG_FD				1   /* 全雙工：同時發送和接收*/
#define SPI_BUS_CONFIG_HD				2   /* 半雙工：單線雙向 */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3   /* 單工：只接收 */

/* ==================== ＠SPI_SclkSpeed ==================== */
/* 波特率(Baud Rate) 控制：APB 時鐘分頻比 */
#define SPI_SCLK_SPEED_DIV2				0   /* f_PCLK / 2 */
#define SPI_SCLK_SPEED_DIV4				1   /* f_PCLK / 4 */
#define SPI_SCLK_SPEED_DIV8				2   /* f_PCLK / 8 */
#define SPI_SCLK_SPEED_DIV16			3   /* f_PCLK / 16 */
#define SPI_SCLK_SPEED_DIV32			4   /* f_PCLK / 32 */
#define SPI_SCLK_SPEED_DIV64			5   /* f_PCLK / 64 */
#define SPI_SCLK_SPEED_DIV128			6   /* f_PCLK / 128 */
#define SPI_SCLK_SPEED_DIV256			7   /* f_PCLK / 256 */

/* ==================== @SPI_DFF ==================== */
#define SPI_DFF_8BITS	0       /* 數據格式：1 byte */
#define SPI_DFF_16BITS	1       /* 數據格式：2 byte */

/* ==================== @SPI_CPOL ==================== */
#define SPI_CPOL_LOW	0       /* 空閒時，時脈為低電位 */
#define SPI_CPOL_HIGH	1       /* 空閒時，時脈為高電位 */

/* ==================== @SPI_CPHA ==================== */
#define SPI_CPHA_LOW	0       /* 第一個時脈邊沿採樣數據 */
#define SPI_CPHA_HIGH	1       /* 第二個時脈邊沿採樣數據 */


/* ==================== @SPI_SSM ==================== */
#define SPI_SSM_EN		1       /* 軟體 NSS 管理（啟用）*/
#define SPI_SSM_DI		0       /* 軟體 NSS 管理（禁用）*/



/* ==================== SPI_SR 旗標 ==================== */
#define SPI_FLAG_TXE    (1 << SPI_SR_TXE)
#define SPI_FLAG_BSY	(1 << SPI_SR_BSY)
#define SPI_FLAG_RXNE	(1 << SPI_SR_RXNE)


/* ==================== SPI Tx / Rx 狀態 ==================== */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	1


/* ==================== SPI 事件 ==================== */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

/* ================================================================================
*   SPI Driver APIs
*   1. SPI initialization：初始化 SPI
*   2. Enable / Disable SPI clock：啟用或禁用 SPI 時脈
*   3. SPI Tx
*   4. SPI Rx
*   5. SPI interrupt config & handling
*   6. Other SPI management APIs
*  ================================================================================
*/

/* ==================== 1. 初始化與重置 ==================== */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* ==================== 2. 時脈設定（啟用或禁用）==================== */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* ==================== 3. 數據讀取 ==================== */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* ==================== 4. 數據寫入 ==================== */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/* ==================== 5. 中斷設定與中斷處理 ==================== */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/* ==================== 其他輔助函式 ==================== */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif
