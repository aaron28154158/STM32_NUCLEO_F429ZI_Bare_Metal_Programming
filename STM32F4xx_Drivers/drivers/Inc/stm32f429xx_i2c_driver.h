#ifndef INC_STM32F429XX_I2C_DRIVER_H_
#define INC_STM32F429XX_I2C_DRIVER_H_

#include "stm32f429xx.h"

/* ==================== I2C 配置結構體 ==================== */
typedef struct
{
	uint32_t I2C_SCLSpeed;		/* 參考 @I2C_SCLSpeed */
	uint8_t	 I2C_DeviceAddress;	/* 設備自身位址（7-bit 或 10-bit 模式）*/
	uint8_t  I2C_ACKControl;	/* 參考 @I2C_ACKControl, 關於 I2C_SR1 的 ACK 欄位*/
	uint16_t I2C_FMDutyCycle;	/* 參考 @I2C_FMDutyCycle, 關於 I2C_CCR 的 DUTY 欄位 */
}I2C_Config_t;

/* ==================== I2C Handle(句柄) 結構體 ==================== */
typedef struct
{
	I2C_RegDef_t *pI2Cx;		/* 指向 I2Cx 的基底位址 (I2C1, I2C2, I2C3 等) */
	I2C_Config_t I2C_Config;	/* I2C 參數配置 */

	/* ========== 以下為中斷模式使用 ========== */
	uint8_t 	 *pTxBuffer;	/* 指向發送數據緩衝區的指標 */
	uint8_t		 *pRxBuffer;	/* 指向接收數據緩衝區的指標 */
	uint32_t	 TxLen;			/* 待發送的數據長度 */
	uint32_t 	 RxLen;			/* 待接收的數據長度 */
	uint8_t		 TxRxState;		/* I2C 通訊狀態：準備、發送忙碌、接收忙碌 */
	uint8_t		 DevAddr;		/* 當前事務的從機設備位址 */
	uint32_t	 RxSize;		/* 接收數據的總大小 */
	uint8_t		 Sr;			/* 重複起始條件：啟用或禁用 */
}I2C_Handle_t;

/* ==================== @I2C_SCLSpeed ==================== */
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM2k 200000
#define I2C_SCL_SPEED_FM4k 400000

/* ==================== @I2C_AckControl ==================== */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/* ==================== @I2C_FMDutyCycle ==================== */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/* ==================== I2C SR1 旗標 ==================== */
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)		/* Start Bit (Master mode) */
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)		/* Address sent (Master) / Address matched (Slave) */
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)		/* Byte Transfer Finished */
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)	/* 10-bit header sent (Master mode) */
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)	/* Stop detection (Slave mode) */
#define I2C_FLAG_RxNE		(1 << I2C_SR1_RxNE)		/* Data register not empty (Receivers) */
#define I2C_FLAG_TxE		(1 << I2C_SR1_TxE)		/* Data register empty (Transmitters) */
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)		/* Bus error */
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)		/* Arbitration lost (Master mode) */
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)		/* Acknowledge failure */
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)		/* Overrun/Underrun */
#define I2C_FLAG_PECERR		(1 << I2C_SR1_PECERR)	/* PEC Error in reception */
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)	/* Timeout or Tlow error */
#define I2C_FLAG_SMBALERT	(1 << I2C_SR1_SMBALERT)	/* SMBus alert */

/* ==================== I2C SR2 旗標 ==================== */
#define I2C_FLAG_MSL		(1 << I2C_SR2_MSL)		/* Master/Slave mode */
#define I2C_FLAG_BUSY		(1 << I2C_SR2_BUSY)		/* Bus busy */
#define I2C_FLAG_TRA		(1 << I2C_SR2_TRA)		/* Transmitter/Receiver */
#define I2C_FLAG_GENCALL	(1 << I2C_SR2_GENCALL)	/* General call address (Slave mode) */
#define I2C_FLAG_SMBDEFAULT	(1 << I2C_SR2_SMBDFFAULT) /* SMBus device default address (Slave mode) */
#define I2C_FLAG_SMBHOST	(1 << I2C_SR2_SMBHOST)	/* SMBus host header (Slave mode) */
#define I2C_FLAG_DUALF		(1 << I2C_SR2_DUALF)	/* Dual flag (Slave mode) */

/* ==================== Repeated START ==================== */
#define I2C_DISABLE_SR  0  /* 發送 STOP，釋放總線 */
#define I2C_ENABLE_SR   1  /* 不發送 STOP，保持總線佔用 */

/* ==================== 中斷狀態旗標 ==================== */
#define I2C_READY       0
#define I2C_BUSY_IN_TX  1
#define I2C_BUSY_IN_RX  2

/* ==================== 應用程式事件 ==================== */
#define I2C_EV_TX_CMPLT     0
#define I2C_EV_RX_CMPLT     1
#define I2C_EV_STOP         2

/* ==================== 錯誤事件 ==================== */
#define I2C_ERROR_BERR      3
#define I2C_ERROR_ARLO      4
#define I2C_ERROR_AF        5
#define I2C_ERROR_OVR       6
#define I2C_ERROR_TIMEOUT   7
#define I2C_EV_DATA_REQ     8
#define I2C_EV_DATA_RCV     9


/* ================================================================================
*   I2C Driver APIs
*   1. I2C initialization：初始化 I2C
*   2. Enable / Disable I2C clock：啟用或禁用 I2C 時脈
*   3. I2C Master data transmission (Blocking mode)：主機模式數據發送（阻塞式）
*   4. I2C Master data reception (Blocking mode)：主機模式數據接收（阻塞式）
*   5. I2C Master data transmission (Interrupt mode)：主機模式數據發送（中斷式）
*   6. I2C Master data reception (Interrupt mode)：主機模式數據接收（中斷式）
*   7. I2C interrupt config & handling：中斷配置與處理
*   8. Other I2C management APIs：其他 I2C 管理函數
*  ================================================================================
*/

/* ==================== 1. 初始化與重置 ==================== */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/* ==================== 2. 時脈設定（啟用或禁用）==================== */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/* ==================== 3. 主機模式數據發送（阻塞式）==================== */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/* ==================== 4. 主機模式數據接收（阻塞式）==================== */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/* ==================== 5. 主機模式數據發送（中斷式）==================== */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/* ==================== 6. 主機模式數據接收（中斷式）==================== */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/* ==================== 7. 關閉中斷傳輸 ==================== */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/* ==================== 從機模式 ==================== */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/* ==================== 8. 中斷設定與中斷處理 ==================== */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);	/* Event interrupt handler */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);	/* Error interrupt handler */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

/* ==================== 9. 輔助函式 ==================== */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

#endif
