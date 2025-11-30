#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include "stm32f429xx.h"

/* ==================== GPIO 引腳配置結構體 ==================== */
typedef struct
{
    uint8_t GPIO_PinNumber;			/* 參考 @GPIO 引腳編號 */
	uint8_t GPIO_PinMode;			/* 參考 @GPIO_引腳模式 */
	uint8_t GPIO_PinSpeed;			/* 參考 @GPIO_引腳輸出速度 */
	uint8_t GPIO_PinPuPdControl;	/* 參考 @GPIO_引腳上拉/下拉控制 */
	uint8_t GPIO_PinOPType;			/* 參考 @GPIO_引腳輸出類型 */
	uint8_t GPIO_PinAltFunMode;		/* 參考 @GPIO_引腳複用功能 */
}GPIO_PinConfig_t;

/* ==================== GPIO Handle(句柄) 結構體 ==================== */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;              /* 指向欲操作的 GPIOx 的基底位址 (GPIOA, GPIOB 等等)*/
    GPIO_PinConfig_t GPIO_PinConfig;    /* 保存對應的 GPIO 引腳配置 */
}GPIO_Handle_t;

/* ================================================================================
*   GPIO 參數設置
*  ================================================================================
*/

/* ==================== @GPIO 引腳編號 ==================== */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/* ==================== @GPIO_引腳模式 ==================== */
#define GPIO_MODE_IN		    0	/* Input */
#define GPIO_MODE_OUT 		    1	/* General purpose output mode */
#define GPIO_MODE_ALTFN		    2	/* Alternate function mode */
#define GPIO_MODE_ANALOG	    3	/* Analog mode */
#define GPIO_MODE_IT_FT		    4	/* Interrupt Falling Trigger */
#define GPIO_MODE_IT_RT		    5	/* Interrupt Rising Trigger */
#define GPIO_MODE_IT_RFT	    6	/* Interrupt Falling/Rising Trigger */

/* ==================== @GPIO_引腳上拉/下拉控制 ==================== */
#define GPIO_NO_PUPD		    0	/* No Pull-Up, Pull-Down */
#define GPIO_PIN_PU				1	/* Pull-Up */
#define GPIO_PIN_PD				2	/* Pull-Down */

/* ==================== @GPIO_引腳輸出類型 ==================== */
#define GPIO_OP_TYPE_PP		    0	/* Output Push-Pull */
#define GPIO_OP_TYPE_OD		    1   /* Output Open-Drain */

/* ==================== @GPIO_引腳輸出速度 ==================== */
#define GPIO_SPEED_LOW		    0	/* Low speed */
#define GPIO_SPEED_MEDIUM	    1	/* Medium speed */
#define GPIO_SPEED_HIGH		    2	/* High speed */
#define GPIO_SPEED_VERYHIGH	    3	/* Very High speed */

/* ==================== @GPIO_引腳複用功能==================== */
#define GPIO_ALTFN_AF00		0
#define GPIO_ALTFN_AF01		1
#define GPIO_ALTFN_AF02		2
#define GPIO_ALTFN_AF03		3
#define GPIO_ALTFN_AF04		4
#define GPIO_ALTFN_AF05		5
#define GPIO_ALTFN_AF06		6
#define GPIO_ALTFN_AF07		7
#define GPIO_ALTFN_AF08		8
#define GPIO_ALTFN_AF09		9
#define GPIO_ALTFN_AF10		10
#define GPIO_ALTFN_AF11		11
#define GPIO_ALTFN_AF12		12
#define GPIO_ALTFN_AF13		13
#define GPIO_ALTFN_AF14		14
#define GPIO_ALTFN_AF15		15



/* ================================================================================
*   GPIO Driver APIs
*   1. GPIO initialization：初始化 GPIO
*   2. Enable / Disable GPIO port clock：啟用或禁用GPIO時脈
*   3. Read from a GPIO pin：從數據暫存器讀取數據
*   4. Write to GPIO pin：寫入數據到數據暫存器
*   5. Configure alternate functionality：設置複用功能
*   6. Interrupt Handling：中斷處理
*  ================================================================================
*/

/* ==================== 1. 初始化與重置 ==================== */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* ==================== 2. 時脈設定（啟用或禁用）==================== */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* ==================== 3. 數據讀取 ==================== */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/* ==================== 4. 數據寫入 ==================== */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* ==================== 5. 中斷設定與中斷處理 ==================== */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif
