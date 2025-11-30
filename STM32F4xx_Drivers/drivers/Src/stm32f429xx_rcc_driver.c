#include "stm32f429xx_rcc_driver.h"

/* ==================== 時脈分頻係數查詢表 ==================== */
uint16_t AHB_PreScaler[8]  = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB_PreScaler[4]  = {2, 4, 8, 16};

/*****************************************************************
 * @fn     RCC_GetPCLK1Value
 * @brief  取得 APB1 周邊時脈頻率 (PCLK1)
 * @param  None
 * @return uint32_t: PCLK1 頻率（單位：Hz）
 * @note   時脈路徑：System Clock → AHB Prescaler → APB1 Prescaler → PCLK1
 *         - System Clock 來源：HSI (16MHz) / HSE (8MHz) / PLL
 *         - AHB  分頻：1, 2, 4, 8, 16, 64, 128, 256, 512
 *         - APB1 分頻：1, 2, 4, 8, 16
 *****************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, systemClock;
	uint8_t temp, ahbPrescaler, apb1Prescaler;

	/* ==================== 1. 讀取系統時脈源 ==================== */
	/* RCC_CFGR[3:2] SWS: System clock switch status
	 * 00: HSI oscillator (16 MHz)
	 * 01: HSE oscillator (8 MHz)
	 * 10: PLL
	 * 11: Not applicable
	 */
	uint32_t clockSource = (RCC->CFGR >> 2) & 0x03;

	if(clockSource == 0)
	{
		systemClock = 16000000;  /* HSI = 16 MHz */
	}
	else if(clockSource == 1)
	{
		systemClock = 8000000;   /* HSE = 8 MHz */
	}
	else if(clockSource == 2)
	{
		systemClock = RCC_GetPLLOutputClock();  /* PLL output */
	}

	/* ==================== 2. 計算 AHB 分頻係數 ==================== */
	/* RCC_CFGR[7:4] HPRE: AHB prescaler
	 * 0xxx: system clock not divided  (AHB = SYSCLK)
	 * 1000: system clock divided by 2
	 * 1001: system clock divided by 4
	 * 1010: system clock divided by 8
	 * ...
	 * 1111: system clock divided by 512
	 */
	temp = (RCC->CFGR >> 4) & 0x0F;

	if(temp < 8)
	{
		ahbPrescaler = 1;  /* 不分頻 */
	}
	else
	{
		/* temp = 8~15 對應到分頻係數 2, 4, 8, ..., 512 */
		ahbPrescaler = AHB_PreScaler[temp - 8];
	}

	/* ==================== 3. 計算 APB1 分頻係數 ==================== */
	/* RCC_CFGR[12:10] PPRE1: APB Low-speed prescaler (APB1)
	 * 0xx: AHB clock not divided  (APB1 = HCLK)
	 * 100: AHB clock divided by 2
	 * 101: AHB clock divided by 4
	 * 110: AHB clock divided by 8
	 * 111: AHB clock divided by 16
	 */
	temp = (RCC->CFGR >> 10) & 0x07;

	if(temp < 4)
	{
		apb1Prescaler = 1;  /* 不分頻 */
	}
	else
	{
		/* temp = 4~7 對應到分頻係數 2, 4, 8, 16 */
		apb1Prescaler = APB_PreScaler[temp - 4];
	}

	/* ==================== 4. 計算最終 PCLK1 頻率 ==================== */
	/* PCLK1 = SYSCLK / AHB_Prescaler / APB1_Prescaler
	 * 範例：SYSCLK = 16MHz, AHB = /1, APB1 = /1
	 *        PCLK1 = 16MHz / 1 / 1 = 16MHz
	 */
	pclk1 = (systemClock / ahbPrescaler) / apb1Prescaler;

	return pclk1;
}

/*****************************************************************
 * @fn     RCC_GetPCLK2Value
 * @brief  取得 APB2 周邊時脈頻率 (PCLK2)
 * @param  None
 * @return uint32_t: PCLK2 頻率（單位：Hz）
 * @note   時脈路徑：System Clock → AHB Prescaler → APB2 Prescaler → PCLK2
 *         - System Clock 來源：HSI (16MHz) / HSE (8MHz) / PLL
 *         - AHB  分頻：1, 2, 4, 8, 16, 64, 128, 256, 512
 *         - APB2 分頻：1, 2, 4, 8, 16
 *****************************************************************/
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, systemClock;
	uint8_t temp, ahbPrescaler, apb2Prescaler;

	/* ==================== 1. 讀取系統時脈源 ==================== */
	/* RCC_CFGR[3:2] SWS: System clock switch status
	 * 00: HSI oscillator (16 MHz)
	 * 01: HSE oscillator (8 MHz)
	 * 10: PLL
	 * 11: Not applicable
	 */
	uint32_t clockSource = (RCC->CFGR >> 2) & 0x03;

	if(clockSource == 0)
	{
		systemClock = 16000000;  /* HSI = 16 MHz */
	}
	else if(clockSource == 1)
	{
		systemClock = 8000000;   /* HSE = 8 MHz */
	}
	else if(clockSource == 2)
	{
		systemClock = RCC_GetPLLOutputClock();  /* PLL output */
	}

	/* ==================== 2. 計算 AHB 分頻係數 ==================== */
	/* RCC_CFGR[7:4] HPRE: AHB prescaler
	 * 0xxx: system clock not divided  (AHB = SYSCLK)
	 * 1000: system clock divided by 2
	 * 1001: system clock divided by 4
	 * 1010: system clock divided by 8
	 * ...
	 * 1111: system clock divided by 512
	 */
	temp = (RCC->CFGR >> 4) & 0x0F;

	if(temp < 8)
	{
		ahbPrescaler = 1;  /* 不分頻 */
	}
	else
	{
		/* temp = 8~15 對應到分頻係數 2, 4, 8, ..., 512 */
		ahbPrescaler = AHB_PreScaler[temp - 8];
	}

	/* ==================== 3. 計算 APB2 分頻係數 ==================== */
	/* RCC_CFGR[15:13] PPRE2: APB high-speed prescaler (APB2)
	 * 0xx: AHB clock not divided  (APB2 = HCLK)
	 * 100: AHB clock divided by 2
	 * 101: AHB clock divided by 4
	 * 110: AHB clock divided by 8
	 * 111: AHB clock divided by 16
	 */
	temp = (RCC->CFGR >> 13) & 0x07;

	if(temp < 4)
	{
		apb2Prescaler = 1;  /* 不分頻 */
	}
	else
	{
		/* temp = 4~7 對應到分頻係數 2, 4, 8, 16 */
		apb2Prescaler = APB_PreScaler[temp - 4];
	}

	/* ==================== 4. 計算最終 PCLK2 頻率 ==================== */
	/* PCLK2 = SYSCLK / AHB_Prescaler / APB2_Prescaler
	 * 範例：SYSCLK = 16MHz, AHB = /1, APB2 = /1
	 *        PCLK2 = 16MHz / 1 / 1 = 16MHz
	 */
	pclk2 = (systemClock / ahbPrescaler) / apb2Prescaler;

	return pclk2;
}

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}
