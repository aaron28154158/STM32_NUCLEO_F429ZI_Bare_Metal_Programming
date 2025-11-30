#include "stm32f429xx_usart_driver.h"

/* ==================== 1. 初始化與重置 ==================== */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    	uint32_t tempreg = 0;

	/* ========== 1. 啟用 USART 時脈 ========== */
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/* ======================================== 設置 CR1 暫存器 ======================================== */
	/* ========== 2. 啟用 Tx/Rx 模式 (CR1 暫存器)========== */
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	/* ========== 3. 設置 Word Length (CR1 暫存器) ========== */
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	/* ========== 4. 設置 Parity Control Bit (CR1 暫存器) ========== */
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		/* ========== 4.1 啟用 Parity Control ========== */
		tempreg |= (1 << USART_CR1_PCE);

		/* ========== 4.2 啟用 Even Parity ========== */
		tempreg |= (USART_PARITY_EN_EVEN << USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		/* ========== 4.1 啟用 Parity Control ========== */
		tempreg |= (1 << USART_CR1_PCE);

		/* ========== 4.2 啟用 Odd Parity ========== */
		tempreg |= (USART_PARITY_EN_ODD << USART_CR1_PS);
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/* ======================================== 設置 CR2 暫存器 ======================================== */
	tempreg = 0;
	/* ========== 5. 設置 STOP bits (CR2 暫存器) ========== */
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/* ======================================== 設置 CR3 暫存器 ======================================== */
	tempreg = 0;
	/* ========== 6. 設置 CTS/RTS (CR3 暫存器) ========== */
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/* ======================================== 7. 設置 Baud Rate ======================================== */
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}


/* ==================== 2. 資料傳輸（阻塞模式）==================== */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	/* ========== 1. 傳送長度為 Len 大小的數據 ========== */
	for(uint32_t i = 0; i < Len; i++)
	{
		/* ========== 2. 等待 SR (狀態暫存器) 中的 TXE 標誌被設置 ========== */
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TXE_FLAG));

		/* ========== 3. 檢查 USART_WordLength 是 9BIT 還是 8BIT ========== */
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/* ========== 4. 9BIT 要放入 DR (數據暫存器), 所以將數據轉型為 2 byte ========== */
			uint16_t* pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			/* ========== 5. 檢查 USART_ParityControl 是否啟用 ========== */
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* ========== 5.1 沒有使用 parity, 所以 9BIT 都是數據, 需要 2 byte ========== */
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				/* ========== 5.2 有使用 parity, 所以 8BIT 是數據, 1BIT 是 parity, 只需要 1 byte ========== */
				pTxBuffer++;
			}
		}
		else
		{
			/* 4. 8BIT 放入 DR (數據暫存器) */
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	/* ========== 5. 等待 SR (狀態暫存器) 中的 TC (Transmission complete) 標誌被設置 ========== */
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TC_FLAG));
}


void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxRuffer, uint32_t Len);

/* ==================== 3. 資料傳輸（中斷模式）==================== */
void USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/* ==================== 4. 中斷處理 ==================== */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
        /* 使用巨集計算暫存器索引和位元位置 */
    uint8_t reg_index = NVIC_REG_INDEX(IRQNumber);  /* 0, 1, 或 2 */
    uint8_t bit_pos = NVIC_BIT_POS(IRQNumber);      /* 0-31 */

    if(EnorDi == ENABLE)
    {
        /* ==================== 啟用中斷 (ISER) ==================== */
        if(reg_index == 0)
        {
            *NVIC_ISER0 |= (1 << bit_pos);
        }
        else if(reg_index == 1)
        {
            *NVIC_ISER1 |= (1 << bit_pos);
        }
        else if(reg_index == 2)
        {
            *NVIC_ISER2 |= (1 << bit_pos);
        }
    }
    else
    {
        /* ==================== 禁用中斷 (ICER) ==================== */
        if(reg_index == 0)
        {
            *NVIC_ICER0 |= (1 << bit_pos);
        }
        else if(reg_index == 1)
        {
            *NVIC_ICER1 |= (1 << bit_pos);
        }
        else if(reg_index == 2)
        {
            *NVIC_ICER2 |= (1 << bit_pos);
        }
    }
}
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr_index = NVIC_IPR_INDEX(IRQNumber);
    uint8_t ipr_position = NVIC_IPR_SECTION(IRQNumber);
    uint8_t shift = (8 * ipr_position) + (8 - NO_IPR_BITS_IMPLEMENTED);

    *(NVIC_IPR_BASEADDR + ipr_index) |= (IRQPriority << shift);
}

void USART_IRQHandling(USART_Handle_t *pHandle);

/* ==================== 5. 輔助函式 ==================== */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}



uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
	if(pUSARTx->SR & StatusFlagName)
	{
		return SET;
	}
	return RESET;
}


void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
    pUSARTx->SR &= ~(1 << FlagName);
}


void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	/* ========== 1. 取得 APB 時脈 ========== */
	uint32_t PCLKx;
	uint32_t tempreg = 0;

	/* ========== 2. 根據 USART 外設判斷時脈源 ========== */
	if( (pUSARTx == USART1) || (pUSARTx == USART6) )
	{
		/* USART1, USART6 掛載於 APB2 */
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		/* USART2 ~ USART5 掛載於 APB1 */
		PCLKx = RCC_GetPCLK1Value();
	}

	/* ========== 3. 計算 USARTDIV (放大100倍以保留精度) ========== */
	uint32_t usartdiv_100;

	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		/* ========== 3.1 Oversampling by 8 ========== */
		/* 公式：USARTDIV = PCLKx / (8 * BaudRate)
		 * 放大100倍：usartdiv_100 = (100 * PCLKx) / (8 * BaudRate)
		 *                         = (25 * PCLKx) / (2 * BaudRate)
		 * 注意：必須先乘後除，避免整數除法造成精度損失 */
		usartdiv_100 = ((25 * PCLKx) / (2 * BaudRate));
	}
	else
	{
		/* ========== 3.2 Oversampling by 16 (預設) ========== */
		/* 公式：USARTDIV = PCLKx / (16 * BaudRate)
		 * 放大100倍：usartdiv_100 = (100 * PCLKx) / (16 * BaudRate)
		 *                         = (25 * PCLKx) / (4 * BaudRate) */
		usartdiv_100 = ((25 * PCLKx) / (4 * BaudRate));
	}

	/* ========== 4. 計算 Mantissa (整數部分) ========== */
	uint32_t M_part = usartdiv_100 / 100;
	tempreg |= (M_part << 4);  // BRR[15:4]

	/* ========== 5. 計算 Fraction (小數部分) ========== */
	/* 取得 100 倍的小數部分 */
	uint32_t F_part = usartdiv_100 - (M_part * 100);

	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		/* ========== 5.1 Oversampling by 8: Fraction 佔 3 bits ========== */
		/* 公式：DIV_Fraction[2:0] = ((F_part * 8) / 100)
		 * 加 50 做四捨五入，遮罩取低 3 bits */
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	}
	else
	{
		/* ========== 5.2 Oversampling by 16: Fraction 佔 4 bits ========== */
		/* 公式：DIV_Fraction[3:0] = ((F_part * 16) / 100)
		 * 加 50 做四捨五入，遮罩取低 4 bits */
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	tempreg |= F_part;  // BRR[3:0] for over16, BRR[2:0] for over8

	/* ========== 6. 寫入 BRR 暫存器 ========== */
	pUSARTx->BRR = tempreg;
}
