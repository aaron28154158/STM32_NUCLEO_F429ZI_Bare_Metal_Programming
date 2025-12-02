#include "stm32f429xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlagIT(I2C_Handle_t *pI2CHandle);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_MasterHandleRxNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTxEInterrupt(I2C_Handle_t *pI2CHandle);

/* ==================== 1. 初始化與重置 ==================== */
/*****************************************************************
 * @fn     I2C_Init
 * @brief  根據 pI2CHandle 中的設定來初始化指定的 I2C 周邊
 * @param  pI2CHandle: 指向 I2C handle 結構體的指標，包含配置參數
 * @return void
 *****************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	/* ==================== 1. 啟用 I2C 周邊時脈 ==================== */
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	/* ==================== 2. 配置 CR2 的 FREQ 欄位 ==================== */
	/* FREQ[5:0]: 周邊時脈頻率（單位：MHz）
	 * 計算時序參數，範圍：2 ~ 50 MHz
	 * 目前使用 HSI：APB1 = 16 MHz → FREQ = 16
	 */
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;  /* 單位: MHz */
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); /* 只使用低 6 bits */

	/* ==================== 3. 配置設備自身位址（從機模式使用）==================== */
	/* OAR1 暫存器格式（7-bit addressing mode）：
	 * Bit 14: 保留位（必須設為 1）
	 * Bit [7:1]: 7-bit 設備位址
	 * Bit 0: 不使用（R/W bit 在通訊時由硬體控制）
	 */
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);  /* 參考手冊：Bit 14 保留位必須由軟體設定為 1 */
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	/* ==================== 4. 配置 SCL 時脈頻率（CCR 暫存器）==================== */
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* ========== 標準模式（Standard Mode: <= 100 kHz）========== */
		/* CCR 計算公式：CCR = T_high / T_PCLK1 = (T_SCL / 2) / T_PCLK1
		 * 推導：CCR = f_PCLK1 / (2 * f_SCL)
		 * 範例：f_PCLK1 = 16 MHz, f_SCL = 100 kHz
		 *        CCR = 16,000,000 / (2 * 100,000) = 80
		 */
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0x0FFF);  /* CCR[11:0] */
	}
	else
	{
		/* ========== 快速模式（Fast Mode: <= 400 kHz）========== */
		tempreg |= (1 << 15);  /* F/S: 1 = Fast mode, 0 = Standard mode */
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);  /* DUTY bit */

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			/* ===== Duty cycle = 2 (T_low / T_high = 2:1) ===== */
			/* CCR 計算公式：CCR = f_PCLK1 / (3 * f_SCL)
			 * 推導：T_SCL = T_high + T_low = T_high + 2*T_high = 3*T_high
			 *      T_high = CCR * T_PCLK1 → CCR = f_PCLK1 / (3 * f_SCL)
			 */
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			/* ===== Duty cycle = 16/9 (T_low / T_high = 16:9) ===== */
			/* CCR 計算公式：CCR = f_PCLK1 / (25 * f_SCL)
			 * 推導：T_SCL = 9*T_high + 16*T_high = 25*T_high
			 *      T_high = CCR * T_PCLK1 → CCR = f_PCLK1 / (25 * f_SCL)
			 */
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0x0FFF);  /* CCR[11:0] */
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	/* ==================== 5. 配置 TRISE（SCL 上升時間）==================== */
	/* TRISE = (T_rise_max / T_PCLK1) + 1
	 * 標準模式：T_rise_max = 1000 ns
	 * 快速模式：T_rise_max = 300 ns
	 */
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* ===== 標準模式：T_rise_max = 1000 ns ===== */
		/* TRISE = (1000 ns / T_PCLK1) + 1
		 * 範例：f_PCLK1 = 16 MHz → T_PCLK1 = 62.5 ns
		 *        TRISE = (1000 / 62.5) + 1 = 17
		 */
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		/* ===== 快速模式：T_rise_max = 300 ns ===== */
		/* TRISE = (300 ns / T_PCLK1) + 1
		 * 範例：f_PCLK1 = 16 MHz → T_PCLK1 = 62.5 ns
		 *        TRISE = (300 / 62.5) + 1 = 5.8 ≈ 6
		 */
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);  /* TRISE[5:0] */
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/* ==================== 2. 時脈設定（啟用或禁用）==================== */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/* ==================== 3. 主機模式資料發送（阻塞式）==================== */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

    while(Len > 0)
    {
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    if(Sr == I2C_DISABLE_SR)
    {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
}


/* ==================== 4. 主機模式資料接收（阻塞式）==================== */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    if(Len == 1)
    {
        I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

        if(Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        *pRxBuffer = pI2CHandle->pI2Cx->DR;

        return;
    }
    else
    {
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        for(uint32_t i = Len; i > 0; i--)
        {
            while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

            if(i == 2)
            {
                I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

                if(Sr == I2C_DISABLE_SR)
                {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
            }

            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            pRxBuffer++;
        }
    }
    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
    }
}


/* ==================== 5. 主機模式資料發送（中斷式）==================== */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVFEN);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
    }

    return busystate;
}


/* ==================== 6. 主機模式資料接收（中斷式）==================== */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->RxSize = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVFEN);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
    }

    return busystate;
}


/* ==================== 7. 關閉中斷傳輸 ==================== */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
    pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t)pI2C->DR;
}


/* ==================== 8. 中斷設定與中斷處理 ==================== */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    /* 計算暫存器索引和位元位置 */
    uint8_t reg_index = NVIC_REG_INDEX(IRQNumber);  /* 0, 1, 或 2 */
    uint8_t bit_pos = NVIC_BIT_POS(IRQNumber);      /* 0-31 */

    if(EnorDi == ENABLE)
    {
        /* ==================== 啟用中斷 (ISER) ==================== */
        *(NVIC_ISER0 + reg_index) |= (1 << bit_pos);
    }
    else
    {
        /* ==================== 禁用中斷 (ICER) ==================== */
        *(NVIC_ICER0 + reg_index) |= (1 << bit_pos);
    }
}


void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr_index = NVIC_IPR_INDEX(IRQNumber);
    uint8_t ipr_position = NVIC_IPR_SECTION(IRQNumber);
    uint8_t shift = (8 * ipr_position) + (8 - NO_IPR_BITS_IMPLEMENTED);

    *(NVIC_IPR_BASEADDR + ipr_index) |= (IRQPriority << shift);
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t Event_IT = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVFEN);
    uint32_t Buffer_IT = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

    uint32_t SB_Event = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
    if(Event_IT && SB_Event)
    {
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }


    uint32_t ADDR_Event = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
    if(Event_IT && ADDR_Event)
    {
        I2C_ClearADDRFlagIT(pI2CHandle);
    }


    uint32_t BTF_Event = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    if(Event_IT && BTF_Event)
    {
        if(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE))
        {
            return;
        }

        if(pI2CHandle->TxRxState != I2C_BUSY_IN_TX)
        {
            return;
        }

        if(pI2CHandle->TxLen == 0)
        {
            if(pI2CHandle->Sr == I2C_DISABLE_SR)
            {
                I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }

            I2C_CloseSendData(pI2CHandle);

            I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
        }
    }


    uint32_t STOP_Event = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    if(Event_IT && STOP_Event)
    {
        /* 清除 STOPF : 讀 SR1, 寫 CR1 */
        pI2CHandle->pI2Cx->CR1 |= 0;

        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }


    uint32_t RxNE_Event = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
    if(Buffer_IT && Event_IT && RxNE_Event)
    {
        /* 1. 檢查是否為 Master 模式 */
        if((pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) )
        {
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_MasterHandleRxNEInterrupt(pI2CHandle);
            }
        }
        else
        {
            if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }


    uint32_t TxE_Event = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
    if(Buffer_IT && Event_IT && TxE_Event)
    {
        /* 1. 檢查是否為 Master 模式 */
        if((pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) )
        {
            /* 2. 檢查軟體狀態 */
            if( pI2CHandle->TxRxState == I2C_BUSY_IN_TX )
            {
                I2C_MasterHandleTxEInterrupt(pI2CHandle);
            }
        }
        else
        {
            if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }
}


static void I2C_MasterHandleRxNEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if(pI2CHandle->RxSize == 1)
    {
        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxSize > 1)
    {
        if(pI2CHandle->RxLen == 2)
        {
            I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);
        }

        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxLen == 0)
    {
        if(pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        I2C_CloseReceiveData(pI2CHandle);

        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}


static void I2C_MasterHandleTxEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if(pI2CHandle->TxLen > 0)
    {
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

        pI2CHandle->TxLen--;

        pI2CHandle->pTxBuffer++;
    }
}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    /* 1. 取得 CR2 中 ITERREN (Error Interrupt Enable) 的狀態 */
    /* 這是錯誤中斷的總開關 */
    uint32_t Error_IT = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    /*********************** Check for Bus error (BERR) ************************************/
    uint32_t BERR_Error = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);

    if(Error_IT && BERR_Error)
    {
        /* 清除 Bus error flag (寫 0 清除) */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        /* 通知 Application 層發生了 BERR 錯誤 */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /*********************** Check for Arbitration lost error (ARLO) ************************************/
    uint32_t ARLO_Error = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);

    if(Error_IT && ARLO_Error)
    {
        /* 清除 Arbitration lost error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        /* 通知 Application 層發生了 ARLO 錯誤 */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    /*********************** Check for ACK failure error (AF) ************************************/
    uint32_t AF_Error = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);

    if(Error_IT && AF_Error)
    {
        /* 清除 ACK failure error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        /* 通知 Application 層發生了 AF 錯誤 */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    /*********************** Check for Overrun/Underrun error (OVR) ************************************/
    uint32_t OVR_Error = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);

    if(Error_IT && OVR_Error)
    {
        /* 清除 Overrun/Underrun error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        /* 通知 Application 層發生了 OVR 錯誤 */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    /*********************** Check for Time out error (TIMEOUT) ************************************/
    uint32_t TIMEOUT_Error = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);

    if(Error_IT && TIMEOUT_Error)
    {
        /* 清除 Time out error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        /* 通知 Application 層發生了 TIMEOUT 錯誤 */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}



/* ==================== 9. 輔助函式 ==================== */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if(pI2Cx->SR1 & FlagName)
    {
        return FLAG_SET;
    }

    return FLAG_RESET;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    /* 1 ~ 8 bit = slave address */
    SlaveAddr = SlaveAddr << 1;

    /* Write bit = 0 */
    SlaveAddr &= ~(1);

    pI2Cx->DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    /* 1 ~ 8 bit = slave address */
    SlaveAddr = SlaveAddr << 1;

    /* Read bit = 0 */
    SlaveAddr |= 1;

    pI2Cx->DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
    uint32_t dummyReadSR1 = pI2Cx->SR1;
    uint32_t dummyReadSR2 = pI2Cx->SR2;
    (void)dummyReadSR1;
    (void)dummyReadSR2;
}

static void I2C_ClearADDRFlagIT(I2C_Handle_t *pI2CHandle)
{
    if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
    {
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if(pI2CHandle->RxSize == 1)
            {
                I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

                I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
            }
        }
        else
        {
        	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
        }
    }
    else
    {
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
    }
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == I2C_ACK_ENABLE)
    {
        /* Enable ACK */
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else
    {
        /* Disable ACK */
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
    }
    else
    {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    }
}


__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pSPIHandle, uint8_t AppEv)
{
    /* 此函數應該被應用層實作 */
}
