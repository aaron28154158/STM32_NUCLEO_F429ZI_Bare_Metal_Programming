#include "stm32f429xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pHandle);


/* ==================== 1. 初始化與重置 ==================== */

/*****************************************************************
 * @fn     SPI_Init
 * @brief  根據 pSPIHandle 中的設定來初始化指定的 SPI 外設
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標，包含 SPI 外設和配置參數
 * @return void
 * @note   使用前必須先呼叫 SPI_PeriClockControl() 啟用時脈
 *****************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    /* 啟用 SPI 周邊時脈 */
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    uint32_t tempreg = 0;

    /* ==================== 設置 SPI_CR1 的參數 ==================== */

    /* ========== 1. 設置主機/從機模式 (MSTR) ========== */
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    /* ========== 2. 設置匯流排配置模式 (BIDIMODE & RXONLY) ========== */
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        /* 全雙工模式：BIDIMODE = 0 */
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        /* 半雙工模式：BIDIMODE = 1 */
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        /* 單工只接收模式：BIDIMODE = 0, RXONLY = 1 */
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    /* ========== 3. 設置時脈波特率 (BR) ========== */
    tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    /* ========== 4. 設置數據格式 (DFF) ========== */
    tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

    /* ========== 5. 設置時脈極性 (CPOL) ========== */
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    /* ========== 6. 設置時脈相位 (CPHA) ========== */
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    /* ========== 7. 設置軟體從機管理 (SSM) ========== */
    tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

    /* 將配置寫入 SPI_CR1 寄存器 */
    pSPIHandle->pSPIx->CR1 = tempreg;
}


/*****************************************************************
 * @fn     SPI_DeInit
 * @brief  將指定的 SPI 外設重置為預設值
 * @param  pSPIx: SPIx 的基底位址 (例如 SPI1, SPI2, SPI3)
 * @return void
 * @note   透過 RCC 的 APB reset 寄存器來重置 SPI
 *****************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if(pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if(pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
}


/* ==================== 2. 時脈設定（啟用或禁用）==================== */

/*****************************************************************
 * @fn     SPI_PeriClockControl
 * @brief  啟用或禁用指定 SPI 的周邊時脈
 * @param  pSPIx: SPIx 的基底位址 (例如 SPI1, SPI2, SPI3)
 * @param  EnorDi: ENABLE 或 DISABLE
 * @return void
 * @note   配置 SPI 之前必須先啟用時脈
 *****************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
    }
}


/* ==================== 3. 數據接收（阻塞模式）==================== */

/*****************************************************************
 * @fn     SPI_ReceiveData
 * @brief  通過 SPI 接收數據（阻塞模式）
 * @param  pSPIx: SPIx 的基底位址
 * @param  pRxBuffer: 指向接收數據緩衝區的指標
 * @param  Len: 要接收的數據長度（字節數）
 * @return void
 * @note   - 此函數為阻塞模式，會等待接收完成
 *         - 16 位模式下，Len 應為偶數
 *****************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        /* ========== 1. 等待 RXNE 標誌位（接收緩衝區非空）========== */
        while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);

        /* ========== 2. 根據數據幀格式接收數據 ========== */
        if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            /* 16 位模式：接收 2 個字節 */
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;
        }
        else
        {
            /* 8 位模式：接收 1 個字節 */
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}


/* ==================== 4. 數據發送（阻塞模式）==================== */

/*****************************************************************
 * @fn     SPI_SendData
 * @brief  通過 SPI 發送數據（阻塞模式）
 * @param  pSPIx: SPIx 的基底位址
 * @param  pTxBuffer: 指向發送數據緩衝區的指標
 * @param  Len: 要發送的數據長度（字節數）
 * @return void
 * @note   - 此函數為阻塞模式，會等待發送完成
 *         - 只能透過 DR 寄存器操控 Tx Buffer 和 Rx Buffer
 *         - 16 位模式下，Len 應為偶數
 *****************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        /* ========== 1. 等待 TXE 標誌位（發送緩衝區為空）========== */
        while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

        /* ========== 2. 根據數據幀格式發送數據 ========== */
        if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            /* 16 位模式：發送 2 個字節 */
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            pTxBuffer += 2;
            Len -= 2;
        }
        else
        {
            /* 8 位模式：發送 1 個字節 */
            pSPIx->DR = *pTxBuffer;
            pTxBuffer++;
            Len--;
        }
    }
}


/* ==================== 5. 中斷模式數據傳輸 ==================== */

/*****************************************************************
 * @fn     SPI_SendDataIT
 * @brief  通過 SPI 發送數據（中斷模式）
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @param  pTxBuffer: 指向發送數據緩衝區的指標
 * @param  Len: 要發送的數據長度（字節數）
 * @return uint8_t: 當前 SPI 傳輸狀態
 * @note   - 此函數為非阻塞模式，透過中斷處理數據發送
 *         - 實際發送在 ISR 中完成
 *****************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if(state != SPI_BUSY_IN_TX)
    {
        /* ==================== 1. 儲存 Tx buffer 位址和長度 ==================== */
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        /* ==================== 2. 標記 SPI 為忙碌狀態 ==================== */
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        /* ==================== 3. 啟用 TXEIE 中斷 ==================== */
        /* 當 TXE 標誌被設置時會觸發中斷 */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }

    return state;
}


/*****************************************************************
 * @fn     SPI_ReceiveDataIT
 * @brief  通過 SPI 接收數據（中斷模式）
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @param  pRxBuffer: 指向接收數據緩衝區的指標
 * @param  Len: 要接收的數據長度（字節數）
 * @return uint8_t: 當前 SPI 接收狀態
 * @note   - 此函數為非阻塞模式，透過中斷處理數據接收
 *         - 實際接收在 ISR 中完成
 *****************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if(state != SPI_BUSY_IN_RX)
    {
        /* ==================== 1. 儲存 Rx buffer 位址和長度 ==================== */
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        /* ==================== 2. 標記 SPI 為忙碌狀態 ==================== */
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        /* ==================== 3. 啟用 RXNEIE 中斷 ==================== */
        /* 當 RXNE 標誌被設置時會觸發中斷 */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return state;
}


/* ==================== 6. NVIC 中斷配置 ==================== */

/*****************************************************************
 * @fn     SPI_IRQInterruptConfig
 * @brief  配置 NVIC 中斷（啟用或禁用）
 * @param  IRQNumber: 中斷號碼
 * @param  EnorDi: ENABLE 或 DISABLE
 * @return void
 * @note   SPI 直接連接到 NVIC，不像 GPIO 需要透過 EXTI
 *****************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


/*****************************************************************
 * @fn     SPI_IRQPriorityConfig
 * @brief  配置 SPI 中斷優先級
 * @param  IRQNumber: 中斷號碼
 * @param  IRQPriority: 中斷優先級 (0-15)
 * @return void
 * @note   STM32F4 只實作高 4 位元的優先級
 *****************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr_index = NVIC_IPR_INDEX(IRQNumber);
    uint8_t ipr_position = NVIC_IPR_SECTION(IRQNumber);
    uint8_t shift = (8 * ipr_position) + (8 - NO_IPR_BITS_IMPLEMENTED);

    *(NVIC_IPR_BASEADDR + ipr_index) |= (IRQPriority << shift);
}


/*****************************************************************
 * @fn     SPI_IRQHandling
 * @brief  SPI 中斷處理主函數
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @return void
 * @note   - 在各個 SPIx_IRQHandler 中呼叫此函數
 *         - 根據 SR 和 CR2 判斷中斷來源並處理
 *         - 可能的中斷來源：TXE, RXNE, OVR, MODF, FRE
 *****************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
    uint8_t Check_TXE, Check_TXEIE;

    /* ==================== 檢查 TXE 中斷 ==================== */
    Check_TXE = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    Check_TXEIE = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if(Check_TXE && Check_TXEIE)
    {
        /* 處理 TXE 中斷 (發送緩衝區為空) */
        spi_txe_interrupt_handle(pSPIHandle);
    }

    /* ==================== 檢查 RXNE 中斷 ==================== */
    uint8_t Check_RXNE, Check_RXNEIE;
    Check_RXNE = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    Check_RXNEIE = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if(Check_RXNE && Check_RXNEIE)
    {
        /* 處理 RXNE 中斷 (接收緩衝區非空) */
        spi_rxne_interrupt_handle(pSPIHandle);
    }

    /* ==================== 檢查 OVR 錯誤中斷 ==================== */
    uint8_t Check_OVR, Check_ERRIE;
    Check_OVR = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    Check_ERRIE = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if(Check_OVR && Check_ERRIE)
    {
        /* 處理 Overrun 錯誤 */
        spi_ovr_interrupt_handle(pSPIHandle);
    }
}


/* ==================== 7. 中斷處理輔助函數 ==================== */

/*****************************************************************
 * @fn     spi_txe_interrupt_handle
 * @brief  處理 TXE (Transmit buffer Empty) 中斷
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @return void
 * @note   - 此函數在 TXE 標誌被設置時由 SPI_IRQHandling 呼叫
 *         - 根據 DFF 位元判斷是 8-bit 或 16-bit 模式
 *         - 傳輸完成後關閉中斷並通知應用層
 *****************************************************************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    /* 檢查 CR1 DFF 位元，判斷是 8 bit 還是 16 bit 模式 */
    if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
    {
        /* ==================== 16 bit 模式 ==================== */
        /* 將 16 bit 資料寫入 Data Register (DR) */
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);

        /* 16 bit = 2 bytes，所以長度減 2 */
        pSPIHandle->TxLen -= 2;

        /* 指標移動 2 bytes */
        pSPIHandle->pTxBuffer += 2;
    }
    else
    {
        /* ==================== 8 bit 模式 ==================== */
        /* 將 8 bit 資料寫入 Data Register (DR) */
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;

        /* 長度減 1 byte */
        pSPIHandle->TxLen--;

        /* 指標移動 1 byte */
        pSPIHandle->pTxBuffer++;
    }

    /* TxLen 長度如果為 0，則關閉 SPI 傳輸，並通知應用層傳輸完成 */
    if(!(pSPIHandle->TxLen))
    {
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}


/*****************************************************************
 * @fn     spi_rxne_interrupt_handle
 * @brief  處理 RXNE (Receive buffer Not Empty) 中斷
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @return void
 * @note   - 此函數在 RXNE 標誌被設置時由 SPI_IRQHandling 呼叫
 *         - 根據 DFF 位元判斷是 8-bit 或 16-bit 模式
 *         - 接收完成後關閉中斷並通知應用層
 *****************************************************************/
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    /* 檢查 CR1 DFF 位元，判斷是 8 bit 還是 16 bit 模式 */
    if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
    {
        /* ==================== 16 bit 模式 ==================== */
        /* 讀取 Data Register (DR) 的 16 bit 數據 */
        *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;

        /* 16 bit = 2 bytes，所以長度減 2 */
        pSPIHandle->RxLen -= 2;

        /* 指標移動 2 bytes */
        pSPIHandle->pRxBuffer += 2;
    }
    else
    {
        /* ==================== 8 bit 模式 ==================== */
        /* 讀取 Data Register (DR) 的 8 bit 數據 */
        *pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;

        /* 長度減 1 byte */
        pSPIHandle->RxLen--;

        /* 指標移動 1 byte */
        pSPIHandle->pRxBuffer++;
    }

    /* RxLen 長度如果為 0，則關閉 SPI 接收，並通知應用層接收完成 */
    if(!(pSPIHandle->RxLen))
    {
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}


/*****************************************************************
 * @fn     spi_ovr_interrupt_handle
 * @brief  處理 OVR (Overrun) 錯誤中斷
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @return void
 * @note   - Overrun 發生於新數據到達但上一筆數據尚未讀取
 *         - 清除方法：依序讀取 DR 和 SR
 *         - 只在非傳輸狀態時清除 (避免干擾正在進行的傳輸)
 *****************************************************************/
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;

    /* 清除 OVR 標誌：依序讀取 DR 和 SR */
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;  /* 避免編譯器警告 unused variable */

    /* 通知應用層發生 Overrun 錯誤 */
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


/* ==================== 8. 狀態與控制函數 ==================== */

/*****************************************************************
 * @fn     SPI_GetFlagStatus
 * @brief  檢查 SPI 狀態暫存器 (SR) 中的特定標誌位
 * @param  pSPIx: SPIx 的基底位址
 * @param  FlagName: 要檢查的標誌位 (例如 SPI_FLAG_TXE, SPI_FLAG_RXNE)
 * @return uint8_t: FLAG_SET (1) 或 FLAG_RESET (0)
 * @note   常用標誌：TXE, RXNE, BSY, OVR, MODF
 *****************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }

    return FLAG_RESET;
}


/*****************************************************************
 * @fn     SPI_PeripheralControl
 * @brief  啟用或禁用 SPI 外設 (設定 SPE 位元)
 * @param  pSPIx: SPIx 的基底位址
 * @param  EnorDi: ENABLE 或 DISABLE
 * @return void
 * @note   - 必須在配置完成後才啟用 SPE
 *         - 啟用後不應修改大部分 CR1 的配置
 *****************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}


/*****************************************************************
 * @fn     SPI_SSIConfig
 * @brief  配置內部 NSS 訊號 (SSI 位元)
 * @param  pSPIx: SPIx 的基底位址
 * @param  EnorDi: ENABLE 或 DISABLE
 * @return void
 * @note   - 當 SSM=1 (軟體管理 NSS) 時，SSI 決定 NSS 的邏輯電平
 *         - SSI=1 表示 Master 模式，SSI=0 表示 Slave 模式
 *         - 防止 Master mode fault error
 *****************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}


/*****************************************************************
 * @fn     SPI_SSOEConfig
 * @brief  配置 NSS 輸出致能 (SSOE 位元)
 * @param  pSPIx: SPIx 的基底位址
 * @param  EnorDi: ENABLE 或 DISABLE
 * @return void
 * @note   - 當 SSM=0 且 SSOE=1 時，NSS 引腳由硬體自動管理
 *         - Master 模式：NSS 輸出低電平
 *         - Slave 模式：NSS 為輸入
 *****************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}


/* ==================== 9. 中斷關閉與清除函數 ==================== */

/*****************************************************************
 * @fn     SPI_CloseTransmission
 * @brief  關閉 SPI 傳輸並清理相關資源
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @return void
 * @note   - 禁用 TXEIE 中斷
 *         - 清空 buffer 指標和長度
 *         - 恢復 SPI 狀態為 READY
 *****************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    /* 禁用 TXE 中斷 */
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

    /* 清空 Tx buffer 指標 */
    pSPIHandle->pTxBuffer = NULL;

    /* 清空 Tx 長度 */
    pSPIHandle->TxLen = 0;

    /* 恢復狀態為 READY */
    pSPIHandle->TxState = SPI_READY;
}


/*****************************************************************
 * @fn     SPI_CloseReception
 * @brief  關閉 SPI 接收並清理相關資源
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @return void
 * @note   - 禁用 RXNEIE 中斷
 *         - 清空 buffer 指標和長度
 *         - 恢復 SPI 狀態為 READY
 *****************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    /* 禁用 RXNE 中斷 */
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

    /* 清空 Rx buffer 指標 */
    pSPIHandle->pRxBuffer = NULL;

    /* 清空 Rx 長度 */
    pSPIHandle->RxLen = 0;

    /* 恢復狀態為 READY */
    pSPIHandle->RxState = SPI_READY;
}


/*****************************************************************
 * @fn     SPI_ClearOVRFlag
 * @brief  清除 Overrun (OVR) 錯誤標誌
 * @param  pSPIx: SPIx 的基底位址
 * @return void
 * @note   - 清除方法：依序讀取 DR 和 SR
 *         - 根據 Reference Manual 的規定
 *****************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t temp;

    /* 讀取 DR */
    temp = pSPIx->DR;

    /* 讀取 SR */
    temp = pSPIx->SR;

    /* 避免編譯器警告 */
    (void)temp;
}


/* ==================== 10. 應用層回呼函數（弱定義）==================== */

/*****************************************************************
 * @fn     SPI_ApplicationEventCallback
 * @brief  SPI 事件回呼函數（弱定義）
 * @param  pSPIHandle: 指向 SPI_Handle_t 結構體的指標
 * @param  AppEv: 事件類型 (例如 SPI_EVENT_TX_CMPLT, SPI_EVENT_RX_CMPLT)
 * @return void
 * @note   - 此函數應該在應用層重新定義 (override)
 *         - 用於處理 SPI 中斷事件的應用層邏輯
 *         - 弱定義確保未定義時不會產生連結錯誤
 *****************************************************************/
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
    /* This is a week implementation. The application may override this function. */
}
