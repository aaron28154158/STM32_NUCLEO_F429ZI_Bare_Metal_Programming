#include "stm32f429xx_gpio_driver.h"

/* ==================== 1. 初始化與重置 ==================== */
/*****************************************************************
 * @fn     GPIO_Init
 * @brief  根據 pGPIOHandle 中的設定來初始化指定的 GPIO 引腳
 * @param  pGPIOHandle: 指向 GPIO handle 結構體的指標，包含 Port 和 Pin 的配置
 * @return void
 * @note   使用前必須先呼叫 GPIO_PeriClockControl() 啟用時脈
 *****************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    uint32_t temp = 0;
    uint8_t pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    /* ==================== 1. 配置引腳模式 (MODER) ==================== */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        /* 非中斷模式：Input(00) / Output(01) / AF(10) / Analog(11) */
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pin));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pin));   /* 清除對應的 2 bits */
        pGPIOHandle->pGPIOx->MODER |= temp;                  /* 設定新值 */
    }
    else
    {
        /* ==================== 1.1 中斷模式配置：引腳必須是輸入模式 ==================== */
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pin));  /* 清除為 00 (輸入模式) */

        /* ==================== 1.2 配置 EXTI 觸發邊緣 ==================== */
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            /* ==================== 下降沿觸發 (Falling Trigger) ==================== */
            EXTI->FTSR |= (1 << pin);       /* 啟用下降沿觸發 */
            EXTI->RTSR &= ~(1 << pin);      /* 清除上升沿觸發 */
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            /* ==================== 上升沿觸發 (Rising Trigger) ==================== */
            EXTI->RTSR |= (1 << pin);       /* 啟用上升沿觸發 */
            EXTI->FTSR &= ~(1 << pin);      /* 清除下降沿觸發 */
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            /* ==================== 雙邊緣觸發 (Rising + Falling Trigger) ==================== */
            EXTI->RTSR |= (1 << pin);       /* 啟用上升沿觸發 */
            EXTI->FTSR |= (1 << pin);       /* 啟用下降沿觸發 */
        }

        /* ==================== 設定 EXTICR (選擇哪一個 Port 的 Pin 可以連接到 EXTI 線路上)==================== */
        uint8_t exticr_index = pin / 4;         /* 選擇 EXTICR[0-3] */
        uint8_t exticr_position = pin % 4;      /* 在暫存器中的位置 (0-3) */
        uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); /* 將 GPIO 基底位址轉換成 Port 編號, GPIOA = 0, GPIOB = 1 ...*/
        SYSCFG_PCLK_EN();   /* 啟用 SYSCFG 時脈 */
        SYSCFG->EXTICR[exticr_index] &= ~(0xF << (4 * exticr_position));      /* 清除對應的 4 bits */
        SYSCFG->EXTICR[exticr_index] |= (port_code << (4 * exticr_position)); /* 設定 Port 編號 */

        /* ==================== 啟用 EXTI 中斷遮罩 ==================== */
        EXTI->IMR |= (1 << pin);
    }

    /* ==================== 2. 配置輸出速度 (OSPEEDR) ==================== */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pin));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pin)); /* 清除對應的 2 bits */
    pGPIOHandle->pGPIOx->OSPEEDR |= temp; /* 設定新值 */

    /* ==================== 3. 配置上拉/下拉 (PUPDR) ==================== */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pin));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pin)); /* 清除對應的 2 bits */
    pGPIOHandle->pGPIOx->PUPDR |= temp; /* 設定新值 */

    /* ==================== 4. 配置輸出類型 (OTYPER) ==================== */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pin);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pin);  /* 清除對應的 1 bits */
    pGPIOHandle->pGPIOx->OTYPER |= temp; /* 設定新值 */

    /* ==================== 5. 配置複用功能 (AFR) ==================== */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        /* AFR[0] 對應 Pin 0-7, AFR[1] 對應 Pin 8-15 */
        uint8_t afr_index = pin / 8;       /* 選擇 AFRL(0) 或 AFRH(1) */
        uint8_t afr_position = pin % 8;    /* 在該暫存器中的位置 (0-7) */

        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * afr_position));
        pGPIOHandle->pGPIOx->AFR[afr_index] &= ~(0xF << (4 * afr_position));  /* 清除 4 bits */
        pGPIOHandle->pGPIOx->AFR[afr_index] |= temp; /* 設定新值 */
    }
}


/*****************************************************************
 * @fn     GPIO_DeInit
 * @brief  將指定的 GPIO Port 所有暫存器重設為預設值 (Reset)
 * @param  pGPIOx: GPIO Port 的基底位址 (例如 GPIOA, GPIOB)
 * @return void
 * @note   此函式會透過 RCC 的 AHB1RSTR 暫存器實現硬體重置
 *****************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOx); /* 將 GPIO 基底位址轉換成 Port 編號, GPIOA = 0, GPIOB = 1 ...*/
    GPIOx_REG_RESET(port_code); /* 重置指定的 GPIO */
}


/* ==================== 2. 時脈設定（啟用或禁用）==================== */
/*****************************************************************
 * @fn     GPIO_PeriClockControl
 * @brief  啟用或禁用指定 GPIO Port 的周邊時脈
 * @param  pGPIOx: GPIO Port 的基底位址 (例如 GPIOA, GPIOB)
 * @param  EnorDi: ENABLE 或 DISABLE (通常是 1 或 0)
 * @return void
 * @note   1. 配置 GPIO 之前必須先啟用時脈
 *         2. 使用 GPIO_BASEADDR_TO_CODE 將 GPIO 基底位址轉換成 Port 編號
 *****************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOx); /* 將 GPIO 基底位址轉換成 Port 編號, GPIOA = 0, GPIOB = 1 ...*/

    if(EnorDi == ENABLE)
    {
        GPIOx_PCLK_EN(port_code); /* 啟用指定的 GPIO 的時脈*/
    }
    else
    {
        GPIOx_PCLK_DI(port_code); /* 禁用指定的 GPIO 的時脈*/
    }
}


/* ==================== 3. 數據讀取 ==================== */
/*****************************************************************
 * @fn     GPIO_ReadFromInputPin
 * @brief  從指定的 GPIO 輸入引腳讀取狀態 (高電位或低電位)
 * @param  pGPIOx: GPIO Port 的基底位址
 * @param  PinNumber: 引腳編號 (0 ~ 15)
 * @return uint8_t: 讀取到的值 (0 或 1)
 * @note   讀取 IDR (Input Data Register)
 *****************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 1);

    return value;
}

/*****************************************************************
 * @fn     GPIO_ReadFromInputPort
 * @brief  從指定的 GPIO Port 讀取 16-bit 的輸入狀態
 * @param  pGPIOx: GPIO Port 的基底位址
 * @return uint16_t: 16-bit 的 Port 輸入值
 * @note   直接回傳 IDR (Input Data Register) 的值
 *****************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value = (uint16_t)pGPIOx->IDR;

    return value;
}


/* ==================== 4. 數據寫入 ==================== */
/*****************************************************************
 * @fn     GPIO_WriteToOutputPin
 * @brief  指定 GPIO 引腳的輸出狀態 (高電位或低電位)
 * @param  pGPIOx: GPIO Port 的基底位址
 * @param  PinNumber: 引腳編號 (0 ~ 15)
 * @param  Value: 寫入的值 (0 或 1)
 * @return void
 * @note   可以使用 BSRR 暫存器進行原子操作
 *         BSRR[15:0]  = Set bits   (寫 1 設定對應引腳為高電位)
 *         BSRR[31:16] = Reset bits (寫 1 設定對應引腳為低電位)
 *****************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        //pGPIOx->ODR |= (GPIO_PIN_SET << PinNumber);
        pGPIOx->BSRR = (1 << PinNumber);
    }
    else
    {
        //pGPIOx->ODR &= ~(GPIO_PIN_SET << PinNumber);
        pGPIOx->BSRR = (1 << (PinNumber + 16));
    }
}


/*****************************************************************
 * @fn     GPIO_WriteToOutputPort
 * @brief  設定整個 GPIO Port 的輸出狀態
 * @param  pGPIOx: GPIO Port 的基底位址
 * @param  Value: 16-bit 的 Port 輸出值
 * @return void
 *****************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}


/*****************************************************************
 * @fn     GPIO_ToggleOutputPin
 * @brief  反轉指定 GPIO 引腳的輸出狀態
 * @param  pGPIOx: GPIO Port 的基底位址
 * @param  PinNumber: 欲翻轉的引腳編號 (0 ~ 15)
 * @return void
 *****************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}



/* ==================== 5. 中斷設定與中斷處理 ==================== */
/*****************************************************************
 * @fn     GPIO_IRQInterruptConfig
 * @brief  配置 NVIC 中斷 (啟用/禁用)
 * @param  IRQNumber: 中斷請求編號，參考 NVIC 中斷向量表
 * @param  EnorDi: ENABLE 或 DISABLE (1 或 0)
 * @return void
 *****************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn     GPIO_IRQPriorityConfig
 * @brief  配置 NVIC 中斷優先級
 * @param  IRQNumber: 中斷請求編號，參考 NVIC 中斷向量表
 * @param  IRQPriority: 中斷優先級 (0 ~ 15)
 * @return void
 * @note   8-bit 優先級欄位結構（STM32F4 實現 4 bits）：
           bit:  7    6    5    4  | 3    2    1    0
           ┌────┬────┬────┬────┬────┬────┬────┬────┐
           │ ✓  │ ✓  │ ✓  │ ✓  │ ✗  │ ✗  │ ✗  │ ✗  │
           └────┴────┴────┴────┴────┴────┴────┴────┘
           實現的位元             未實現的低位元
           (high-order)         (low-order)
           可讀寫                讀取=0, 寫入被忽略
 *****************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr_index = NVIC_IPR_INDEX(IRQNumber);
    uint8_t ipr_position = NVIC_IPR_SECTION(IRQNumber);
    uint8_t shift = (8 * ipr_position) + (8 - NO_IPR_BITS_IMPLEMENTED);

    *(NVIC_IPR_BASEADDR + ipr_index) |= (IRQPriority << shift);
}


/*****************************************************************
 * @fn     GPIO_IRQHandling
 * @brief  中斷服務常式 (ISR) 的處理函式
 * @param  PinNumber: 觸發中斷的引腳編號 (0 ~ 15)
 * @return void
 * @note   此函式應在 EXTI 的 ISR (例如 EXTI0_IRQHandler) 中被呼叫，
 *         用於清除 EXTI 的 pending bit 並執行用戶自定義的回調 (callback)。
 *****************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
    /* ==================== 清除 EXTI_PR (Pending Register) ==================== */
    /* 寫 1 清除 */
    EXTI->PR = (1 << PinNumber);
}
