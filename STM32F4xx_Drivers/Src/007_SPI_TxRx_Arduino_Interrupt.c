/*
 * SPI_TxRx_Arduino_Interrupt_F429ZI.c
 *
 * 描述：
 *   STM32F429ZI 透過 SPI1 以中斷模式與 Arduino 通訊
 *   Arduino 傳送資料後會拉高 PD0，觸發 STM32 讀取
 *
 * 硬體連接：
 *   STM32F429ZI (Master) <--> Arduino (Slave)
 *   - PA5 (SPI1_SCK)   -> Arduino SCK
 *   - PA6 (SPI1_MISO)  <- Arduino MISO
 *   - PA7 (SPI1_MOSI)  -> Arduino MOSI
 *   - PA4 (SPI1_NSS)   -> Arduino SS
 *   - PD0 (GPIO)       <- Arduino Data Available 中斷信號
 *   - PC13 (USER BTN)  : 板載按鈕（可用於其他功能）
 *
 * 注意：
 *   1. 在 Arduino 上傳 003SPISlaveUartReadOverSPI.ino
 *   2. 重置兩塊板子
 *   3. 啟用 SWV ITM data console 查看接收到的訊息
 *   4. 在 Arduino IDE Serial Monitor 輸入訊息並發送
 *      （確保 line ending 設定為 carriage return）
 */

#include <stdio.h>
#include <string.h>
#include "stm32f429xx.h"
#include "stm32f429xx_spi_driver.h"
#include "stm32f429xx_gpio_driver.h"


/* ==================== 全局變量 ==================== */
SPI_Handle_t SPI1Handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];              /* 接收緩衝區 */
volatile char ReadByte;             /* 單字節讀取緩衝 */
volatile uint8_t rcvStop = 0;       /* 接收完成標誌 */
volatile uint8_t dataAvailable = 0; /* Arduino 資料準備好標誌 */


/* ==================== 延遲函數 ==================== */
void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}


/* ==================== SPI1 GPIO 初始化 ==================== */
/*****************************************************************
 * @brief  配置 SPI1 的 GPIO 引腳
 * @note   PA5 -> SCK, PA6 -> MISO, PA7 -> MOSI, PA4 -> NSS
 *         複用功能 AF5
 *****************************************************************/
void SPI1_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;          /* AF5 for SPI1 */
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    /* ========== SCK Init (PA5) ========== */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&SPIPins);

    /* ========== MISO Init (PA6) ========== */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&SPIPins);

    /* ========== MOSI Init (PA7) ========== */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&SPIPins);

    /* ========== NSS Init (PA4) ========== */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&SPIPins);
}


/* ==================== SPI1 初始化 ==================== */
/*****************************************************************
 * @brief  配置 SPI1 外設參數
 * @note   - Master 模式
 *         - 全雙工
 *         - 8-bit 數據格式
 *         - CPOL=0, CPHA=0
 *         - 硬體 NSS 管理
 *****************************************************************/
void SPI1_Inits(void)
{
    SPI1Handle.pSPIx = SPI1;
    SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;  /* 根據需求調整 */
    SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;  /* 硬體 NSS 管理 */

    SPI_Init(&SPI1Handle);
}


/* ==================== Arduino Data Available 中斷引腳初始化 ==================== */
/*****************************************************************
 * @brief  配置 PD0 為 Arduino data available 中斷輸入
 * @note   - Arduino 準備好資料後會拉高此引腳
 *         - 上升沿觸發中斷
 *****************************************************************/
void Slave_GPIO_InterruptPinInit(void)
{
    GPIO_Handle_t spiIntPin;
    memset(&spiIntPin, 0, sizeof(spiIntPin));

    spiIntPin.pGPIOx = GPIOD;
    spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;  /* 上升沿觸發 */
    spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&spiIntPin);

    /* 配置 NVIC */
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
}


/* ==================== 主函數 ==================== */
int main(void)
{
    uint8_t dummy = 0xFF;
    uint8_t dataByte;

    Slave_GPIO_InterruptPinInit();
    SPI1_GPIOInits();
    SPI1_Inits();
    SPI_SSOEConfig(SPI1, ENABLE);

    printf("Start... \n");
    // 不需要啟用 SPI 中斷
    // SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);

    while(1)
    {
        /* 等待 Arduino 資料準備好 */
        while(!dataAvailable);
        printf("Data available detected!\n");

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, DISABLE);

        /* 啟用 SPI */
        SPI_PeripheralControl(SPI1, ENABLE);
        printf("SPI enabled, NSS should be LOW now\n");


        /* 接收第一個字節（0xf1） */
        SPI_SendData(SPI1, &dummy, 1);
        SPI_ReceiveData(SPI1, &dataByte, 1);

        if(dataByte != 0xf1) {
            /* 協議錯誤 */
            SPI_PeripheralControl(SPI1, DISABLE);
            dataAvailable = 0;
            GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
            continue;
        }

        /* 逐字節接收資料 */
        uint32_t idx = 0;
        do {
            SPI_SendData(SPI1, &dummy, 1);
            SPI_ReceiveData(SPI1, &dataByte, 1);
            RcvBuff[idx++] = dataByte;
        } while(dataByte != '\0' && idx < MAX_LEN);

        RcvBuff[idx-1] = '\0';  /* 確保結尾 */

        /* 等待 SPI 完成 */
        while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));

        /* 禁用 SPI */
        SPI_PeripheralControl(SPI1, DISABLE);

        /* 輸出資料 */
        printf("Rcvd data = %s\n", RcvBuff);

        /* 重置標誌 */
        dataAvailable = 0;
        GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
    }

    return 0;
}


/* ==================== SPI1 中斷處理函數 ==================== */
/*****************************************************************
 * @brief  SPI1 中斷服務常式
 * @note   在 stm32f429xx_it.c 中定義的 SPI1_IRQHandler 會呼叫此函數
 *****************************************************************/
void SPI1_IRQHandler(void)
{
    SPI_IRQHandling(&SPI1Handle);
}


/* ==================== SPI 應用層事件回呼函數 ==================== */
/*****************************************************************
 * @brief  SPI 中斷事件回呼
 * @param  pSPIHandle: SPI Handle 指標
 * @param  AppEv: 事件類型
 * @note   - 此函數覆寫 driver 中的弱定義
 *         - 接收完成後，將資料存入緩衝區
 *         - 遇到 '\0' 或達到最大長度時停止接收
 *****************************************************************/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    static uint32_t i = 0;

    /* ==================== RX 完成事件 ==================== */
    if(AppEv == SPI_EVENT_RX_CMPLT)
    {
        RcvBuff[i++] = ReadByte;

        /* 檢查是否為字串結束符號或達到最大長度 */
        if(ReadByte == '\0' || (i == MAX_LEN))
        {
            rcvStop = 1;
            RcvBuff[i-1] = '\0';  /* 確保字串結束 */
            i = 0;
        }
    }
}


/* ==================== Arduino Data Available 中斷處理 ==================== */
/*****************************************************************
 * @brief  EXTI0 中斷服務常式（對應 PD0）
 * @note   Arduino 拉高 PD0 時觸發，表示有資料可讀取
 *****************************************************************/
void EXTI0_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_0);
    dataAvailable = 1;
}
