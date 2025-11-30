#include "stm32f429xx.h"

/* ==================== SPI1 引腳配置（無衝突）==================== */
/*
*   Alternate Function Mode : AF05
*   SPI1_NSS  : PA4  (無衝突)
*   SPI1_SCK  : PA5  (無衝突)
*   SPI1_MISO : PA6  (無衝突)
*   SPI1_MOSI : PA7  (無衝突)
*/

void SPI1_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOA;  // 主要使用 GPIOC
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF05;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    /* ==================== SCK ==================== */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&SPIPins);

    /* ==================== MISO ==================== */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&SPIPins);

    /* ==================== MOSI ==================== */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&SPIPins);

    /* ==================== NSS ==================== */
    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&SPIPins);
}

void SPI1_Inits(void)
{
    SPI_Handle_t SPI1handle;

    SPI1handle.pSPIx = SPI1;
    SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
    SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
    SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
    SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

    SPI_Init(&SPI1handle);
}

int main(void)
{
    /* 1. 初始化 GPIO */
    SPI1_GPIOInits();

    /* 2. 初始化 SPI */
    SPI1_Inits();

    /* 3. 設定 SSI (軟體 NSS) */
    SPI_SSIConfig(SPI1, ENABLE);

    /* 4. 啟用 SPI */
    SPI_PeripheralControl(SPI1, ENABLE);

    /* 5. 發送數據 */
    char data[] = "Test";
    SPI_SendData(SPI1, (uint8_t*)data, strlen(data));

    /* 6. 等待傳輸完成 */
    while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));

    /* 7. 禁用 SPI */
    SPI_PeripheralControl(SPI1, DISABLE);

    while(1);
}
