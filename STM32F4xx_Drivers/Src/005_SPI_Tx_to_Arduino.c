#include "stm32f429xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

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
    SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
    SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI1handle);
}

void GPIO_BUTTON_Init()
{
    GPIO_Handle_t GPIO_BUTTON = {0};

    GPIO_BUTTON.pGPIOx = GPIOC;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_BUTTON);
    return;
}

int main(void)
{
	GPIO_BUTTON_Init();

	/* 1. 初始化 GPIO */
    SPI1_GPIOInits();

    /* 2. 初始化 SPI */
    SPI1_Inits();

    SPI_SSOEConfig(SPI1, ENABLE);

    char data[] = "Test";
    uint8_t dataLen = strlen(data);
    while(1)
    {
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

        delay();

        /* 3. 啟用 SPI */
        SPI_PeripheralControl(SPI1, ENABLE);

        SPI_SendData(SPI1, &dataLen, 1);

        /* 4. 發送數據 */
        SPI_SendData(SPI1, (uint8_t*)data, strlen(data));

        /* 5. 等待傳輸完成 */
        while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));

        /* 6. 禁用 SPI */
        SPI_PeripheralControl(SPI1, DISABLE);

        delay();
    }

    while(1);
}
