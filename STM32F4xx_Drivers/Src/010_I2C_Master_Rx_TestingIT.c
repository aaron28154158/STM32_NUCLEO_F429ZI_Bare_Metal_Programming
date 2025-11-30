#include "stm32f429xx.h"
#include <stdio.h>

#define SLAVE_ADDR      0x68

/* ==================== 延遲函數 ==================== */
void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

/*
*   PB8 -> I2C1_SCL
*   PB9 -> I2C1_SDA
*/

void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF04;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    /* SCL */
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_Init(&I2CPins);

    /* SDA */
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&I2CPins);
}

I2C_Handle_t I2C1Handle = {0};
#define MY_ADDR     0x61

void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handle);
}

void GPIO_BUTTON_Init(void)
{
    GPIO_Handle_t GPIO_BUTTON = {0};

    GPIO_BUTTON.pGPIOx = GPIOC;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_BUTTON);
}

volatile uint8_t len_rx_complete = RESET;
volatile uint8_t data_rx_complete = RESET;

int main(void)
{
	GPIO_BUTTON_Init();

    I2C1_GPIOInits();

    I2C1_Inits();

    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    I2C_PeripheralControl(I2C1, ENABLE);

    I2C_ManageACK(I2C1, I2C_ACK_ENABLE);

    uint8_t CommandCode;
    uint8_t Len;
    uint8_t ReceiveBuf[32];
    while(1)
    {
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
        delay();

        /* 發送命令 0x51 */
        CommandCode = 0x51;
        while(I2C_MasterSendDataIT(&I2C1Handle, &CommandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        /* 接收長度 */
        len_rx_complete = RESET;
        while(I2C_MasterReceiveDataIT(&I2C1Handle, &Len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
        while(len_rx_complete != SET);  // 等待長度接收完成

        /* 發送命令 0x52 */
        CommandCode = 0x52;
        while(I2C_MasterSendDataIT(&I2C1Handle, &CommandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        /* 接收數據 */
        data_rx_complete = RESET;
        while(I2C_MasterReceiveDataIT(&I2C1Handle, ReceiveBuf, Len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);
        while(data_rx_complete != SET);  // 等待數據接收完成

        ReceiveBuf[Len] = '\0';
        printf("Data: %s\n", ReceiveBuf);
    }

}

void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    static uint8_t rx_count = 0;

    if(AppEv == I2C_EV_TX_CMPLT)
    {
        printf("Tx is Completed\n");
    }
    else if(AppEv == I2C_EV_RX_CMPLT)
    {
        rx_count++;
        if(rx_count == 1)
        {
            printf("Length Rx completed\n");
            len_rx_complete = SET;
        }
        else if(rx_count == 2)
        {
            printf("Data Rx completed\n");
            data_rx_complete = SET;
            rx_count = 0;  // 重置計數
        }
    }
    else if(AppEv == I2C_ERROR_AF)
    {
        printf("Error: ACK failure\n");
        I2C_CloseSendData(pI2CHandle);
        I2C_GenerateStopCondition(I2C1);
        while(1);
    }
}
