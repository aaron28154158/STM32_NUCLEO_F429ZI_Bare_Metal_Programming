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


int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);

	GPIO_BUTTON_Init();

    I2C1_GPIOInits();

    I2C1_Inits();

    I2C_PeripheralControl(I2C1, ENABLE);

    /* PE = 1 時，才能設置 ACK */
    I2C_ManageACK(I2C1, I2C_ACK_ENABLE);

    uint8_t CommandCode;
    uint8_t Len;
    uint8_t ReceiveBuf[32];
    while(1)
    {
    	while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

    	delay();

        CommandCode = 0x51;
        I2C_MasterSendData(&I2C1Handle, &CommandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        I2C_MasterReceiveData(&I2C1Handle, &Len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        CommandCode = 0x52;
        I2C_MasterSendData(&I2C1Handle, &CommandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        I2C_MasterReceiveData(&I2C1Handle, ReceiveBuf, Len, SLAVE_ADDR, I2C_DISABLE_SR);

        ReceiveBuf[Len] = '\0';

        printf("%d \n", Len);
        printf("%s \n", ReceiveBuf);
    }

}
