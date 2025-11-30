#include "stm32f429xx.h"
#include <stdio.h>

#define SLAVE_ADDR      0x68
#define MY_ADDR     	SLAVE_ADDR

I2C_Handle_t I2C1Handle = {0};
uint8_t data[] = "STM32 Slave Arduino Master";

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
	GPIO_BUTTON_Init();

    I2C1_GPIOInits();

    I2C1_Inits();

    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

    I2C_PeripheralControl(I2C1, ENABLE);

    I2C_ManageACK(I2C1, I2C_ACK_ENABLE);

    while (1);

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
	static uint8_t CommandCode = 0;
	static uint8_t count = 0;
    if(AppEv == I2C_EV_DATA_REQ)
    {
        if(CommandCode == 0x51)
        {
            I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)data));
        }
        else if(CommandCode == 0x52)
        {
            I2C_SlaveSendData(pI2CHandle->pI2Cx, data[count++]);
        }
    }
    else if(AppEv == I2C_EV_DATA_RCV)
    {
        CommandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
    }
    else if(AppEv == I2C_ERROR_AF)
    {
        CommandCode = 0xff;
        count = 0;
    }
    else if(AppEv == I2C_EV_STOP)
    {

    }
}
