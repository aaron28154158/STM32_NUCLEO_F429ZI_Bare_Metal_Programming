#include "stm32f429xx.h"

GPIO_Handle_t GPIO_LED;

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

void GPIO_LED_Init()
{
    GPIO_LED.pGPIOx = GPIOB;
    GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

    GPIO_Init(&GPIO_LED);
    return;
}

int main(void)
{
    GPIO_LED_Init();

    while(1)
    {
        GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
        delay();
    }

    return 0;
}
