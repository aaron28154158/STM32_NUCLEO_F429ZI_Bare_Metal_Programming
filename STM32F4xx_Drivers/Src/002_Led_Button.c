#include "stm32f429xx.h"

#define BTN_PRESSED     SET

GPIO_Handle_t GPIO_LED, GPIO_BUTTON;

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
    GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_LED);
    return;
}

void GPIO_BUTTON_Init()
{
    GPIO_BUTTON.pGPIOx = GPIOC;
    GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_BUTTON);
    return;
}

int main(void)
{
    GPIO_LED_Init();
    GPIO_BUTTON_Init();

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
        {
            delay();
            GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
        }

    }

    return 0;
}
