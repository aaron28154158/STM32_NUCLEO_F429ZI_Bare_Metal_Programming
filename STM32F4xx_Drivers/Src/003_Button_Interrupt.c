#include "stm32f429xx.h"

#define BTN_PRESSED     SET

GPIO_Handle_t GPIO_LED = {0}, GPIO_BUTTON = {0};

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
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_BUTTON);
    return;
}

int main(void)
{
    GPIO_LED_Init();
    GPIO_BUTTON_Init();

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI_15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

    while(1)
    {

    }

    return 0;
}

void EXTI15_10_IRQHandler(void)
{
    /* PC13 按鈕觸發中斷 */
    if(EXTI->PR & (1 << GPIO_PIN_NO_13))
    {
        /* 清除 Pending bit */
        GPIO_IRQHandling(GPIO_PIN_NO_13);

        /* 執行動作 */
        GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
    }
}
