#include "stm32f429xx.h"

/* ==================== 延遲函數 ==================== */
void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

/*
*   Alternate Function Port : AF07
*   PD3 -> USART2_CTS
*   PD4 -> USART2_RTS
*   PD5 -> USART2_TX
*   PD6 -> USART2_RX
*   PD7 -> USART2_CK
*/
char msg[] = "USART2 Tx testing... \n";

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}


void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;

    usart_gpios.pGPIOx = GPIOD;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF07;
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    /* USART2 Tx */
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&usart_gpios);

    /* USART2 Rx */
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&usart_gpios);
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

    USART2_GPIOInit();

    USART2_Init();

    USART_PeripheralControl(USART2, ENABLE);

    while(1)
    {
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

        delay();

        USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));
    }

    return 0;
}
