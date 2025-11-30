#include "lcd.h"

static void write_4_bits(uint8_t value);
static void lcd_enable(void);


void lcd_send_command(uint8_t cmd)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(cmd >> 4);
    write_4_bits(cmd & 0x0F);
}


void lcd_print_char(uint8_t data)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(data >> 4);
    write_4_bits(data & 0x0F);
}


void lcd_init(void)
{
    GPIO_Handle_t lcd_signal;

    lcd_signal.pGPIOx = LCD_GPIO_PORT;
    lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
    GPIO_Init(&lcd_signal);

    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);



    /* =============== 2. LCD Init =============== */
    ms_delay(40);

    /*
    *   RS R/W DB7 DB6 DB5 DB4
    *    0   0   0   0   1   1
    */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(0x3); /* 0 0 1 1 */

    ms_delay(5);

    write_4_bits(0x3); /* 0 0 1 1 */

    us_delay(100);

    write_4_bits(0x3); /* 0 0 1 1 */

    us_delay(100);

    write_4_bits(0x2); /* 0 0 1 0 */

    us_delay(100);

    /* =============== 3. Function Set =============== */
    lcd_send_command(LCD_CMD_4DL_2N_5X8F);

    lcd_send_command(LCD_CMD_DisplayON_CursorON);

    lcd_display_clear();

    lcd_send_command(LCD_CMD_INCADDR);


}

static void write_4_bits(uint8_t value)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1) );

    lcd_enable();
}


void lcd_print_string(char *message)
{
	do
	{
		lcd_print_char((uint8_t) *message++);
	}while(*message != '\0');
}


void lcd_display_clear(void)
{
    lcd_send_command(LCD_CMD_DIS_CLEAR);

    ms_delay(2);
}


void lcd_display_return_home(void)
{
    lcd_send_command(LCD_CMD_DIS_RETURN_HOME);

    ms_delay(2);
}


void lcd_set_cursor(uint8_t row, uint8_t column)
{
    column--;
    switch(row)
    {
        case 1:
            /* Set cursor to 1st row address and add index */
            lcd_send_command((column |= 0x80));
            break;
        case 2:
            /* Set cursor to 2nd row address and add index */
            lcd_send_command((column |= 0xC0));
            break;

        default:
            break;
    }
}


static void lcd_enable(void)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
    us_delay(10);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    us_delay(100); /* 執行時間 > 37 us */
}



void ms_delay(uint32_t count)
{
    for(uint32_t i = 0; i < (count * 1000); i++);
}

void us_delay(uint32_t count)
{
    for(uint32_t i = 0; i < (count * 1); i++);
}
