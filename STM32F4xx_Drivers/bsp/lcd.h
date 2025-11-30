#ifndef LCD_H_
#define LCD_H_

#include "stm32f429xx.h"

void lcd_init(void);
void lcd_display_clear(void);
void lcd_send_command(uint8_t cmd);
void lcd_display_return_home(void);
void lcd_print_string(char *message);
void lcd_print_char(uint8_t data);
void lcd_set_cursor(uint8_t row, uint8_t column);
void ms_delay(uint32_t count);
void us_delay(uint32_t count);

#define LCD_GPIO_PORT   GPIOD
#define LCD_GPIO_RS     GPIO_PIN_NO_0
#define LCD_GPIO_RW     GPIO_PIN_NO_1
#define LCD_GPIO_EN     GPIO_PIN_NO_2
#define LCD_GPIO_D4     GPIO_PIN_NO_3
#define LCD_GPIO_D5     GPIO_PIN_NO_4
#define LCD_GPIO_D6     GPIO_PIN_NO_5
#define LCD_GPIO_D7     GPIO_PIN_NO_6


/* =============== LCD 指令 =============== */

/* =============== 4 Bit Data Length / 2 Lines / 5 x 8 dots ===============*/
#define LCD_CMD_4DL_2N_5X8F             0x28


/* =============== 開啟 Display / 開啟 Cursor ===============*/
#define LCD_CMD_DisplayON_CursorON      0x0E

/* =============== 記憶體位址遞增 (Increment) ===============*/
#define LCD_CMD_INCADDR                 0x06

/* =============== 清除顯示 (Clear Display) ===============*/
#define LCD_CMD_DIS_CLEAR               0x01

/* =============== 顯示 Return Home ===============*/
#define LCD_CMD_DIS_RETURN_HOME         0x02


#endif
