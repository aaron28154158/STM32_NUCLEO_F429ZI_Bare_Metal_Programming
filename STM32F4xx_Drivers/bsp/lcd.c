#include "lcd.h"

static void write_4_bits(uint8_t value);
static void lcd_enable(void);

/*
 * @brief  發送指令到 LCD 
 * @param  cmd: 8 位元指令碼
 * @note   控制線設定：
 *         - RS=0: 選擇指令暫存器
 *         - RW=0: 寫入模式 
 */
void lcd_send_command(uint8_t cmd)
{
    /* =============== 指令模式 (RS = 0) =============== */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /* =============== 寫入模式 (RW = 0) =============== */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    /* =============== 4-bit 模式：分兩次傳輸 8 位元的指令 =============== */
    write_4_bits(cmd >> 4);     /* 傳送高 4 位 (4 ~ 7) */
    write_4_bits(cmd & 0x0F);   /* 傳送低 4 位 (0 ~ 3) */
}

/*
 * @brief  發送單個字元到 LCD 顯示
 * @param  data: 要顯示的 ASCII 字元 
 * @note   控制線設定：
 *         - RS=1: 選擇資料暫存器 
 *         - RW=0: 寫入模式 
 */
void lcd_print_char(uint8_t data)
{
    /* =============== 資料模式 (RS = 1) =============== */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

    /* =============== 寫入模式 (RW = 0) =============== */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    /* =============== 4-bit 模式：分兩次傳輸 8 位元的指令 =============== */
    write_4_bits(data >> 4);    /* 傳送高 4 位 (4 ~ 7) */
    write_4_bits(data & 0x0F);  /* 傳送低 4 位 (0 ~ 3) */
}


/*
 * @brief  在 LCD 當前游標位置顯示字串
 * @param  message: 指向以 '\0' 結尾的字串
 * @retval None
 * 
 * @note   使用前需要先使用 lcd_set_cursor() 設定游標位置
 */
void lcd_print_string(char *message)
{
	do
	{
		lcd_print_char((uint8_t) *message++);
	}while(*message != '\0');
}


/*
 * @brief  初始化 LCD 
 * @note   初始化流程 
 *         1. 上電後等待 > 40ms 讓電源穩定
 *         2. 發送 3 次 0x3 指令切換到 8-bit 模式
 *         3. 發送 0x2 切換到 4-bit 模式
 *         4. 設定功能：4-bit / 2-line / 5x8 font
 *         5. 設定顯示：Display ON / Cursor ON
 *         6. 清除顯示
 *         7. 設定進入模式：位址遞增 / 不偏移
 */
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

    /* 
     * 初始化序列：確保 LCD 從任何狀態復位
     * 發送 3 次 0x3 (Function Set: 8-bit mode)
     */
    write_4_bits(0x3); /* 0 0 1 1 */

    ms_delay(5);

    write_4_bits(0x3); /* 0 0 1 1 */

    us_delay(100);

    write_4_bits(0x3); /* 0 0 1 1 */

    us_delay(100);

    /* 切換到 4-bit 模式 */
    write_4_bits(0x2); /* 0 0 1 0 */

    us_delay(100);

    /* =============== 3. Function Set =============== */

    /* Function Set: 4-bit mode, 2 lines, 5x8 font */
    lcd_send_command(LCD_CMD_4DL_2N_5X8F);

    /* Display Control: Display ON, Cursor ON, Blink OFF */
    lcd_send_command(LCD_CMD_DisplayON_CursorON);

    /* Clear Display: 清除所有顯示內容, 延遲時間較久，所以獨立封裝 */
    lcd_display_clear();

    /* Entry Mode Set: Increment mode, No shift */
    lcd_send_command(LCD_CMD_INCADDR);

}


/*
 * @brief  寫入 4 個位元到 LCD 資料線 
 * @param  value: 8 位元數值，僅使用 4 位
 */
static void write_4_bits(uint8_t value)
{
    /* 將 4-bit 的資料分別寫入GPIO (D4 ~ D7) */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1) );
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1) );

    /* 寫入 EN, 產生 Enable 脈衝，LCD 在下降緣鎖存資料 */
    lcd_enable();
}



/*
 * @brief  清除LCD顯示內容並將游標移至起始位置
 * @note   需要 1.52ms
 */
void lcd_display_clear(void)
{
    lcd_send_command(LCD_CMD_DIS_CLEAR);

    ms_delay(2);
}

/*
 * @brief  游標返回起始位置，不清除顯示內容
 * @note   需要 1.52ms (根據 HD44780U datasheet)
 */
void lcd_display_return_home(void)
{
    lcd_send_command(LCD_CMD_DIS_RETURN_HOME);

    ms_delay(2);
}


/*
 * @brief  設定游標位置
 * @note   指令格式： 1 A A A A A A A (1 的位置是指令識別位元)
 */
void lcd_set_cursor(uint8_t row, uint8_t column)
{
    column--; /* 使用者輸入 1~16，轉換為 0~15 */ 

    switch(row)
    {
        case 1:
            /* column = 0~15 (偏移量) */
            /* 0x80 = 指令識別位元 + 0x00 (第1行基底位址) */
            /* 0x80 | column = 0x80 ~ 0x8F */ 
            lcd_send_command((column | 0x80));
            break;
        case 2:
            /* column = 0~15 (偏移量) */ 
            /* 0xC0 = 指令識別位元 + 0x40 (第 2 行基底位址) */
            /* 0xC0 | column = 0xC0 ~ 0xCF */ 
            lcd_send_command((column | 0xC0));
            break;

        default:
            break;
    }
}


/*
 * @brief  產生Enable脈衝信號以鎖存資料到LCD
 * @param  None
 */
static void lcd_enable(void)
{
    /* =============== 1. EN 拉高：開始傳輸 =============== */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);

    /* =============== 2. 確保資料穩定 =============== */
    us_delay(10);

    /* =============== 3. EN 拉低：下降緣鎖存資料到 LCD 內部暫存器 =============== */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);

    /* =============== 4. 等待指令執行完畢, 執行時間 > 37 us =============== */
    us_delay(100); 
}



void ms_delay(uint32_t count)
{
    for(uint32_t i = 0; i < (count * 1000); i++);
}

void us_delay(uint32_t count)
{
    for(uint32_t i = 0; i < (count * 1); i++);
}
