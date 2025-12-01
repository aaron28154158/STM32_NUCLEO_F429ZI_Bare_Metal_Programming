#include "ds1307.h"
#include "lcd.h"

#include <stdio.h>

/* ==================== 系統時脈 16 MHz ==================== */
#define SYSTICK_TIMER_CLK     16000000UL

/* ==================== 全域變數, 中斷旗標使用 ==================== */
volatile uint8_t g_update_display_flag = 0;

char* get_day_of_week(uint8_t i)
{
    char* days[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

    return days[i - 1];
}


/*
 * @brief  將時間結構轉換為字串 hh:mm:ss
 * @param  rtc_time: 時間結構指標
 * @param  buf: 存放結果
 * @param  size: buf 大小
 * @return 寫入的字元數
 */
void time_to_string(RTC_time_t *rtc_time, char* buf, size_t size)
{
    snprintf(buf, size, "%02d:%02d:%02d",
                        rtc_time->hours,
                        rtc_time->minutes,
                        rtc_time->seconds);
}

/*
 * @brief  將日期結構轉換為字串 dd:mm:yy
 * @param  rtc_date: 日期結構指標
 * @param  buf: 存放結果
 * @param  size: buf 大小
 * @return 寫入的字元數
 */
void date_to_string(RTC_date_t *rtc_date, char* buf, size_t size)
{
    snprintf(buf, size, "%02d/%02d/%02d",
                        rtc_date->date,
                        rtc_date->month,
                        rtc_date->year);
}


/*
 * @brief  初始化 SysTick 定時器
 * @param  tick_hz: 期望的中斷頻率 (Hz)
 * @return None
 */
void InitSystickTimer(uint32_t tick_hz)
{
    uint32_t *pSCSR = (uint32_t*)0xE000E010;
    uint32_t *pSRVR = (uint32_t*)0xE000E014;

    /* ==================== 計算重載值 ====================
    * - tick_hz = 1 (每秒 1 次)
    *   count_value = (16,000,000 / 1) - 1 = 15,999,999
    *   週期 = 16,000,000 / 16,000,000 = 1 秒
    * 
    * - tick_hz = 1000 (每毫秒 1 次)
    *   count_value = (16,000,000 / 1000) - 1 = 15,999
    *   週期 = 16,000 / 16,000,000 = 0.001 秒 = 1 ms
    */
    uint32_t count_value = (SYSTICK_TIMER_CLK / tick_hz) - 1;

    /* ==================== 清除 SRVR 暫存器 ==================== */
    *pSRVR &= ~(0x00FFFFFF);

    /* ==================== 載入重載值 ==================== */
    *pSRVR |= count_value;


    /* ==================== 配置 SysTick Control Register ==================== */

    *pSCSR |= (1 << 1); /* Bit 1: TICKINT - 啟用 SysTick 異常請求 */

    *pSCSR |= (1 << 2); /* Bit 2: CLKSOURCE - 選擇時脈來源 */

    *pSCSR |= (1 << 0); /* Bit 0: ENABLE - 啟用 SysTick 計數器 */
}


void SysTick_Handler(void)
{
    g_update_display_flag = 1;
}

void update_display(void)
{
    RTC_time_t current_time;
    RTC_date_t current_date;
    
    ds1307_get_current_date(&current_date);
    ds1307_get_current_time(&current_time);

    char time_str[9], date_str[9]; 
    time_to_string(&current_time, time_str, sizeof(time_str));
    date_to_string(&current_date, date_str, sizeof(date_str));

    /* 顯示時間 (12/24 小時制) */
    char *am_pm = "";
    if(current_time.time_format != TIME_FORMAT_24HRS) {
        am_pm = (current_time.time_format == TIME_FORMAT_12HRS_PM) ? "PM" : "AM";
    }

    /* 更新 LCD 第一行：時間 */
    lcd_set_cursor(1, 1);
    lcd_print_string(time_str);
    if(current_time.time_format != TIME_FORMAT_24HRS) {
        lcd_print_char(' ');
        lcd_print_string(am_pm);
    }
    
    /* 更新 LCD 第二行：日期與星期 */
    lcd_set_cursor(2, 1);
    lcd_print_string(date_str);
    lcd_print_char(' ');
    lcd_print_char("<"); 
    lcd_print_string(get_day_of_week(current_date.day));
    lcd_print_char(">");
}

int main(void)
{
    RTC_time_t current_time;
    RTC_date_t current_date;

    /* ==================== 初始化 LCD ==================== */
    lcd_init();

    /* ==================== 初始化 DS1307 ==================== */
    if(ds1307_init())
    {
        printf("RTC init has failed \n");
        while(1);
    }

    /* ==================== 設定初始時間和日期 ==================== */
    current_date.day = FRIDAY;
    current_date.date = 15;
    current_date.month = 1;
    current_date.year = 21;

    current_time.hours = 4;
    current_time.minutes = 25;
    current_time.seconds = 50;
    current_time.time_format = TIME_FORMAT_12HRS_PM;

    ds1307_set_current_time(&current_time);
    ds1307_set_current_date(&current_date);

    /* ==================== 第一次顯示 ==================== */
    update_display();

    /* ==================== 啟動 SysTick (1 Hz) ==================== */
    InitSystickTimer(1);
    while(1)
    {
        if(g_update_display_flag)
        {
            g_update_display_flag = 0;  // 清除旗標
            update_display();           // 執行更新
        }
    }
}
