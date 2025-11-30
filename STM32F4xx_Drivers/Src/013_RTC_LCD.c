#include "ds1307.h"
#include "lcd.h"

#include <stdio.h>

#define SYSTICK_TIMER_CLK     16000000UL

char* get_day_of_week(uint8_t i)
{
    char* days[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

    return days[i - 1];
}

void number_to_string(uint8_t num, char* buf)
{
    if(num < 10)
    {
        buf[0] = '0';
        buf[1] = num + 48; /* ASCII 要加 48 */
    }
    else if(num >= 10 && num < 99)
    {
        buf[0] = (num / 10) + 48;   /* 十位數 */
        buf[1] = (num % 10) + 48;   /* 個位數 */
    }
}

/* hh:mm:ss */
char* time_to_string(RTC_time_t *rtc_time)
{
    static char buf[9];

    buf[2] = ':';
    buf[5] = ':';

    number_to_string(rtc_time->hours, buf);
    number_to_string(rtc_time->minutes, &buf[3]);
    number_to_string(rtc_time->seconds, &buf[6]);

    buf[8] = '\0';

    return buf;
}

/* dd/mm/yy */
char* date_to_string(RTC_date_t *rtc_date)
{
    static char buf[9];

    buf[2] = '/';
    buf[5] = '/';

    number_to_string(rtc_date->date, buf);
    number_to_string(rtc_date->month, &buf[3]);
    number_to_string(rtc_date->year, &buf[6]);

    buf[8] = '\0';

    return buf;
}


void InitSystickTimer(uint32_t tick_hz)
{
    uint32_t *pSCSR = (uint32_t*)0xE000E010;
    uint32_t *pSRVR = (uint32_t*)0xE000E014;

    /* Calculation of reload value */
    uint32_t count_value = (SYSTICK_TIMER_CLK / tick_hz) - 1;

    /* Clear the value of SVR */
    *pSRVR &= ~(0x00FFFFFFFF);

    /* Load the value into SVR */
    *pSRVR |= count_value;

    /* Settings: Enable SysTick exception request */
    *pSCSR |= (1 << 1);

    /* Settings: Indicate processor clock source */
    *pSCSR |= (1 << 2);

    /* Settings: Enable SysTick counter */
    *pSCSR |= (1 << 0);
}

void SysTick_Handler(void)
{
    RTC_time_t current_time;
    RTC_date_t current_date;

    ds1307_get_current_date(&current_date);
    ds1307_get_current_time(&current_time);

    char *am_pm;
    lcd_set_cursor(1,1);
    if(current_time.time_format != TIME_FORMAT_24HRS)
    {
        am_pm = (current_time.time_format == TIME_FORMAT_12HRS_PM) ? "PM" : "AM";
        printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
        lcd_print_string(time_to_string(&current_time));
        lcd_print_string(am_pm);
    }
    else
    {
        printf("Current time = %s \n", time_to_string(&current_time));
    }

    lcd_set_cursor(2,1);
    lcd_print_string(date_to_string(&current_date));
    lcd_print_char('<');
    lcd_print_string(get_day_of_week(current_date.day));
    lcd_print_char('>');

    printf("Current date = %s <%s> \n", date_to_string(&current_date), get_day_of_week(current_date.day));
}



int main(void)
{
    RTC_time_t current_time;
    RTC_date_t current_date;

    printf("RTC test \n");

    lcd_init();
    lcd_print_string("RTC Test..");

    ms_delay(2000);
    lcd_display_clear();
    lcd_display_return_home();


    if(ds1307_init())
    {
        printf("RTC init has failed \n");
    }

    InitSystickTimer(1);

    current_date.day = FRIDAY;
    current_date.date = 1;
    current_date.month = 1;
    current_date.year = 25;

    current_time.hours = 10;
    current_time.minutes = 10;
    current_time.seconds = 0;
    current_time.time_format = TIME_FORMAT_12HRS_PM;

    ds1307_set_current_date(&current_date);
    ds1307_set_current_time(&current_time);

    ds1307_get_current_date(&current_date);
    ds1307_get_current_time(&current_time);

    char *am_pm;
    if(current_time.time_format != TIME_FORMAT_24HRS)
    {
        am_pm = (current_time.time_format == TIME_FORMAT_12HRS_PM) ? "PM" : "AM";
        printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
        lcd_print_string(time_to_string(&current_time));
        lcd_print_string(am_pm);
    }
    else
    {
        printf("Current time = %s \n", time_to_string(&current_time));
    }

    printf("Current date = %s <%s> \n", date_to_string(&current_date), get_day_of_week(current_date.day));

    lcd_set_cursor(2,1);
    lcd_print_string(date_to_string(&current_date));

    while(1);

}


