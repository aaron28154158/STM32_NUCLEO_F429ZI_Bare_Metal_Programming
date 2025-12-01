#include "ds1307.h"
#include <stdio.h>


static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t value);
static uint8_t bcd_to_binary(uint8_t value);


I2C_Handle_t g_ds1307_I2CHandle;

/**
 * @brief  初始化 DS1307 RTC 模組
 * @param  None
 * @return 0: CH = 0, 初始化成功 
 *         1: CH = 1, 初始化失敗 
 */
uint8_t ds1307_init(void)
{
    /* ==================== 1. 初始化 GPIO I2C 引腳 ==================== */
    ds1307_i2c_pin_config();

    /* ==================== 2. 初始化 I2C ==================== */
    ds1307_i2c_config();

    /* ==================== 3. 啟用 I2C ==================== */
    I2C_PeripheralControl(I2C1, ENABLE);

    /* ==================== 4. 寫入 CH (clock halt) = 0 ==================== */
    ds1307_write(0x00, DS1307_ADDR_SEC);

    /* ==================== 5. 讀取秒數暫存器位址的資料 ==================== */
    uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

    /* ==================== 6. 只關心 CH 是否成功被設定成 0 ==================== */
    return (clock_state >> 7) & 0x1;
}


/*
 * @brief  設定 DS1307 當前時間
 * @note   - 轉換為 BCD 格式
 *         - 根據 time_format 設定 12/24 小時制
 * @param  rtc_time: 指向時間結構的指標 (十進位格式)
 * @return None
 */
void ds1307_set_current_time(RTC_time_t *rtc_time)
{
    /* ==================== 1. 設定秒數 ==================== */
    uint8_t seconds = binary_to_bcd(rtc_time->seconds);     /* 將十進制秒數轉換成 BCD 格式 */
    seconds &= ~(1 << 7);   /* 清除 CH bit (Clock Halt) */
    ds1307_write(seconds, DS1307_ADDR_SEC);


    /* ==================== 2. 設定分鐘 ==================== */
    ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);


    /* ==================== 3. 設定小時 ==================== */
    uint8_t hrs;
    hrs = binary_to_bcd(rtc_time->hours);

    if(rtc_time->time_format == TIME_FORMAT_24HRS)
    {
        hrs &= ~(1 << 6);   /* 24 小時制: 清除 bit 6 */
    }
    else
    {
        hrs |= (1 << 6);    /* 12 小時制: 設定 bit 6 */
        
        /* ===== 設定 AM/PM (bit 5): 1 = PM, 0 = AM ===== */
        if (rtc_time->time_format == TIME_FORMAT_12HRS_PM)
        {
            hrs |= (1 << 5);    // PM
        }
        else
        {
            hrs &= ~(1 << 5);   // AM
        }
    }

    ds1307_write(hrs, DS1307_ADDR_HRS);
}


/**
 * @brief  讀取 DS1307 當前時間
 * @note   - ds1307_set_current_time() 只會呼叫一次，在初始化時
 *         - ds1307_get_current_time() 每秒讀取+解析 time_format
 *         硬體自動切換 AM/PM
 * @param  rtc_time: 指向時間結構的指標 
 * @return None
 */
void ds1307_get_current_time(RTC_time_t *rtc_time)
{

    /* ==================== 1. 讀取秒數 ==================== */
    uint8_t seconds = ds1307_read(DS1307_ADDR_SEC);
    seconds &= ~(1 << 7);
    rtc_time->seconds = bcd_to_binary(seconds);


    /* ==================== 2. 讀取分鐘 ==================== */
    rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));


    /* ==================== 3. 讀取小時 ==================== */
    uint8_t hrs;

    hrs = ds1307_read(DS1307_ADDR_HRS);

    if(hrs & (1 << 6)) /* bit 6 = 1 代表為 12 小時制 */
    {
        /* 12 小時制 */
        /* 解析 AM/PM (bit 5) */
        if (hrs & (1 << 5))
        {
            rtc_time->time_format = TIME_FORMAT_12HRS_PM;
        }
        else
        {
            rtc_time->time_format = TIME_FORMAT_12HRS_AM;
        }
        
        hrs &= ~(0x3 << 5); /* 提取小時值: 只保留 bit 0 ~ 4 */
    }
    else /* bit 6 = 0 代表為 24 小時制 */
    {
        /* 24 小時制 */
        rtc_time->time_format = TIME_FORMAT_24HRS;
    }

    rtc_time->hours = bcd_to_binary(hrs);
}


/*
 * @brief  設定 DS1307 當前日期
 * @note   - 轉換為 BCD 格式
 *         - DS1307 會自動處理月底日期和閏年
 *         - 星期不會自動計算，需由使用者提供
 * @param  rtc_date: 指向日期結構的指標 (十進位格式)
 *         - date: 1 ~ 31 (日期)
 *         - month: 1 ~ 12 (月份)
 *         - year: 0 ~ 99 (代表 2000 ~ 2099)
 *         - day: 1 ~ 7 (星期，1 = Sunday, 7 = Saturday)
 * @retval None
 */
void ds1307_set_current_date(RTC_date_t *rtc_date)
{
    /* ==================== 設定日期 (1-31) ==================== */
    ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);

    /* ==================== 設定月份 (1-12) ==================== */
    ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);

    /* ==================== 設定年份 (1-12) ==================== */
    ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);

    /* ==================== 設定星期 (1-7) ==================== */
    ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
}


/*
 * @brief  讀取 DS1307 當前日期
 * @note   - 轉換 BCD 格式為十進位
 *         - 星期 (day) 只有使用到個位數，不需要 BCD 轉換 
 * @param  rtc_date: 指向日期結構的指標 (用於存放讀取結果)
 * @retval None
 */
void ds1307_get_current_date(RTC_date_t *rtc_date)
{
    /* ==================== 讀取星期 (1-7) ==================== */
    rtc_date->day = ds1307_read(DS1307_ADDR_DAY);

    /* ==================== 讀取日期 (1-31) ==================== */
    rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));

    /* ==================== 讀取月份 (1-12) ==================== */
    rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));

    /* ==================== 讀取年份 (00-99) ==================== */
    rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}




/* ==================== Static 內部函式 ==================== */

/*
 * @brief  配置 DS1307 I2C 所需的 GPIO 引腳
 * @note   - I2C 使用 Open-Drain 輸出模式
 *         - PB8 -> I2C1_SCL (AF4)
 *         - PB9 -> I2C1_SDA (AF4)
 * @param  None
 * @return None
 */
static void ds1307_i2c_pin_config(void)
{
    GPIO_Handle_t i2c_sda = {0}, i2c_scl = {0};

    /* ==================== 配置 I2C1_SDA (PB9) ==================== */
    i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
    i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C1_SDA_PIN;
    i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF04;
    i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
    i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    GPIO_Init(&i2c_sda);

    /* ==================== 配置 I2C1_SCL (PB8) ==================== */
    i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
    i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C1_SCL_PIN;
    i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF04;
    i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
    i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    GPIO_Init(&i2c_scl);
}


/*
 * @brief  配置 DS1307 I2C 周邊
 * @note   - DS1307 只有支援 I2C 標準模式 (100kHz)
 * @param  None
 * @return None
 */
static void ds1307_i2c_config(void)
{
    g_ds1307_I2CHandle.pI2Cx = DS1307_I2C;

    /* ==================== 啟用 ACK ==================== */
    g_ds1307_I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;

    /* ==================== 使用標準速度模式 ==================== */
    g_ds1307_I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

    I2C_Init(&g_ds1307_I2CHandle);
}


/*
 * @brief  寫入資料到 DS1307 指定暫存器
 * @note   傳輸格式: [START] [Slave Addr + W] [Reg Addr] [Data] [STOP]
 * @param  value: 要寫入的資料 (BCD 格式)
 * @param  reg_addr: 目標暫存器位址 
 * @return None
 */
static void ds1307_write(uint8_t value, uint8_t reg_addr)
{
    uint8_t tx[2];
    tx[0] = reg_addr;   // 第 1 個 byte: 暫存器位址
    tx[1] = value;      // 第 2 個 byte: 寫入的資料值

    I2C_MasterSendData(&g_ds1307_I2CHandle, tx, 2, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
}


/*
 * @brief  從 DS1307 指定暫存器讀取單一位元組
 * @note   - 使用 Repeated Start 
 * @param  reg_addr: 目標暫存器位址 
 * @return 讀取到的資料值 (BCD 格式)
 */
static uint8_t ds1307_read(uint8_t reg_addr)
{
    uint8_t data;

    /* ==================== 1. 寫入要讀取的暫存器位址 (使用 Repeated Start) ==================== */
    I2C_MasterSendData(&g_ds1307_I2CHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, I2C_ENABLE_SR);

    /* ==================== 2. 讀取資料 (發送 STOP) ==================== */
    I2C_MasterReceiveData(&g_ds1307_I2CHandle, &data, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

    return data;
}


/*
 * @brief  將十進位轉換為 BCD 格式
 * @param  value: 十進位數值 (0 ~ 99)
 * @return BCD 格式的數值
 */
static uint8_t binary_to_bcd(uint8_t value)
{

    /* value / 10 : 取得十位數 */
    /* value % 10 : 取得個位數 */
    /* BCD 格式：高四位為十位數，低四位為個位數 */

    /* 範例：
    *  十進位數值：30 (0001 1110)
    *  十位數：3 (0011) = 30 / 10
    *  個位數：0 (0000) = 30 % 10
    *  BCD 格式： 0011 0000
    */

    return ((value / 10) << 4) | (value % 10);
}


/*
 * @brief  將 BCD 格式轉換為十進位
 * @param  value: BCD 格式的數值
 * @return 十進位數值
 */
static uint8_t bcd_to_binary(uint8_t value)
{
    /* BCD 格式：高四位為十位數，低四位為個位數 */
    /* (value >> 4) * 10: 提取十位數乘上 10 */
    /* (value & 0x0F): 取得個位數 */
    
    /* 範例：
    *  BCD 數值：30 (0011 0000)
    *  十位數：3 * 10 = 30
    *  個位數：0 
    *  BCD 格式： 0011 0000
    */
    uint8_t tens = (uint8_t) (value >> 4) * 10;    /* 提取十位數乘上10變回十進制的值 */
    uint8_t ones = (value & (uint8_t)0x0F);        /* 提取個位數 */

    return tens + ones;
}
