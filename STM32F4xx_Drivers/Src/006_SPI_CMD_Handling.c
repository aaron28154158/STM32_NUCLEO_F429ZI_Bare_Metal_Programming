/*
 * 006_spi_cmd_handling.c
 *
 * 功能：STM32 作為 SPI 主機，透過命令協議控制 Arduino 從機
 *
 * 通訊協議：
 * 1. Master 發送命令碼 (1 byte)
 * 2. Master 發送 dummy byte (0xFF) 產生時脈
 * 3. Master 接收 ACK (0xF5 表示成功)
 * 4. 根據命令發送/接收資料
 *
 * 硬體連接：
 * - STM32 SPI1 (Master) <--> Arduino SPI (Slave)
 * - PA5 (SCK)   -> Arduino SCK
 * - PA6 (MISO)  <- Arduino MISO
 * - PA7 (MOSI)  -> Arduino MOSI
 * - PA4 (NSS)   -> Arduino SS
 * - PC13: 按鈕輸入 (按下執行指令)
 */

#include "stm32f429xx.h"
#include <string.h> // 為了使用 strlen

/* ==================== 指令編碼 (Command Codes) ==================== */
#define COMMAND_LED_CTRL          0x50    /* LED 控制指令 */
#define COMMAND_SENSOR_READ       0x51    /* 感測器讀取指令 */
#define COMMAND_LED_READ          0x52    /* LED 狀態讀取指令 */
#define COMMAND_PRINT             0x53    /* 列印指令 */
#define COMMAND_ID_READ           0x54    /* ID 讀取指令 */

/* ==================== LED 狀態 ==================== */
#define LED_ON     1
#define LED_OFF    0

/* ==================== Arduino 類比腳位 ==================== */
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

/* ==================== Arduino LED 腳位 ==================== */
#define LED_PIN       9

/* ==================== ACK/NACK 定義 ==================== */
#define ACK_BYTE      0xF5    /* 從機確認碼 */
#define NACK_BYTE     0xA5    /* 從機否定碼 */

/* ==================== Dummy Data ==================== */
#define DUMMY_WRITE   0xFF    /* 用於產生時脈以接收資料 */

/* ==================== 函數聲明 ==================== */
static void delay_80ms(void);
static void delay_400ms(void);
static void SPI1_GPIOInits(void);
static void SPI1_Inits(void);
static void GPIO_BUTTON_Init(void);
static uint8_t SPI_VerifyResponse(uint8_t ACK_byte);
static void command_led_control(uint8_t CommandCode);
static void command_sensor_read(uint8_t CommandCode);
static void command_led_read(uint8_t CommandCode);
static void command_cmd_print(uint8_t CommandCode);
static void command_id_read(uint8_t CommandCode);


/*****************************************************************
 * 簡單的軟體延遲
 * 實際延遲時間取決於系統時脈頻率
 *****************************************************************/
static void delay_80ms(void)
{
    for(uint32_t i = 0; i < 100000; i++);
}

static void delay_400ms(void)
{
    for(uint32_t i = 0; i < 500000; i++);
}

/*****************************************************************
 * 配置 SPI1 的 GPIO 引腳為複用功能模式
 * SPI1 引腳配置 (Alternate Function AF5):
 * - PA4: NSS  (Chip Select)
 * - PA5: SCK  (Serial Clock)
 * - PA6: MISO (Master In Slave Out)
 * - PA7: MOSI (Master Out Slave In)
 *****************************************************************/
static void SPI1_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF05;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    /* SCK - 時脈輸出 */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&SPIPins);

    /* MISO - 資料輸入 (從 Slave 接收) */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&SPIPins);

    /* MOSI - 資料輸出 (發送至 Slave) */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&SPIPins);

    /* NSS - 片選訊號 (由 SSOE 硬體控制) */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&SPIPins);
}


/*****************************************************************
 * 初始化 SPI1 為主機模式
 * 配置：
 * - Master mode (主機模式)
 * - Full-Duplex (全雙工)
 * - 8-bit data frame
 * - Clock: PCLK/8
 * - CPOL=0, CPHA=0 (Mode 0)
 * - Hardware NSS management (SSOE enabled)
 *****************************************************************/
static void SPI1_Inits(void)
{
    SPI_Handle_t SPI1handle;

    SPI1handle.pSPIx = SPI1;
    SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;           /* 全雙工 */
    SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;     /* 主機模式 */
    SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;                     /* 8 位元資料 */
    SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;         /* 時脈 = PCLK/8 */
    SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;                     /* 第一個邊緣採樣 */
    SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;                     /* 閒置時脈為低 */
    SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI;                        /* 硬體 NSS 管理 */

    SPI_Init(&SPI1handle);
}


/*****************************************************************
 * 初始化按鈕 GPIO (PC13) 為輸入模式
 * STM32F429ZI Nucleo 板載按鈕連接在 PC13
 * 按下時為高電位，釋放時為低電位
 *****************************************************************/
static void GPIO_BUTTON_Init(void)
{
    GPIO_Handle_t GPIO_BUTTON = {0};

    GPIO_BUTTON.pGPIOx = GPIOC;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_BUTTON);
}

/*****************************************************************
 * 驗證從機的 ACK 回應
 *****************************************************************/
static uint8_t SPI_VerifyResponse(uint8_t ACK_byte)
{
    return (ACK_byte == ACK_BYTE) ? 1 : 0;
}

/*****************************************************************
 * LED 控制命令處理函數
 * 傳輸流程：
 * 1. 發送命令碼 0x50
 * 2. 接收 dummy 清除 RXNE
 * 3. 發送 dummy 0xFF 產生時脈
 * 4. 接收 ACK (0xF5)
 * 5. 若 ACK 正確，發送 LED 引腳號 + 狀態
 *****************************************************************/
static void command_led_control(uint8_t CommandCode)
{
    uint8_t dummy_write = DUMMY_WRITE;
    uint8_t dummy_read;
    uint8_t ACK_byte;
    uint8_t args[2];

    /* ========== 步驟 1: 發送命令碼 ========== */
    SPI_SendData(SPI1, &CommandCode, 1);

    /* ========== 步驟 2: 讀取 dummy 資料 (清除 RXNE 旗標) ========== */
    /* 主機發送時會同步接收資料，需讀取以清除接收緩衝區 */
    SPI_ReceiveData(SPI1, &dummy_read, 1);

    /* ========== 步驟 3: 發送 dummy 資料以產生時脈 ========== */
    /* SPI 只有主機能產生時脈，必須主動發送以接收從機回應 */
    SPI_SendData(SPI1, &dummy_write, 1);

    /* ========== 步驟 4: 接收從機的 ACK 回應 ========== */
    SPI_ReceiveData(SPI1, &ACK_byte, 1);

    /* ========== 步驟 5: 驗證 ACK 並發送參數 ========== */
    if(SPI_VerifyResponse(ACK_byte))
    {
        /* ACK 正確，發送 LED 控制參數：
         * args[0] = LED 引腳編號
         * args[1] = LED 狀態 (ON/OFF) */
        args[0] = LED_PIN;
        args[1] = LED_ON;
        SPI_SendData(SPI1, args, 2);
    }
}


/*****************************************************************
 * 感測器讀取命令處理函數
 * 傳輸流程：
 * 1. 發送命令碼 0x51
 * 2. 接收 ACK
 * 3. 發送類比引腳編號
 * 4. 接收類比值 (0~255)
 *****************************************************************/
static void command_sensor_read(uint8_t CommandCode)
{
    uint8_t dummy_write = DUMMY_WRITE;
    uint8_t dummy_read;
    uint8_t ACK_byte;
    uint8_t analog_read;

    /* ========== 步驟 1: 發送命令碼 ========== */
    SPI_SendData(SPI1, &CommandCode, 1);

    /* ========== 步驟 2: 清除 dummy 資料 ========== */
    SPI_ReceiveData(SPI1, &dummy_read, 1);

    /* ========== 步驟 3: 產生時脈以接收 ACK ========== */
    SPI_SendData(SPI1, &dummy_write, 1);

    /* ========== 步驟 4: 接收 ACK ========== */
    SPI_ReceiveData(SPI1, &ACK_byte, 1);

    /* ========== 步驟 5: 驗證 ACK 並發送引腳編號 ========== */
    if(SPI_VerifyResponse(ACK_byte))
    {
        uint8_t analog_pin = ANALOG_PIN0;
        SPI_SendData(SPI1, &analog_pin, 1);

        /* ========== 步驟 6: 清除發送時接收的 dummy 資料 ========== */
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        delay_80ms();

        /* ========== 步驟 7: 產生時脈以接收類比值 ========== */
        SPI_SendData(SPI1, &dummy_write, 1);

        /* ========== 步驟 8: 接收類比值 ========== */
        SPI_ReceiveData(SPI1, &analog_read, 1);
    }
}


/*****************************************************************
 * LED 狀態讀取命令處理函數
 *****************************************************************/
static void command_led_read(uint8_t CommandCode)
{
    uint8_t dummy_write = DUMMY_WRITE;
    uint8_t dummy_read;
    uint8_t ACK_byte;
    uint8_t led_status;

    /* ========== 步驟 1: 發送命令碼 ========== */
    SPI_SendData(SPI1, &CommandCode, 1);

    /* ========== 步驟 2: 清除 dummy 資料 ========== */
    SPI_ReceiveData(SPI1, &dummy_read, 1);

    /* ========== 步驟 3: 產生時脈以接收 ACK ========== */
    SPI_SendData(SPI1, &dummy_write, 1);

    /* ========== 步驟 4: 接收 ACK ========== */
    SPI_ReceiveData(SPI1, &ACK_byte, 1);

    /* ========== 步驟 5: 驗證 ACK 並發送引腳編號 ========== */
    if(SPI_VerifyResponse(ACK_byte))
    {
        uint8_t arg = LED_PIN;
        SPI_SendData(SPI1, &arg, 1);

        /* ========== 清除 dummy 資料 ========== */
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        delay_80ms();

        /* ========== 產生時脈以接收 ACK ========== */
        SPI_SendData(SPI1, &dummy_write, 1);

        /* ========== 接收狀態 ========== */
        SPI_ReceiveData(SPI1, &led_status, 1);
    }
}


/*****************************************************************
 * 字串列印命令處理函數
 *****************************************************************/
static void command_cmd_print(uint8_t CommandCode)
{
    uint8_t dummy_write = DUMMY_WRITE;
    uint8_t dummy_read;
    uint8_t ACK_byte;
    uint8_t message[] = "Testing SPI...";
    uint8_t messageLen = strlen((char*)message);

    /* ========== 步驟 1: 發送命令碼 ========== */
    SPI_SendData(SPI1, &CommandCode, 1);

    /* ========== 步驟 2: 清除 dummy 資料 ========== */
    SPI_ReceiveData(SPI1, &dummy_read, 1);

    /* ========== 步驟 3: 產生時脈以接收 ACK ========== */
    SPI_SendData(SPI1, &dummy_write, 1);

    /* ========== 步驟 4: 接收 ACK ========== */
    SPI_ReceiveData(SPI1, &ACK_byte, 1);

    /* ========== 步驟 5: 驗證 ACK 並發送資料 ========== */
    if(SPI_VerifyResponse(ACK_byte))
    {
        /* 發送字串長度 */
        SPI_SendData(SPI1, &messageLen, 1);

        /* 清除 dummy 資料 */
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        /* 等待從機準備好 */
        delay_80ms();

        for(int i=0; i < messageLen; i++)
        {
            SPI_SendData(SPI1, &message[i], 1);
            SPI_ReceiveData(SPI1, &dummy_read, 1);
        }
    }
}


/*****************************************************************
 * ID 讀取命令處理函數
 * (修正版：使用迴圈進行單字節交換)
 *****************************************************************/
static void command_id_read(uint8_t CommandCode)
{
    uint8_t dummy_write = DUMMY_WRITE;
    uint8_t dummy_read;
    uint8_t ACK_byte;
    uint8_t id[11]; // 10 bytes ID + 1 null terminator

    /* ========== 步驟 1: 發送命令碼 ========== */
    SPI_SendData(SPI1, &CommandCode, 1);

    /* ========== 步驟 2: 清除 dummy 資料 ========== */
    SPI_ReceiveData(SPI1, &dummy_read, 1);

    /* ========== 步驟 3: 產生時脈以接收 ACK ========== */
    SPI_SendData(SPI1, &dummy_write, 1);

    /* ========== 步驟 4: 接收 ACK ========== */
    SPI_ReceiveData(SPI1, &ACK_byte, 1);

    /* ========== 步驟 5: 驗證 ACK 並讀取 ID ========== */
    if(SPI_VerifyResponse(ACK_byte))
    {
        for(int i = 0; i < 10; i++)
        {
            // 1. 發送 Dummy 以產生時脈，將從機資料推入 MISO
            SPI_SendData(SPI1, &dummy_write, 1);

            // 2. 讀取剛接收到的資料，存入陣列 id[i]
            SPI_ReceiveData(SPI1, &id[i], 1);
        }

        id[10] = '\0'; // 字串結尾
    }
}


/*****************************************************************
 * 主程式入口
 * 流程：
 * 1. 初始化按鈕、SPI GPIO、SPI 外設
 * 2. 啟用硬體 NSS 輸出 (SSOE)
 * 3. 等待按鈕按下，依序執行不同指令
 *****************************************************************/
int main(void)
{
    /* ==================== 初始化階段 ==================== */
    GPIO_BUTTON_Init();          /* 初始化按鈕 (PC13) */
    SPI1_GPIOInits();            /* 配置 SPI1 GPIO 引腳 */
    SPI1_Inits();                /* 初始化 SPI1 外設 */

    /* 啟用 SSOE (Slave Select Output Enable)
     * NSS 會在 SPE=1 時自動拉低，SPE=0 時拉高 */
    SPI_SSOEConfig(SPI1, ENABLE);

    /* ==================== 主迴圈 ==================== */
    while(1)
    {
        /* ========== 測試 1: LED Control ========== */
        /* 等待按鈕按下 */
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
        delay_400ms();

        SPI_PeripheralControl(SPI1, ENABLE);
        command_led_control(COMMAND_LED_CTRL);

        while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));
        SPI_PeripheralControl(SPI1, DISABLE);


        /* ========== 測試 2: Sensor Read ========== */
        /* 等待按鈕按下 */
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
        delay_400ms();

        SPI_PeripheralControl(SPI1, ENABLE);
        command_sensor_read(COMMAND_SENSOR_READ);

        while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));
        SPI_PeripheralControl(SPI1, DISABLE);


        /* ========== 測試 3: LED Read ========== */
        /* 等待按鈕按下 */
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
        delay_400ms();

        SPI_PeripheralControl(SPI1, ENABLE);
        command_led_read(COMMAND_LED_READ);

        while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));
        SPI_PeripheralControl(SPI1, DISABLE);


        /* ========== 測試 4: Print Message ========== */
        /* 等待按鈕按下 */
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
        delay_400ms();

        SPI_PeripheralControl(SPI1, ENABLE);
        command_cmd_print(COMMAND_PRINT);

        while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));
        SPI_PeripheralControl(SPI1, DISABLE);


        /* ========== 測試 5: ID Read (已修正迴圈部分) ========== */
        /* 等待按鈕按下 */
        while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
        delay_400ms();

        SPI_PeripheralControl(SPI1, ENABLE);
        command_id_read(COMMAND_ID_READ);

        while(SPI_GetFlagStatus(SPI1, SPI_FLAG_BSY));
        SPI_PeripheralControl(SPI1, DISABLE);
    }
}
