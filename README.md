## STM32_NUCLEO_F429ZI_Bare_Metal_Programming

基於 STM32 Nucleo-F429ZI 開發板的裸機系統專案實作練習，內容參考 Udemy-Mastering Microcontroller and Embedded Driver Development 課程，以暫存器層級建構與封裝 GPIO, SPI, I2C, UART 等周邊驅動程式，使用 Arduino 作為測試對接端 (Slave/Master模擬），搭配 USB 邏輯分析儀進行訊號波形解析與除錯。

## 所需工具與環境
- **STM32 Nucleo-F429ZI Development Board**：Cotex M0/M3/M4 皆可以使用測試 
- **Logic Level Converter-Bi-Directional 雙向邏輯電平轉換模組**：用於 STM32(3.3v) 和 Arduino(5v) 之間的電平轉換 
- **Usb 邏輯分析儀 24MHz 8 通道** 
- **DS1307 RTC 實時時鐘模組** 
- **LCD1602 顯示模組** 
- **STM32CubeIDE**：編譯、燒錄與除錯環境 

## SPI 電路接線圖
![SPI_Circuit_3](https://github.com/user-attachments/assets/2dee4b05-c0b1-4777-8404-7983df084f74)
![SPI_Circuit_1](https://github.com/user-attachments/assets/26c9a64a-ab73-4502-8045-f107bf270a96)
![SPI_Circuit_2](https://github.com/user-attachments/assets/093c5742-6f26-4118-b9fe-3d54972cf223)

## I2C 電路接線圖
![I2C_Circuit_3](https://github.com/user-attachments/assets/baf93b4e-e61d-4abf-be86-dbf820935f09)
![I2C_Circuit_2](https://github.com/user-attachments/assets/1325440f-5490-4110-91e6-a8dcb94e5125)
![I2C_Circuit_1](https://github.com/user-attachments/assets/0e84530a-5add-4f59-ae8d-c370b55f9108)

## UART 電路接線圖
![USART_Circuit_3](https://github.com/user-attachments/assets/daf5bd5f-4500-4a68-b5e9-c2e9c13011db)
![Uart](https://github.com/user-attachments/assets/497675f7-e1b2-4897-91ce-78aedea8c743)

## RTC LCD 電路接線圖
![RTC_LCD_Circuit_3](https://github.com/user-attachments/assets/7ae46a1c-7cd2-4801-8943-1501fc7194ff)
![RTC_LCD_Circuit_1](https://github.com/user-attachments/assets/f50d0f4e-b096-48da-a164-36b8a79fdb5d)
![RTC_LCD_Circuit_2](https://github.com/user-attachments/assets/daa96a7d-04c5-4c54-a592-a8ca1523711b)
