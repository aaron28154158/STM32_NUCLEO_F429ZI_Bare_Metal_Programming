#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stddef.h>
#include <string.h>
#include <stdint.h>

#define __vo volatile

/* ================================================================================
*                          ARM Cortex Mx Processor NVIC Registers
*
*  NVIC 暫存器組說明：
*  - ISER: Interrupt Set-Enable Registers (啟用中斷)
*  - ICER: Interrupt Clear-Enable Registers (禁用中斷)
*  - IPR:  Interrupt Priority Registers (設定優先級)
*
*  每個 ISER/ICER 暫存器控制 32 個中斷：
*  - ISER0/ICER0: IRQ 0-31
*  - ISER1/ICER1: IRQ 32-63
*  - ISER2/ICER2: IRQ 64-95
*  - ISER3/ICER3: IRQ 96-127
*  - ...
*  - ISER7/ICER7: IRQ 224-255
*
*  STM32F429ZI 實際使用：
*  - 共有 96 個外部中斷 (IRQ 0-95)
*  - 實際只需要 ISER0-2 和 ICER0-2
*  ================================================================================
*/

/* ==================== NVIC_ISER0 ~ NVIC_ISER7 (Interrupt Set-enable Registers) 的基底位址 ==================== */
#define NVIC_ISER0                      ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1                      ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2                      ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3                      ((__vo uint32_t*)0xE000E10C)

/* ==================== NVIC_ICER0 ~ NVIC_ICER7 (Interrupt Clear-enable Registers) 的基底位址 ==================== */
#define NVIC_ICER0                      ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                      ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                      ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                      ((__vo uint32_t*)0xE000E18C)

/* ==================== (Interrupt Priority Registers) 的基底位址 ==================== */
#define NVIC_IPR_BASEADDR               ((__vo uint32_t*)0xE000E400)

/* ==================== (Interrupt Priority Registers) 的基底位址 ==================== */
#define NO_IPR_BITS_IMPLEMENTED         4

/* ==================== 中斷優先級編號巨集 ==================== */
#define NVIC_IRQ_PRI_15                 15

/* ================================================================================
*                                     NVIC 輔助巨集
*  ================================================================================
*/

/* 計算 ISER/ICER 暫存器索引 */
#define NVIC_REG_INDEX(irq)     ((irq) / 32)

/* 計算在暫存器中的位元位置 */
#define NVIC_BIT_POS(irq)       ((irq) % 32)

/* 計算 IPR 暫存器索引 */
#define NVIC_IPR_INDEX(irq)     ((irq) / 4)

/* 計算在 IPR 中的 byte 位置 */
#define NVIC_IPR_SECTION(irq)   ((irq) % 4)



/* ================================================================================
*
*  ================================================================================
*/

/* ==================== 記憶體 的基底位址 ==================== */
/* (RM0090 Section 2.3 Memory map ) */
#define FLASH_BASEADDR                  0x08000000U     /* 主快閃記憶體 (Main Flash memory) 基底位址 */
#define SRAM1_BASEADDR                  0x20000000U     /* SRAM1 (112 KB) 基底位址 */
#define SRAM2_BASEADDR                  0x2001C000U     /* SRAM2 (16 KB) 基底位址 */
#define SRAM3_BASEADDR                  0x20020000U     /* SRAM3 (64 KB) 基底位址 */
#define SYSTEM_MEMORY_BASEADDR          0x1FFF0000U     /* 系統記憶體 (System memory) 基底位址 */

/* ==================== APBx / AHBx 的基底位址 ==================== */
/* (RM0090 Section 2.3 Memory map, Table 1 ) */
#define PERIPH_BASEADDR                 0x40000000U         /* 周邊 (Peripheral) 基底位址 */
#define APB1PERIPH_BASEADDR             PERIPH_BASEADDR     /* APB1 周邊基底位址 */
#define APB2PERIPH_BASEADDR             0x40010000U         /* APB2 周邊基底位址 */
#define AHB1PERIPH_BASEADDR             0x40020000U         /* AHB1 周邊基底位址 */
#define AHB2PERIPH_BASEADDR             0x50000000U         /* AHB2 周邊基底位址 */
#define AHB3PERIPH_BASEADDR             0xA0000000U         /* AHB3 周邊基底位址 (FMC) */

/* ==================== AHB1 周邊的基底位址 ==================== */
/* (RM0090 Section 2.3 Memory map, Table 1 ) */
#define GPIOA_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0000U)  /* GPIO Port A 基底位址 */
#define GPIOB_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0400U)  /* GPIO Port B 基底位址 */
#define GPIOC_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0800U)  /* GPIO Port C 基底位址 */
#define GPIOD_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0C00U)  /* GPIO Port D 基底位址 */
#define GPIOE_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1000U)  /* GPIO Port E 基底位址 */
#define GPIOF_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1400U)  /* GPIO Port F 基底位址 */
#define GPIOG_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1800U)  /* GPIO Port G 基底位址 */
#define GPIOH_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1C00U)  /* GPIO Port H 基底位址 */
#define GPIOI_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x2000U)  /* GPIO Port I 基底位址 */
#define GPIOJ_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x2400U)  /* GPIO Port J 基底位址 */
#define GPIOK_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x2800U)  /* GPIO Port K 基底位址 */
#define RCC_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x3800U)  /* RCC 基底位址 */

/* ==================== APB1 周邊的基底位址 ==================== */
/* (RM0090 Section 2.3 Memory map, Table 1 ) */
#define I2C1_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5400U)  /* I2C1 基底位址 */
#define I2C2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5800U)  /* I2C2 基底位址 */
#define I2C3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5C00U)  /* I2C3 基底位址 */
#define SPI2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3800U)  /* SPI2 / I2S2 基底位址 */
#define SPI3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3C00U)  /* SPI3 / I2S3 基底位址 */
#define USART2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4400U)  /* USART2 基底位址 */
#define USART3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4800U)  /* USART3 基底位址 */
#define UART4_BASEADDR                  (APB1PERIPH_BASEADDR + 0x4C00U)  /* UART4 基底位址 */
#define UART5_BASEADDR                  (APB1PERIPH_BASEADDR + 0x5000U)  /* UART5 基底位址 */

/* ==================== APB2 周邊的基底位址 ==================== */
/* (RM0090 Section 2.3 Memory map, Table 1 ) */
#define USART1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x1000U)  /* USART1 基底位址 */
#define USART6_BASEADDR                 (APB2PERIPH_BASEADDR + 0x1400U)  /* USART6 基底位址 */
#define SPI1_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3000U)  /* SPI1 基底位址 */
#define SPI4_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3400U)  /* SPI4 基底位址 (修正：原為 SP14) */
#define EXTI_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3C00U)  /* EXTI (External Interrupt) 基底位址 */
#define SYSCFG_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3800U)  /* SYSCFG (System Configuration) 基底位址 */
#define SPI5_BASEADDR                   (APB2PERIPH_BASEADDR + 0x5000U)  /* SPI5 基底位址 */
#define SPI6_BASEADDR                   (APB2PERIPH_BASEADDR + 0x5400U)  /* SPI6 基底位址 */





/* ================================================================================
*                                   周邊暫存器結構體定義
*  ================================================================================
*/

/* ==================== GPIO 結構體定義 ==================== */
/* (RM0090 Section 8.4.11 GPIO register map, Table 40) */
typedef struct
{
    __vo uint32_t MODER;        /* GPIO port mode register (位址偏移: 0x00) */
    __vo uint32_t OTYPER;       /* GPIO port output type register (位址偏移: 0x04) */
    __vo uint32_t OSPEEDR;      /* GPIO port output speed register (位址偏移: 0x08) */
    __vo uint32_t PUPDR;        /* GPIO port pull-up/pull-down register (位址偏移: 0x0C) */
    __vo uint32_t IDR;          /* GPIO port input data register (位址偏移: 0x10) */
    __vo uint32_t ODR;          /* GPIO port output data register (位址偏移: 0x14) */
    __vo uint32_t BSRR;         /* GPIO port bit set/reset register (位址偏移: 0x18) */
    __vo uint32_t LCKR;         /* GPIO port configuration lock register (位址偏移: 0x1C) */
    __vo uint32_t AFR[2];       /* AFR[0] : GPIO alternate function low register (位址偏移: 0x20)
                                 * AFR[1] : GPIO alternate function high register (位址偏移: 0x24) */
}GPIO_RegDef_t;


/* ==================== RCC 結構體定義 ==================== */
/* (RM0090 Section 6.3.26 RCC register map, Table 34) */
typedef struct
{
	__vo uint32_t CR;				/* RCC clock control register (位址偏移: 0x00) */
	__vo uint32_t PLLCFGR;			/* RCC PLL configuration register (位址偏移: 0x04) */
	__vo uint32_t CFGR;				/* RCC clock configuration register (位址偏移: 0x08) */
	__vo uint32_t CIR;				/* RCC clock interrupt register (位址偏移: 0x0C) */
	__vo uint32_t AHB1RSTR;			/* RCC AHB1 peripheral reset register (位址偏移: 0x10) */
	__vo uint32_t AHB2RSTR;			/* RCC AHB2 peripheral reset register (位址偏移: 0x14) */
	__vo uint32_t AHB3RSTR;			/* RCC AHB3 peripheral reset register (位址偏移: 0x18) */
	uint32_t	  RESERVED0;		/* 保留位址 (0x1C) */
	__vo uint32_t APB1RSTR;			/* RCC APB1 peripheral reset register (位址偏移: 0x20) */
	__vo uint32_t APB2RSTR;			/* RCC APB2 peripheral reset register (位址偏移: 0x24) */
	uint32_t	  RESERVED1[2];		/* 保留位址 (0x28 - 0x2C) */
	__vo uint32_t AHB1ENR;			/* RCC AHB1 peripheral clock enable register (位址偏移: 0x30) */
	__vo uint32_t AHB2ENR;			/* RCC AHB2 peripheral clock enable register (位址偏移: 0x34) */
	__vo uint32_t AHB3ENR;			/* RCC AHB3 peripheral clock enable register (位址偏移: 0x38) */
	uint32_t	  RESERVED2;		/* 保留位址 (0x3C) */
	__vo uint32_t APB1ENR;			/* RCC APB1 peripheral clock enable register (位址偏移: 0x40) */
	__vo uint32_t APB2ENR;			/* RCC APB2 peripheral clock enable register (位址偏移: 0x44) */
	uint32_t	  RESERVED3[2];		/* 保留位址 (0x48 - 0x4C) */
	__vo uint32_t AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register (位址偏移: 0x50) */
	__vo uint32_t AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register (位址偏移: 0x54) */
	__vo uint32_t AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register (位址偏移: 0x58) */
	uint32_t	  RESERVED4;		/* 保留位址 (0x5C) */
	__vo uint32_t APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register (位址偏移: 0x60) */
	__vo uint32_t APB2LPENR;		/* RCC APB2 peripheral clock enable in low power mode register (位址偏移: 0x64) */
	uint32_t	  RESERVED5[2];		/* 保留位址 (0x68 - 0x6C) */
	__vo uint32_t BDCR;				/* RCC Backup domain control register (位址偏移: 0x70) */
	__vo uint32_t CSR;				/* RCC clock control & status register (位址偏移: 0x74) */
	uint32_t	  RESERVED6[2];		/* 保留位址 (0x78 - 0x7C) */
	__vo uint32_t SSCGR;			/* RCC spread spectrum clock generation register (位址偏移: 0x80) */
	__vo uint32_t PLLI2SCFGR;		/* RCC PLLI2S configuration register (位址偏移: 0x84) */
	__vo uint32_t PLLSAICFGR;		/* RCC PLLSAI configuration register (位址偏移: 0x88) */
	__vo uint32_t DCKCFGR;			/* RCC Dedicated Clocks Configuration Register (位址偏移: 0x8C) */
}RCC_RegDef_t;


/* ==================== EXTI 結構體定義 ==================== */
/* (RM0090 Section 12.3.7 EXTI register map, Table 64) */
typedef struct
{
	__vo uint32_t IMR;				/* Interrupt mask register (位址偏移: 0x00)*/
	__vo uint32_t EMR;				/* Event mask register (位址偏移: 0x04) */
	__vo uint32_t RTSR;				/* Rising trigger selection register (位址偏移: 0x08) */
	__vo uint32_t FTSR;				/* Falling trigger selection register (位址偏移: 0x0C) */
	__vo uint32_t SWIER;			/* Software interrupt event register (位址偏移: 0x10) */
	__vo uint32_t PR;				/* Pending register (位址偏移: 0x14) */
}EXTI_RegDef_t;


/* ==================== SYSCFG 結構體定義 ==================== */
/* (RM0090 Section 9.3.8 SYSCFG register map, Table 42) */
typedef struct
{
	__vo uint32_t MEMRMP;			/* SYSCFG memory remap register (位址偏移: 0x00) */
	__vo uint32_t PMC;				/* SYSCFG peripheral mode configuration register (位址偏移: 0x04) */
	__vo uint32_t EXTICR[4];		/* SYSCFG external interrupt configuration register 1 (位址偏移: 0x08) */
									/* SYSCFG external interrupt configuration register 2 (位址偏移: 0x0C) */
									/* SYSCFG external interrupt configuration register 3 (位址偏移: 0x10) */
									/* SYSCFG external interrupt configuration register 4 (位址偏移: 0x14) */
	uint32_t RESERVED0[2];			/* 保留位址 (0x18 - 0x1C) */
	__vo uint32_t CMPCR;			/* Compensation cell control register (位址偏移: 0x20) */
}SYSCFG_RegDef_t;


/* ==================== SPI 結構體定義 ==================== */
/* (RM0090 Section 28.5.10 SPI register map, Table 130) */
typedef struct
{
	__vo uint32_t CR1;				/* SPI control register 1 (位址偏移: 0x00) */
	__vo uint32_t CR2;				/* SPI control register 2 (位址偏移: 0x04) */
	__vo uint32_t SR;				/* SPI status register (位址偏移: 0x08) */
	__vo uint32_t DR;				/* SPI data register (位址偏移: 0x0C) */
	__vo uint32_t CRCPR;			/* SPI CRC polynomial register (位址偏移: 0x10) */
	__vo uint32_t RXCRCR;			/* SPI RX CRC register (位址偏移: 0x14) */
	__vo uint32_t TXCRCR; 			/* SPI TX CRC register (位址偏移: 0x18) */
	__vo uint32_t I2SCFGR;			/* SSPI_I2S configuration register (位址偏移: 0x1C) */
	__vo uint32_t IS2PR; 			/* SPI_I2S rescaler register (位址偏移: 0x20) */
}SPI_RegDef_t;


/* ==================== I2C 結構體定義 ==================== */
/* (RM0090 Section 27.6.11 I2C register map, Table 126) */
typedef struct
{
	__vo uint32_t CR1;		/* Control Register 1 */
	__vo uint32_t CR2;		/* Control Register 2 */
	__vo uint32_t OAR1;		/* Own Address Register 1 */
	__vo uint32_t OAR2;		/* Own Address Register 2 */
	__vo uint32_t DR;		/* Data Register */
	__vo uint32_t SR1;		/* Status Register 1 */
	__vo uint32_t SR2;		/* Status Register 2 */
	__vo uint32_t CCR;		/* Clock Control Register */
	__vo uint32_t TRISE;	/* TRISE Register */
	__vo uint32_t FLTR;		/* Filter Register */
}I2C_RegDef_t;


/* ==================== USART 結構體定義 ==================== */
/* (RM0090 Section 30.6.8 USART register map, Table 150) */
typedef struct
{
	__vo uint32_t SR;		/* Status Register */
	__vo uint32_t DR;		/* Data Register */
	__vo uint32_t BRR;		/* Baud Rate Register */
	__vo uint32_t CR1;		/* Control Register 1 */
	__vo uint32_t CR2;		/* Control Register 2 */
	__vo uint32_t CR3;		/* Control Register 3 */
	__vo uint32_t GTPR;		/* Guard Time and Prescaler Register */
}USART_RegDef_t;



/* ==================== GPIO 指標定義，將基底位址轉型為結構體指標 ==================== */
#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ					((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK					((GPIO_RegDef_t*)GPIOK_BASEADDR)

/* ==================== RCC 指標定義，將基底位址轉型為結構體指標 ==================== */
#define RCC                     ((RCC_RegDef_t*)RCC_BASEADDR)

/* ==================== EXTI 指標定義，將基底位址轉型為結構體指標 ==================== */
#define EXTI                    ((EXTI_RegDef_t*)EXTI_BASEADDR)

/* ==================== SYSCFG 指標定義，將基底位址轉型為結構體指標 ==================== */
#define SYSCFG                  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/* ==================== SPI 指標定義，將基底位址轉型為結構體指標 ==================== */
#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)

/* ==================== I2C 指標定義，將基底位址轉型為結構體指標 ==================== */
#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)

/* ==================== USART 指標定義，將基底位址轉型為結構體指標 ==================== */
#define USART1					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2					((USART_RegDef_t*)USART2_BASEADDR)
#define USART3					((USART_RegDef_t*)USART3_BASEADDR)
#define UART4					((USART_RegDef_t*)UART4_BASEADDR)
#define UART5					((USART_RegDef_t*)UART5_BASEADDR)
#define USART6					((USART_RegDef_t*)USART6_BASEADDR)



/* ================================================================================
*                                   周邊時脈控制巨集
*  ================================================================================
*/
/* RM0090 Section 7.3.10 (RCC_AHB1ENR), 7.3.14 (RCC_APB1ENR), 7.3.15 (RCC_APB2ENR) */

/* ==================== 啟用 GPIOx 的時脈 ==================== */
#define GPIOx_PCLK_EN(x)        ( RCC->AHB1ENR |= (1 << x) )
#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()			( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()			( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()			( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()			( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()			( RCC->AHB1ENR |= (1 << 10) )

/* ==================== 啟用 I2Cx 時脈 ==================== */
#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1 << 23) )

/* ==================== 啟用 SPIx 時脈 ==================== */
#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1 << 15))

/* ==================== 啟用 USARTx 時脈 ==================== */
#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		( RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()			( RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			( RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 << 5))

/* ==================== 啟用 SYSCFG 時脈 ==================== */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14))

/* ==================== 禁用 GPIOx 時脈 ==================== */
#define GPIOx_PCLK_DI(x)		( RCC->AHB1ENR &= ~(1 << x) )
#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 10) )

/* ==================== 禁用 I2Cx 時脈 ==================== */
#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 23) )

/* ==================== 禁用 SPIx 時鐘 ==================== */
#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 15))

/* ==================== 禁用 USARTx 時脈 ==================== */
#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 5))

/* ==================== 禁用 SYSCFG 時脈 ==================== */
#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 14))



/* ==================== 重置 GPIO ==================== */
#define GPIOx_REG_RESET(x)      do{ (RCC->AHB1RSTR |= (1 << x)); (RCC->AHB1RSTR &= ~(1 << x)); }while(0)
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)
#define GPIOJ_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9)); }while(0)
#define GPIOK_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10)); }while(0)

/* ==================== 重置 SPI ==================== */
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/* ==================== 重置 I2C ==================== */
#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/* ==================== 重置 USART ==================== */
#define USART1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)




/* ================================================================================
*                                   ARM Cortex-M4 中斷編號
*  ================================================================================
*/

/* ==================== NVIC 優先級 ==================== */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/* ==================== EXTI 中斷編號 (External Interrupt) ==================== */
/* 獨立中斷向量 (EXTI0-4) */
#define IRQ_NO_EXTI0            6       /* EXTI Line 0 interrupt */
#define IRQ_NO_EXTI1            7       /* EXTI Line 1 interrupt */
#define IRQ_NO_EXTI2            8       /* EXTI Line 2 interrupt */
#define IRQ_NO_EXTI3            9       /* EXTI Line 3 interrupt */
#define IRQ_NO_EXTI4            10      /* EXTI Line 4 interrupt */

/* 共享中斷向量 (EXTI5-9) */
#define IRQ_NO_EXTI9_5          23      /* EXTI Line[9:5] interrupts (共享) */

/* 共享中斷向量 (EXTI10-15) */
#define IRQ_NO_EXTI15_10        40      /* EXTI Line[15:10] interrupts (共享) */

/* ==================== SPI 中斷編號 (SPI Interrupt) ==================== */
#define IRQ_NO_SPI1             35
#define IRQ_NO_SPI2             36
#define IRQ_NO_SPI3             51

/* ==================== I2C 中斷編號 (I2C Interrupt) ==================== */
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32

/* ==================== USART 中斷編號 (USART Interrupt) ==================== */
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71



/* ==================== 通用巨集 ==================== */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET

/* ==================== GPIO 位置編碼 ==================== */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 : \
									 (x == GPIOB) ? 1 : \
									 (x == GPIOC) ? 2 : \
									 (x == GPIOD) ? 3 : \
								     (x == GPIOE) ? 4 : \
								     (x == GPIOF) ? 5 : \
								     (x == GPIOG) ? 6 : \
								     (x == GPIOH) ? 7 : \
								     (x == GPIOI) ? 8 : \
								     (x == GPIOJ) ? 9 : \
								     (x == GPIOK) ? 10 : 0)

/* ==================== SPI_CR1 寄存器位定義 ==================== */
#define SPI_CR1_CPHA        0       /* Clock Phase */
#define SPI_CR1_CPOL        1       /* Clock Polarity */
#define SPI_CR1_MSTR        2       /* Master Selection */
#define SPI_CR1_BR          3       /* Baud Rate Control (3-bit field start) */
#define SPI_CR1_SPE         6       /* SPI Enable */
#define SPI_CR1_LSBFIRST    7       /* Frame Format */
#define SPI_CR1_SSI         8       /* Internal Slave Select */
#define SPI_CR1_SSM         9       /* Software Slave Management */
#define SPI_CR1_RXONLY      10      /* Receive Only */
#define SPI_CR1_DFF         11      /* Data Frame Format */
#define SPI_CR1_CRCNEXT     12      /* CRC Transfer Next */
#define SPI_CR1_CRCEN       13      /* Hardware CRC Calculation Enable */
#define SPI_CR1_BIDIOE      14      /* Output Enable in Bidirectional Mode */
#define SPI_CR1_BIDIMODE    15      /* Bidirectional Data Mode Enable */

/* ==================== SPI_CR2 寄存器位定義 ==================== */
#define SPI_CR2_SSOE		2       /* SS Output Enable*/
#define SPI_CR2_ERRIE		5       /* Error Interrupt Enable*/
#define SPI_CR2_RXNEIE		6       /* RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE		7       /* Tx buffer Empty Interrupt Enable */

/* ==================== SPI_SR 寄存器位定義 ==================== */
#define SPI_SR_RXNE			0       /* Receive buffer Not Empty */
#define SPI_SR_TXE			1       /* Transmit buffer Empty */
#define SPI_SR_CHSIDE		2       /* Channel Side */
#define SPI_SR_UDR			3       /* Underrun flag */
#define SPI_SR_CRCERR		4       /* CRC Error flag */
#define SPI_SR_MODF			5       /* Mode Fault */
#define SPI_SR_OVR			6       /* Overrun flag */
#define SPI_SR_BSY			7       /* Busy flag */
#define SPI_SR_FRE			8       /* Frame format Error */

/* ==================== I2C_CR1 寄存器位定義 ==================== */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/* ==================== I2C_CR2 寄存器位定義 ==================== */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVFEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/* ==================== I2C_SR1 寄存器位定義 ==================== */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/* ==================== I2C_SR2 寄存器位定義 ==================== */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDFFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/* ==================== I2C_OAR1 寄存器位定義 ==================== */
#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD71		1
#define I2C_OAR1_ADD98		8
#define I2C_OAR1_ADDMODE	15

/* ==================== I2C_CCR 寄存器位定義 ==================== */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/* ==================== USART_SR 寄存器位定義 ==================== */
#define USART_SR_PE				0		// Parity error
#define USART_SR_FE				1		// Framing error
#define USART_SR_NF				2		// Noise detected flag
#define USART_SR_ORE			3		// Overrun error
#define USART_SR_IDLE			4		// IDLE line detected
#define USART_SR_RXNE			5		// Read data register not empty
#define USART_SR_TC				6		// Transmission complete
#define USART_SR_TXE			7		// Transmit data register empty
#define USART_SR_LBD			8		// LIN break detection flag
#define USART_SR_CTS			9		// CTS flag

/* ==================== USART_CR1 寄存器位定義 ==================== */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/* ==================== USART_CR2 寄存器位定義 ==================== */
#define USART_CR2_ADD			0		// Address of the USART node (4 bits: bit 0-3)
#define USART_CR2_LBDL			5		// LIN break detection length
#define USART_CR2_LBDIE			6		// LIN break detection interrupt enable
#define USART_CR2_LBCL			8		// Last bit clock pulse
#define USART_CR2_CPHA			9		// Clock phase
#define USART_CR2_CPOL			10		// Clock polarity
#define USART_CR2_CLKEN			11		// Clock enable
#define USART_CR2_STOP			12		// STOP bits (2 bits: bit 12-13)
#define USART_CR2_LINEN			14		// LIN mode enable

/* ==================== USART_CR3 寄存器位定義 ==================== */
#define USART_CR3_EIE			0		// Error interrupt enable
#define USART_CR3_IREN			1		// IrDA mode enable
#define USART_CR3_IRLP			2		// IrDA low-power
#define USART_CR3_HDSEL			3		// Half-duplex selection
#define USART_CR3_NACK			4		// Smartcard NACK enable
#define USART_CR3_SCEN			5		// Smartcard mode enable
#define USART_CR3_DMAR			6		// DMA enable receiver
#define USART_CR3_DMAT			7		// DMA enable transmitter
#define USART_CR3_RTSE			8		// RTS enable
#define USART_CR3_CTSE			9		// CTS enable
#define USART_CR3_CTSIE			10		// CTS interrupt enable
#define USART_CR3_ONEBIT		11		// One sample bit method enable






#include "stm32f429xx_gpio_driver.h"
#include "stm32f429xx_spi_driver.h"
#include "stm32f429xx_i2c_driver.h"
#include "stm32f429xx_usart_driver.h"
#include "stm32f429xx_rcc_driver.h"

#endif /* INC_STM32F429XX_H_ */
