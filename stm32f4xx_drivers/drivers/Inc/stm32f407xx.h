/*
 * stm32f407xx.h
 *
 *  Created on: Dec 27, 2024
 *      Author: wangw
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U		// 112 KB
#define SRAM2_BASEADDR		0x20001C00U		// 112 * 1024 -> 0x1C00
#define ROM					0x1FFF0000U
#define SRAM				SRAM1_BASEADDR

/*
 * Base addresses of AHBx and APBx bus peripheral
 */
#define PERIPH_BASEADDR		0x40000000U

#define APB1PERIPH_BASEADDR	PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR	0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR	0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400)


/**********************************peripheral register definition structures **********************************/

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;		// GPIO port mode register
	__vo uint32_t OTYPER;		// GPIO port output type register
	__vo uint32_t OSPEEDR;		// GPIO port output speed register
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			// GPIO port input data register
	__vo uint32_t ODR;			// GPIO port output data register
	__vo uint32_t BSRR;			// GPIO port bit set/reset register
	__vo uint32_t LCKR;			// GPIO port configuration lock register
	__vo uint32_t AFR[2];		// GPIO alternate function low register, GPIO alternate function high register
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
	__vo uint32_t CR;			// RCC clock control register
	__vo uint32_t PLLCFGR;		// RCC PLL configuration register
	__vo uint32_t CFGR;			// RCC clock configuration register
	__vo uint32_t CIR;			// RCC clock interrupt register
	__vo uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;		// RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;		// RCC APB2 peripheral reset register
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;		// RCC AHB3 peripheral clock enable register
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;	// RCC AHB3 peripheral clock enable in low power mode register
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;	// RCC APB2 peripheral clock enable in low power mode register
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;			// RCC Backup domain control register
	__vo uint32_t CSR;			// RCC clock control & status register
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;		// RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register
}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;			// Interrupt mask register
	__vo uint32_t EMR;			// Event mask register
	__vo uint32_t RTSR;			// Rising trigger selection register
	__vo uint32_t FTSR;			// Falling trigger selection register
	__vo uint32_t SWIER;		// Software interrupt event register
	__vo uint32_t PR;			// Pending register
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;		// SYSCFG memory remap register
	__vo uint32_t PMC;			// SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];	// SYSCFG external interrupt configuration register 1 - 4
	__vo uint32_t RESERVED0[2];
	__vo uint32_t CMPCR;		// Compensation cell control register
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;			// SPI control register 1
	__vo uint32_t CR2;			// SPI control register 2
	__vo uint32_t SR;			// SPI status register
	__vo uint32_t DR;			// SPI data register
	__vo uint32_t CRCPR;		// SPI CRC polynomial register
	__vo uint32_t RXCRCR;		// SPI RX CRC register
	__vo uint32_t TXCRCR;		// SPI TX CRC register
	__vo uint32_t I2SCFGR;		// SPI_I2S configuration register
	__vo uint32_t I2SPR;		// SPI_I2S prescaler register
} SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;			// I2C Control register 1
  __vo uint32_t CR2;			// I2C Control register 2
  __vo uint32_t OAR1;			// I2C Own address register 1
  __vo uint32_t OAR2;			// I2C Own address register 2
  __vo uint32_t DR;				// I2C Data register
  __vo uint32_t SR1;			// I2C Status register 1
  __vo uint32_t SR2;			// I2C Status register 2
  __vo uint32_t CCR;			// I2C Clock control register
  __vo uint32_t TRISE;			// I2C TRISE register
  __vo uint32_t FLTR;			// I2C FLTR register
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;			// Status register
	__vo uint32_t DR;			// Data register
	__vo uint32_t BRR;			// Baud rate register
	__vo uint32_t CR1;			// Control register 1
	__vo uint32_t CR2;			// Control register 2
	__vo uint32_t CR3;			// Control register 3
	__vo uint32_t GTPR;			// Guard time and prescaler register
}USART_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6				((USART_RegDef_t*)USART6_BASEADDR)

//******************************************************************************************
/*
 * Clock Enable Macros
 */
//******************************************************************************************
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))	// Set
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
	Bit 0 GPIOAEN: IO port A clock enable
	Set and cleared by software.
	0: IO port A clock disabled
	1: IO port A clock enabled
 */

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
	Bit 21 I2C1EN: I2C1 clock enable
	Set and cleared by software.
	0: I2C1 clock disabled
	1: I2C1 clock enabled
 */

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 12))

/*
	Bit 12 SPI1EN: SPI1 clock enable
	Set and cleared by software.
	0: SPI1 clock disabled
	1: SPI1 clock enabled
 */

#define SPI2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
	Bit 14 SPI2EN: SPI2 clock enable
	Set and cleared by software.
	0: SPI2 clock disabled
	1: SPI2 clock enabled
 */

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |= (1 << 5))

/*
	Bit 4 USART1EN: USART1 clock enable
	Set and cleared by software.
	0: USART1 clock disabled
	1: USART1 clock enabled
 */

#define USART2_PCLK_EN() 	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  	(RCC->APB1ENR |= (1 << 20))

/*
	Bit 17 USART2EN: USART2 clock enable
	Set and cleared by software.
	0: USART2 clock disabled
	1: USART2 clock enabled
 */

//******************************************************************************************
/*
 * Clock Disable Macros
 */
//******************************************************************************************
/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))	// Reset
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))

#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))

//******************************************************************************************
/*
 *  Macros to reset GPIOx peripherals
 */
//******************************************************************************************
/*
	Bit 0 GPIOARST: IO port A reset
	Set and cleared by software.
	0: does not reset IO port A
	1: resets IO port A
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * Some generic macros
 */
#define ENABLE				1
#define DISABLE				0

#define SET					ENABLE
#define RESET				DISABLE

#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#define FLAG_RESET			RESET
#define FLAG_SET			SET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
