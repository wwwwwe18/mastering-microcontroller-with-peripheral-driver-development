/**
 ******************************************************************************
 * @file           : main.c
 * @author         : May Wang
 * @brief          : Main program body
 ******************************************************************************
 * 006HSE_Measurement
 ******************************************************************************
 */

#include<stdint.h>

#define RCC_BASE_ADDR		0x40023800UL

#define RCC_CR_REG_OFFSET	0x00UL
#define RCC_CFGR_REG_OFFSET	0x08UL

#define RCC_CR_REG_ADDR		(RCC_BASE_ADDR + RCC_CR_REG_OFFSET)
#define RCC_CFGR_REG_ADDR	(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)

#define GPIOA_BASE_ADDR		0x40020000UL

int main(void)
{
    uint32_t *pRCC_CR_REG = (uint32_t*)RCC_CR_REG_ADDR;
    uint32_t *pRCC_CFGR_REG = (uint32_t*)RCC_CFGR_REG_ADDR;

    //******************************************************************
    // 1.Enable the HSE clock using HSEON bit (RCC_CR)
    //******************************************************************
    *pRCC_CR_REG |= (1 << 16);			// Set

	/*
	Bit 16 HSEON: HSE clock enable
	Set and cleared by software.
	Cleared by hardware to stop the HSE oscillator when entering Stop or Standby mode. This
	bit cannot be reset if the HSE oscillator is used directly or indirectly as the system clock.
	0: HSE oscillator OFF
	1: HSE oscillator ON
	*/

    //******************************************************************
    // 2. Wait until HSE clock from the external crystal stabilizes (only if crystal is connected)
    //******************************************************************
    while(!(*pRCC_CR_REG & (1 << 17)));	// Test

    /*
    Bit 17 HSERDY: HSE clock ready flag
    Set by hardware to indicate that the HSE oscillator is stable. After the HSEON bit is cleared,
    HSERDY goes low after 6 HSE oscillator clock cycles.
    0: HSE oscillator not ready
    1: HSE oscillator ready
    */

    //******************************************************************
    // 3. Switch the system clock to HSE (RCC_CFGR)
    //******************************************************************
    *pRCC_CFGR_REG |= (1 << 0);			// Set

    /*
    Bits 1:0 SW: System clock switch
	Set and cleared by software to select the system clock source.
	Set by hardware to force the HSI selection when leaving the Stop or Standby mode or in
	case of failure of the HSE oscillator used directly or indirectly as the system clock.
	00: HSI oscillator selected as system clock
	01: HSE oscillator selected as system clock
	10: PLL selected as system clock
	11: not allowed
	*/

    //******************************************************************
    // 4. Do MCO1 settings to measure it
	//******************************************************************

    //-------------------------------------------------------------------
    // Modified based on code from 005HSI_Measurement
    //-------------------------------------------------------------------

    //******************************************************************
    // 1. Configure the RCC clock configuration register (RCC_CFGR)
    // MCO1 bit fields to select HSE as clock source
    //******************************************************************
    *pRCC_CFGR_REG |= (1 << 22);	// Set

    /*
    Bits 22:21 MCO1: Microcontroller clock output 1
    Set and cleared by software. Clock source selection may generate glitches on MCO1. It is
    highly recommended to configure these bits only after reset before enabling the external
    oscillators and PLL.
    00: HSI clock selected
    01: LSE oscillator selected
    10: HSE oscillator clock selected
    11: PLL clock selected
    */

    // Configure MCO1 prescaler
    // 110: division by 4
    *pRCC_CFGR_REG |= (1 << 25);	// Set
    *pRCC_CFGR_REG |= (1 << 26);	// Set

	/*
	Bits 26:24 MCO1PRE: MCO1 prescaler
	Set and cleared by software to configure the prescaler of the MCO1. Modification of this
	prescaler may generate glitches on MCO1. It is highly recommended to change this
	prescaler only after reset before enabling the external oscillators and the PLL.
	0xx: no division
	100: division by 2
	101: division by 3
	110: division by 4
	111: division by 5
	 */

    //****************************************
    // 2. Configure PA8 to AF0 mode to behave as MCO1 signal
    //****************************************
    // a) Enable the peripheral clock for GPIOA peripheral
	 uint32_t *pRCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCC_AHB1ENR |= (1 << 0);		// Enable GPIOA peripheral clock

	// b) Configure the mode of GPIOA pin 8 as alternate function mode
	uint32_t *pGPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 00);
	*pGPIOA_MODER &= ~(0x3 << 16); // Clear
	*pGPIOA_MODER |= (0x2 << 16);  // Set

	// c) Configure the alternation function register to set the mode 0 for PA8
	uint32_t *pGPIOA_AFRH = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);
	*pGPIOA_AFRH &= ~(0xf << 0);

	for(;;);
}
