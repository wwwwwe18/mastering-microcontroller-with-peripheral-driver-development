/**
 ******************************************************************************
 * @file           : main.c
 * @author         : May Wang
 * @brief          : Main program body
 ******************************************************************************
 * Description
 * Enable to peripheral clock for ADC1
 ******************************************************************************
 */

#include<stdint.h>

#define RCC_BASE_ADDR		0x40023800UL
#define RCC_APB2_ENR_OFFSET	0x44UL
#define RCC_APB2_ENR_ADDR	(RCC_BASE_ADDR + RCC_APB2_ENR_OFFSET)

#define ADC_BASE_ADDR		0x40012000UL
#define ADC_CR1_REG_OFFSET	0x04UL
#define ADC_CR1_REG_ADDR	(ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)

int main(void)
{
    uint32_t *pRccApb2Enr = (uint32_t*)RCC_APB2_ENR_ADDR;
    uint32_t *pAdcCr1Reg = (uint32_t*)ADC_CR1_REG_ADDR;

    //****************************************
    // 1. Enable to peripheral clock for ADC1
    //****************************************
    *pRccApb2Enr |= (1 << 8);

    // Bit 8 ADC1EN: ADC1 clock enable
    // This bit is set and cleared by software.
    // 0: ADC1 clock disabled
    // 1: ADC1 clock enabled

    //****************************************
    // 2. Modify the ADC CR1 register
    //****************************************
    *pAdcCr1Reg |= (1 << 8);

    // Bit 8 SCAN: Scan mode
    // This bit is set and cleared by software to enable/disable the Scan mode. In Scan mode, the
    // inputs selected through the ADC_SQRx or ADC_JSQRx registers are converted.
    // 0: Scan mode disabled
    // 1: Scan mode enabled

	for(;;);
}
