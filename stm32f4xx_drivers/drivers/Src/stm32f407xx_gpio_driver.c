/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 27, 2024
 *      Author: wangw
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)
{
	// Enable GPIOx peripheral clock
	if(ENorDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	// Disable GPIOx peripheral clock
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Initialize the given GPIO port and pin
 *
 * @param[in]         - Handle structure for a GPIO pin
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// 1. Configure the mode of GPIO pin

	// Non-interrupt mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// GPIO_PinMode << (2 * GPIO_PinNumber)
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// Clear
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// Set
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	// Interrupt mode
	else
	{

	}

	// 2. Configure the speed
	temp = 0;
	// GPIO_PinSpeed << (2 * GPIO_PinNumber)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// Clear
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// Set
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. Configure the pupd settings
	temp = 0;
	// GPIO_PinPuPdControl << (2 * GPIO_PinNumber)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// Clear
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// Set
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. Configure the optype
	temp = 0;
	// GPIO_PinOPType << GPIO_PinNumber
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// Clear
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// Set
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Configure the alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;	// Choose AFRL or AFRH
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		uint32_t temp = 0;
		// GPIO_PinAltFunMode << (4 * temp2)
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
		// Clear
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		// Set
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI)
{

}

void GPIO_IRQPriorityConfig(void)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
