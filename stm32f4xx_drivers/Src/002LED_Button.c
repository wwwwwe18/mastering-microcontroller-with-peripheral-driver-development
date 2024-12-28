/*
 * 002LED_Button.c
 *
 *  Created on: Dec 28, 2024
 *      Author: wangw
 */

#include "stm32f407xx.h"

#define HIGH		1
#define BTN_PRESSED HIGH

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed, GPIOBtn;

	// LED
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_PP;	// Push pull
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLed);

	// Button
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();	// Avoid button de-bouncing issues
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}

	return 0;
}
