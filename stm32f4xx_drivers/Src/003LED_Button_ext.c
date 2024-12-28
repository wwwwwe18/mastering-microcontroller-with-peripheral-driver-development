/*
 * 003LED_Button_ext.c
 *
 *  Created on: Dec 28, 2024
 *      Author: wangw
 */

/*
 * Connect external button to PB12 and external LED to PA8.
 * Toggle the LED whenever the external button is pressed.
 */

#include "stm32f407xx.h"

#define HIGH		1
#define LOW			0
#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed, GPIOBtn;

	// LED
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_8;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_PP;	// Push pull
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOLed);

	// Button
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;		// Pull up

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			delay();	// Avoid button de-bouncing issues
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}

	return 0;
}
