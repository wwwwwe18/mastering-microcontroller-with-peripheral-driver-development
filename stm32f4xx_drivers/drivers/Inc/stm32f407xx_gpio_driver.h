/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 27, 2024
 *      Author: wangw
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			// Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;	// Possible values from @GPIO_PUPD
	uint8_t GPIO_PinOPType;			// Possible values from @GPIO_PIN_OTYPE
	uint8_t GPIO_PinAltFunMode;		// Possible values from
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t		*pGPIOx;			// Pointer to hold the base address of the GPIO peripheral
	GPIO_PinConfig_t	GPIO_PinConfig;  	// GPIO pin configuration settings
}GPIO_Handle_t;

//******************************************************************************************
/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
//******************************************************************************************
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

//******************************************************************************************
/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
//******************************************************************************************
/*
	GPIO port mode register (GPIOx_MODER)

	MODERy[1:0]: Port x configuration bits (y = 0..15)
	These bits are written by software to configure the I/O direction mode.
	00: Input (reset state)
	01: General purpose output mode
	10: Alternate function mode
	11: Analog mode
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG 	3

// Interrupt
#define GPIO_MODE_IT_FT		4	// Falling trigger
#define GPIO_MODE_IT_RT		5	// Rising trigger
#define GPIO_MODE_IT_RFT	6	// Falling and rising trigger

//******************************************************************************************
/*
 * @GPIO_PIN_OTYPE
 * GPIO pin possible output types
 */
//******************************************************************************************
/*
	GPIO port output type register (GPIOx_OTYPER)

	Bits 15:0 OTy: Port x configuration bits (y = 0..15)
	These bits are written by software to configure the output type of the I/O port.
	0: Output push-pull (reset state)
	1: Output open-drain
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

//******************************************************************************************
/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
//******************************************************************************************
/*
	GPIO port output speed register (GPIOx_OSPEEDR)

	OSPEEDRy[1:0]: Port x configuration bits (y = 0..15)
	These bits are written by software to configure the I/O output speed.
	00: Low speed
	01: Medium speed
	10: High speed
	11: Very high speed
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPOI_SPEED_HIGH		3

//******************************************************************************************
/*
 * @GPIO_PUPD
 * GPIO pin pull-up and pull-down configuration macros
 */
//******************************************************************************************
/*
	GPIO port pull-up/pull-down register (GPIOx_PUPDR)

	PUPDRy[1:0]: Port x configuration bits (y = 0..15)
	These bits are written by software to configure the I/O pull-up or pull-down
	00: No pull-up, pull-down
	01: Pull-up
	10: Pull-down
	11: Reserved
 */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
