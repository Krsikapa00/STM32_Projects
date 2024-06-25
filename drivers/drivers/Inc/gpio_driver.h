/*
 * gpio_driver.h
 *
 *  Created on: Jun 24, 2024
 *      Author: krsin
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include "stm32f446xx.h"

#define GPIO_PIN_NUM_0 			0
#define GPIO_PIN_NUM_1 			1
#define GPIO_PIN_NUM_2 			2
#define GPIO_PIN_NUM_3 			3
#define GPIO_PIN_NUM_4 			4
#define GPIO_PIN_NUM_5 			5
#define GPIO_PIN_NUM_6 			6
#define GPIO_PIN_NUM_7 			7
#define GPIO_PIN_NUM_8 			8
#define GPIO_PIN_NUM_9 			9
#define GPIO_PIN_NUM_10 		10
#define GPIO_PIN_NUM_11 		11
#define GPIO_PIN_NUM_12 		12
#define GPIO_PIN_NUM_13 		13
#define GPIO_PIN_NUM_14 		14
#define GPIO_PIN_NUM_15 		15

#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFUN		2
#define GPIO_MODE_ANALOG		3

#define GPIO_OTYPE_PUSHPULL		0
#define GPIO_OTYPE_OPENDRAIN	1

#define GPIO_SPEED_FREQ_LOW		0
#define GPIO_SPEED_FREQ_MED		1
#define GPIO_SPEED_FREQ_FAST	2
#define GPIO_SPEED_FREQ_HIGH	3

#define GPIO_PUPD_NOPU			0
#define GPIO_PUPD_PU			1
#define GPIO_PUPD_PD			2

#define GPIOA_MODE_REG_RST	0xA8000000U
#define GPIOB_MODE_REG_RST	0x00000280U
#define GPIOX_MODE_REG_RST	0x00000000U

#define GPIOX_OTYPE_REG_RST	0x00000000U

#define GPIOB_SPEED_REG_RST	0x000000C0U
#define GPIOX_SPEED_REG_RST	0x00000000U

#define GPIOA_PUPD_REG_RST	0x64000000U
#define GPIOB_PUPD_REG_RST	0x00000100U
#define GPIOX_PUPD_REG_RST	0x00000000U

#define GPIOX_IDR_REG_RST	0x00000000U
#define GPIOX_ODR_REG_RST	0x00000000U
#define GPIOX_BSRR_REG_RST	0x00000000U
#define GPIOX_LCK_REG_RST	0x00000000U
#define GPIOX_AFR_REG_RST	0x00000000U


typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t 	*pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Port_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_Pin_DeInit(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDI);

GPIO_PINSTATE GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
uint8_t GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI);
void GPIO_IRQHandling(uint8_t PinNumber);






#endif /* GPIO_DRIVER_H_ */
