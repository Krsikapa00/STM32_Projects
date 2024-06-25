/*
 * gpio_driver.c
 *
 *  Created on: Jun 24, 2024
 *      Author: krsin
 */

#include "gpio_driver.h"


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0x0;


    temp = pGPIOHandle->pGPIOx->MODER & ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));    //Reset bits for the pin being accessed
    temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
    pGPIOHandle->pGPIOx->MODER = temp;

//    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
//    pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
//    pGPIOHandle->pGPIOx->MODER |= temp;



    // Speed
    temp = pGPIOHandle->pGPIOx->OSPEEDER & ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
    pGPIOHandle->pGPIOx->OSPEEDER = temp;

    // Pull up/Down 
    temp = pGPIOHandle->pGPIOx->PUPDR & ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
    pGPIOHandle->pGPIOx->PUPDR = temp;

    //Output Type
    temp = pGPIOHandle->pGPIOx->OTYPER & ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER = temp;

    // Alt Fun Type
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
    {
        temp = pGPIOHandle->pGPIOx->AFR & ~(0xF << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4));
        temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4);
        pGPIOHandle->pGPIOx->AFR = temp;
    }

}

void GPIO_Port_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA) 
    {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB) 
    {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC) 
    {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD) 
    {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE) 
    {
        GPIOE_REG_RESET();
    } else if (pGPIOx == GPIOF) 
    {
        GPIOF_REG_RESET();
    } else if (pGPIOx == GPIOG) 
    {
        GPIOG_REG_RESET();
    } else if (pGPIOx == GPIOH) 
    {
        GPIOH_REG_RESET();
    }
    
}

void GPIO_Pin_DeInit(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   
    pGPIOx->MODER &= ~(0x3 << (PinNumber * 2));
    pGPIOx->OSPEEDER &= ~(0x3 << (PinNumber * 2));
    pGPIOx->PUPDR &= ~(0x3 << (PinNumber * 2));
    pGPIOx->OTYPER &= ~(0x1 << PinNumber);
    pGPIOx->AFR &= ~(0xF << (PinNumber * 4));
}

void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDI)
{
    if (EnorDI == ENABLE)
    {
        if (pGPIOx == GPIOA) 
        {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) 
        {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) 
        {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) 
        {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) 
        {
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOF) 
        {
            GPIOF_PCLK_EN();
        } else if (pGPIOx == GPIOG) 
        {
            GPIOG_PCLK_EN();
        } else if (pGPIOx == GPIOH) 
        {
            GPIOH_PCLK_EN();
        }
    } else 
    {
        if (pGPIOx == GPIOA) 
        {
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) 
        {
            GPIOB_PCLK_DI();
        } else if (pGPIOx == GPIOC) 
        {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOD) 
        {
            GPIOD_PCLK_DI();
        } else if (pGPIOx == GPIOE) 
        {
            GPIOE_PCLK_DI();
        } else if (pGPIOx == GPIOF) 
        {
            GPIOF_PCLK_DI();
        } else if (pGPIOx == GPIOG) 
        {
            GPIOG_PCLK_DI();
        } else if (pGPIOx == GPIOH) 
        {
            GPIOH_PCLK_DI();
        }
    }
}


GPIO_PINSTATE GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    
    if (((pGPIOx->IDR >> PinNumber) & 0x1) == GPIO_PIN_RESET)
    {
        return GPIO_PIN_RESET;
    }else 
    {
        return GPIO_PIN_SET;
    }
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t temp = 0x0;
    temp |= pGPIOx->IDR;
    return temp;
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == (uint8_t) GPIO_PIN_RESET)
    {
        pGPIOx->BSRR = (uint32_t) (0x1 << (16 + PinNumber));
    } else 
    {
        pGPIOx->BSRR = (uint32_t) (0x1 << (PinNumber));
    }
    
}

/*
Takes the 16 bit value to represent new state the 16 pins will be set to.
Value:  0 1 1 0 1 1 0 1 0 1 1  0  1  1  0  1
Pin:    0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
*/
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    // Setting output pins using the right BSRR registers
    // ===============
    // uint32_t temp = 0x0;
    // // Set the reset values Make all places that are 0 into a 1
    // temp |= (~Value) << 16;
    // pGPIOx->BSRR = temp;
    // temp = 0x0 | (uint32_t) Value;
    // pGPIOx->BSRR = temp;
    // ====================

    // Setting output pins directly with ODR register
    // Is there a beneifit to using the BSRR registers?
    pGPIOx->ODR = Value;
}

uint8_t GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    // Toggling Pins using the BSRR register
    uint32_t temp = pGPIOx->ODR;
    pGPIOx->BSRR = ((temp & (0x1 << PinNumber)) << 16) | (~temp & (0x1 << PinNumber));

    // Toggling the ODR Pin directly
//     pGPIOx->ODR = pGPIOx->ODR ^ (0x1 << PinNumber);

}


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDI)
{
    
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    
}



