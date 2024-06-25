/*
 * stm32f446xx.h
 *
 *  Created on: Jun 24, 2024
 *      Author: krsin
 */

#ifndef INC_STM21F446XX_H_
#define INC_STM21F446XX_H_

#include <stdint.h>

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE


#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#if !defined(UNUSED)
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#endif /* UNUSED */

/*
Base Address of Flash and SRAM (From Reference Manual)
*Treat all address values as Unsigned
*/
#define FLASH_BASEADDR  0x08000000U
#define SRAM1_BASEADDR  0x20000000U
#define SRAM2_BASEADDR  0x2001C000U
#define SYSTEM_MEMADDR  0x1FFF0000U //ROM
#define SRAM            SRAM1_BASEADDR
#define ROM             SYSTEM_MEMADDR

// Main bus domain Addresses
#define APB1_BASEADDR   0x40000000U
#define APB2_BASEADDR   0x40010000U
#define AHB1_BASEADDR   0x40020000U
#define AHB2_BASEADDR   0x50000000U
#define AHB3_BASEADDR   0x60000000U

/* AHB1 Peripherals*/
#define GPIOA_BASEADDR  (AHB1_BASEADDR)
#define GPIOB_BASEADDR  (AHB1_BASEADDR + 0x400)
#define GPIOC_BASEADDR  (AHB1_BASEADDR + 0x800)
#define GPIOD_BASEADDR  (AHB1_BASEADDR + 0xC00)
#define GPIOE_BASEADDR  (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR  (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR  (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR  (AHB1_BASEADDR + 0x1C00)

#define CRC_BASEADDR    (AHB1_BASEADDR + 0x3000)
#define RCC_BASEADDR    (AHB1_BASEADDR + 0x3800UL)

/* APB1 Bus Peripherals*/
#define TIM2            (APB1_BASEADDR + 0x0000)
#define TIM3            (APB1_BASEADDR + 0x0400)
#define TIM4            (APB1_BASEADDR + 0x0800)
#define TIM5            (APB1_BASEADDR + 0x0C00)
#define TIM6            (APB1_BASEADDR + 0x1000)
#define TIM7            (APB1_BASEADDR + 0x1400)
#define TIM12           (APB1_BASEADDR + 0x1800)
#define TIM13           (APB1_BASEADDR + 0x1C00)
#define TIM14           (APB1_BASEADDR + 0x2000)

#define I2C1_BASEADDR   (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR   (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR   (APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR   (APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR   (APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR  (APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR  (APB1_BASEADDR + 0x5000)

#define CAN1_BASEADDR   (APB1_BASEADDR + 0x6400)
#define CAN2_BASEADDR   (APB1_BASEADDR + 0x6800)

/* APB2 Bus Peripherals*/
#define TIM1            (APB2_BASEADDR + 0x0000)
#define TIM8            (APB2_BASEADDR + 0x0400)
#define TIM9            (APB2_BASEADDR + 0x4000)
#define TIM10           (APB2_BASEADDR + 0x4400)
#define TIM11           (APB2_BASEADDR + 0x4800)

#define SPI1_BASEADDR   (APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR   (APB2_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR (APB2_BASEADDR + 0x3800)

#define EXTI_BASEADDR   (APB2_BASEADDR + 0x3C00)

#define USART1_BASEADDR (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2_BASEADDR + 0x1400)

//----- REGISTER STRUCTURES -----
/* GPIO Register Structure*/
typedef struct
{
    volatile uint32_t MODER;     //GPIO Port Mode Register          | 0x0
    volatile uint32_t OTYPER;    //GPIO Port Output Type Register   | 0x04
    volatile uint32_t OSPEEDER;  //GPIO Port Output Speed Register  | 0x08
    volatile uint32_t PUPDR;     //GPIO Port Pull-up/down Register  | 0x0C
    volatile uint32_t IDR;       //GPIO Port Input Data Register    | 0x10
    volatile uint32_t ODR;       //GPIO Port Output Data Register   | 0x14
    volatile uint32_t BSRR;      //GPIO Port Bit Set/Reset Register | 0x18
    volatile uint32_t LCKR;      //GPIO Port Config Lock Register   | 0x1C
    volatile uint64_t AFR;      //Alternate Function Low Register  | 0x20
    // volatile uint32_t AFRH;      //Alternate Function High Register | 0x24
} GPIO_RegDef_t;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
}GPIO_PINSTATE;

/* SPI Register Structure*/
typedef struct
{
    volatile uint32_t CR1;       //Control Register 1
    volatile uint32_t CR2;       //Control Register 2
    volatile uint32_t SR;        //Status Register
    volatile uint32_t DR;        //Data Register
    volatile uint32_t CRCPR;     //CRC Polynomial Register
    volatile uint32_t RXCRCR;    //Rx CRC
    volatile uint32_t TXCRCR;    //Tx CRC
    volatile uint32_t I2SCFGR;   //I2S Config Register
    volatile uint32_t I2SPR;     //I2S Prescaler Register
} SPI_RegDef_t;

/* I2C Register Structure*/
typedef struct
{
    volatile uint32_t CR1;       //Control Register 1
    volatile uint32_t CR2;       //Control Register 2
    volatile uint32_t OAR1;      //Own Address 1 Register
    volatile uint32_t OAR2;      //Own Address 2 Register
    volatile uint32_t DR;        //Data Register
    volatile uint32_t SR1;       //Status Register 1
    volatile uint32_t SR2;       //Status Register 2
    volatile uint32_t CCR;       //Clock Control Register
    volatile uint32_t TRISE;     //Rise Time Register
    volatile uint32_t FLTR;      //Filter Register
} I2C_RegDef_t;


/* UART/USART Register Structure*/
typedef struct
{
    volatile uint32_t SR;        //Status Register
    volatile uint32_t DR;        //Data Register
    volatile uint32_t BRR;       //Baud Rate Register
    volatile uint32_t CR1;       //Control Register 1
    volatile uint32_t CR2;       //Control Register 2
    volatile uint32_t CR3;       //Control Register 3
    volatile uint32_t GTPR;      //Guard Time and Prescaler Register
} USART_RegDef_t;

/* RCC Register Structure*/
typedef struct
{
    volatile uint32_t CR;        //Control Register
    volatile uint32_t PLLCFGR;   //PLL Config Register
    volatile uint32_t CFGR;      //Config Register
    volatile uint32_t CIR;       //Clock Interupt Register
    volatile uint32_t AHB1RSTR;  //AHB1 Peripheral Reset Register
    volatile uint32_t AHB2RSTR;  //AHB2 Peripheral Reset Register
    volatile uint32_t AHB3RSTR;  //AHB3 Peripheral Reset Register
    uint32_t		  RESERVED0;
    volatile uint32_t APB1RSTR;  //APB1 Peripheral Reset Register
    volatile uint32_t APB2RSTR;  //APB2 Peripheral Reset Register
    uint32_t		  RESERVED1[2];
    volatile uint32_t AHB1ENR;   //AHB1 Peripheral Enable Register
    volatile uint32_t AHB2ENR;   //AHB2 Peripheral Enable Register
    volatile uint32_t AHB3ENR;   //AHB3 Peripheral Enable Register
    uint32_t		  RESERVED2;
    volatile uint32_t APB1ENR;   //APB1 Peripheral Enable Register
    volatile uint32_t APB2ENR;   //APB2 Peripheral Enable Register
    uint32_t		  RESERVED3[2];
    volatile uint32_t AHB1LPENR; //AHB1 Peripheral Low-Power Enable Register
    volatile uint32_t AHB2LPENR; //AHB2 Peripheral Low-Power Enable Register
    volatile uint32_t AHB3LPENR; //AHB3 Peripheral Low-Power Enable Register
    uint32_t		  RESERVED4;
    volatile uint32_t APB1LPENR; //APB1 Peripheral Low-Power Enable Register
    volatile uint32_t APB2LPENR; //APB2 Peripheral Low-Power Enable Register
    uint32_t		  RESERVED5[2];
    volatile uint32_t BDCR;      //Backup Domain Control Register
    volatile uint32_t CSR;       //Control and Status Register
    uint32_t		  RESERVED6[2];
    volatile uint32_t SSCGR;     //Spread Spectrum Clock Generation Register
    volatile uint32_t PLLI2SCFGR;//PLLI2S Config Register
    volatile uint32_t PLLSAICFGR;//PLL Config Register
    volatile uint32_t DCKCFGR;   //Dedicated Clock Config Register
    volatile uint32_t CKGATENR;  //Clock Gated Enable Register
    volatile uint32_t DCKCFGR2;  //Dedicated Clock Config Register 2
} RCC_RegDef_t;

//----- PERIPHERAL DEFINITIONS -----

/*GPIO PORTS*/
#define GPIOA ((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *) GPIOH_BASEADDR)

/* RCC */
#define RCC ((RCC_RegDef_t *) RCC_BASEADDR)

/*I2C*/
#define I2C1 ((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *) I2C3_BASEADDR)

/*SPI*/
#define SPI1 ((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *) SPI4_BASEADDR)

//----- Clock Enable Macros -----
#define GPIOA_PCLK_EN()     do{	uint32_t tmpreg = 0x00U;\
								(RCC->AHB1ENR |= (1 << 0));\
								tmpreg = READ_BIT(RCC->AHB1ENR, (0x1 << 0));\
								UNUSED(tmpreg); \
								} while (0)

#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |= (1 << 7))

#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()      (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()      (RCC->APB1ENR |= (1 << 23))

#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()      (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()      (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()      (RCC->APB2ENR |= (1 << 13))

#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()    (RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()    (RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()    (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()    (RCC->APB2ENR |= (1 << 5))

#define CAN1_PCLK_EN()      (RCC->APB1ENR |= (1 << 25))
#define CAN2_PCLK_EN()      (RCC->APB1ENR |= (1 << 26))

#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))

//----- Clock Disable Macros -----
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 7))

#define I2C1_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 23))

#define SPI1_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 13))

#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 5))

#define CAN1_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 25))
#define CAN2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 26))

#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14))

//----- RCC Reset Macros -----
#define GPIOA_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()     do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)

#endif

