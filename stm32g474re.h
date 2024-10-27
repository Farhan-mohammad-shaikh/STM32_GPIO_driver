/*
 * stm32g474re.h
 *
 *  Created on: Oct 16, 2024
 *      Author: Lenovo
 */

#ifndef INC_STM32G474RE_H_
#define INC_STM32G474RE_H_

#include <stdint.h>

// base address of flash and SRAM memory

#define flash_baseAdd       0x08000000U    /* flash base address */
#define SRAM_baseAdd        0x20000000U    /* SRAM base address */
#define SRAM                SRAM_baseAdd
#define ROM                 0x1FFF0000U    /* System memory base address */


// base address of bus peripheral

#define PERIPH_BASE         0x40000000U   /* peripheral base address */
#define APB1PER_base        PERIPH_BASE   /* APB1 peripheral base address */
#define APB2PER_base        0x40010000U   /* APB2 peripheral base address */
#define AHB1PER_base        0x40020000U   /* AHB1 peripheral base address */
#define AHB2PER_base        0x48000000U   /* AHB2 peripheral base address */

// all the peripheral for AHB1 bus

#define RCC_baseAdd        (AHB1PER_base + 0x1000)


// all the peripheral for AHB2 bus

#define GPIOA_baseAdd      (AHB2PER_base + 0x0000)
#define GPIOB_baseAdd      (AHB2PER_base + 0x0400)
#define GPIOC_baseAdd      (AHB2PER_base + 0x0800)
#define GPIOD_baseAdd      (AHB2PER_base + 0x0C00)
#define GPIOE_baseAdd      (AHB2PER_base + 0x1000)
#define GPIOF_baseAdd      (AHB2PER_base + 0x1400)
#define GPIOG_baseAdd      (AHB2PER_base + 0x1800)

// all the peripheral for APB2 bus

#define SPI1_baseAdd       (APB2PER_base + 0x3000)
#define SPI4_baseAdd       (APB2PER_base + 0x3C00)
#define EXTI_baseAdd       (APB2PER_base + 0x0400)
#define SYSCFG_baseAdd     (APB2PER_base + 0x0000)
#define USART1_baseAdd     (APB2PER_base + 0x3800)

// all the peripheral for APB1 bus

#define SPI2_baseAdd       (APB1PER_base + 0x3800)
#define SPI3_baseAdd       (APB1PER_base + 0x3C00)
#define USART2_baseAdd     (APB1PER_base + 0x4400)
#define USART3_baseAdd     (APB1PER_base + 0x4800)
#define UART4_baseAdd      (APB1PER_base + 0x4C00)
#define UART5_baseAdd      (APB1PER_base + 0x5000)

#define I2C1_baseAdd       (APB1PER_base + 0x5400)
#define I2C2_baseAdd       (APB1PER_base + 0x5800)
#define I2C3_baseAdd       (APB1PER_base + 0x7800)
#define I2C4_baseAdd       (APB1PER_base + 0x8400)

typedef struct{


	volatile uint32_t MEMRMP;
	volatile uint32_t CFGR1;
	volatile uint32_t EXTICR1;
	volatile uint32_t EXTICR2;
	volatile uint32_t EXTICR3;
	volatile uint32_t EXTICR4;
	volatile uint32_t SCSR;
	volatile uint32_t CFGR2;
	volatile uint32_t SWPR;
	volatile uint32_t SKR;

}SYSCFG_RefDef_t;

#define SYSCFG            ((SYSCFG_RefDef_t*)SYSCFG_baseAdd)


typedef struct
{
	volatile uint32_t IMR1;
	volatile uint32_t EMR1;
	volatile uint32_t RTSR1;
	volatile uint32_t FTSR1;
	volatile uint32_t SWIER1;
	volatile uint32_t PR1;
	volatile uint32_t IMR2;
	volatile uint32_t EMR2;
	volatile uint32_t RTSR2;
	volatile uint32_t FTSR2;
	volatile uint32_t SWIER2;
	volatile uint32_t PR2;

}EXTI_RegDef_t;

// c structure for registers in GPIO peripheral
#define EXTI               ((EXTI_RegDef_t*) EXTI_baseAdd)

typedef struct
{
	volatile uint32_t MODER;                         /* Address offset 0x00 */
	volatile uint32_t OTYPER;                        /* Address offset 0x04 */
	volatile uint32_t OSPEEDR;                       /* Address offset 0x08 */
	volatile uint32_t PUPDR;                         /* Address offset 0x0C */
	volatile uint32_t IDR;                           /* Address offset 0x10 */
	volatile uint32_t ODR;                           /* Address offset 0x14 */
	volatile uint32_t BSRR;                          /* Address offset 0x18 */
	volatile uint32_t LCKR;                          /* Address offset 0x1C */
	volatile uint32_t AFRL;                          /* Address offset 0x20 */
	volatile uint32_t AFRH;                          /* Address offset 0x24 */
	volatile uint32_t BRR;                           /* Address offset 0x28 */


}GPIO_RegDef_t;

#define GPIOA          ((GPIO_RegDef_t*) GPIOA_baseAdd)
#define GPIOB          ((GPIO_RegDef_t*) GPIOB_baseAdd)
#define GPIOC          ((GPIO_RegDef_t*) GPIOC_baseAdd)
#define GPIOD          ((GPIO_RegDef_t*) GPIOD_baseAdd)
#define GPIOE          ((GPIO_RegDef_t*) GPIOE_baseAdd)
#define GPIOF          ((GPIO_RegDef_t*) GPIOF_baseAdd)
#define GPIOG          ((GPIO_RegDef_t*) GPIOG_baseAdd)


// c structure for RCC peripheral

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t ICSCR;
	volatile uint32_t CFGR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CIER;
	volatile uint32_t CIFR;
	volatile uint32_t CICR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t APB1RSTR1;
	volatile uint32_t APB1RSTR2;
	volatile uint32_t APB2RSTR;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t APB1ENR1;
	volatile uint32_t APB1ENR2;
	volatile uint32_t APB2ENR;
	volatile uint32_t AHB1SMENR;
	volatile uint32_t AHB2SMENR;
	volatile uint32_t AHB3SMENR;
	volatile uint32_t APB1SMENR1;
	volatile uint32_t APB1SMENR2;
	volatile uint32_t APB2SMENR;
	volatile uint32_t CCIPR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t CRRCR;
	volatile uint32_t CCIPR2;

}RCC_RegDef_t;

#define RCC      ( (RCC_RegDef_t*) RCC_baseAdd )

//GPIO clock enable

#define GPIOA_PCLK_EN()      (RCC -> AHB2ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()      (RCC -> AHB2ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()      (RCC -> AHB2ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()      (RCC -> AHB2ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()      (RCC -> AHB2ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()      (RCC -> AHB2ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()      (RCC -> AHB2ENR |= (1 << 6) )


//GPIO clock disable

#define GPIOA_PCLK_DI()      (RCC -> AHB2ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()      (RCC -> AHB2ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()      (RCC -> AHB2ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()      (RCC -> AHB2ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()      (RCC -> AHB2ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()      (RCC -> AHB2ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()      (RCC -> AHB2ENR &= ~(1 << 6) )

//SPI1 clock enable

#define SPI1_PCLK_EN()      (RCC -> APB2ENR  |= (1 << 12) )
#define SPI4_PCLK_EN()      (RCC -> APB2ENR  |= (1 << 15) )
#define SPI2_PCLK_EN()      (RCC -> APB1ENR1 |= (1 << 14) )
#define SPI3_PCLK_EN()      (RCC -> APB1ENR1 |= (1 << 15) )

//SPI1 clock disable

#define SPI1_PCLK_DI()      (RCC -> APB2ENR  &= ~(1 << 12) )
#define SPI4_PCLK_DI()      (RCC -> APB2ENR  &= ~(1 << 15) )
#define SPI2_PCLK_DI()      (RCC -> APB1ENR1 &= ~(1 << 14) )
#define SPI3_PCLK_DI()      (RCC -> APB1ENR1 &= ~(1 << 15) )

//SYS_CFG register clock enable and disable

#define SYS_CFG_PCLK_EN()   (RCC ->APB2RSTR |= (1 << 0))
#define SYS_CFG_PCLK_DI()   (RCC ->APB2RSTR &= ~(1 << 0))

//Some generic Macros

#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET


#include "stm32G474re_GPIO_driver.h"
#endif /* INC_STM32G474RE_H_ */


