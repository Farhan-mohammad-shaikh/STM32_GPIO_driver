/*
 * stm32G474re_GPIO_driver.c
 *
 *  Created on: Oct 20, 2024
 *      Author: Lenovo
 */
#include "stm32G474re_GPIO_driver.h"

//Clock enable

/* @fn         - Peripheral clock enable or disable
 * @brief      - This function enable or disable peripheral clock for given GPIO port
 *
 * @param[in]  - base address of GPIO peripheral
 * @param[in]  - ENABLE or DISABLE macros
 *
 *
 * @return     - none
 *
 * @note       - none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{

	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}

	}

	else

	{

		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}


	}


}


//GPIO initialize and De_Init

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)

{

	//GPIO pin mode
	uint32_t temp = 0;
	if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
	    pGPIOHandle ->pGPIOx->MODER |= temp;
	}
	else
	{
		//this code will make later.
		if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI ->FTSR1 |= (1 << GPIO_PinConfig.GPIO_PinNumber);

			EXTI ->RTSR1 &= ~(1 << GPIO_PinConfig.GPIO_PinNumber);

		}

		else if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI ->RTSR1 |= (1 << GPIO_PinConfig.GPIO_PinNumber);

	        EXTI ->FTSR1 &= ~(1 << GPIO_PinConfig.GPIO_PinNumber);

		}

		else if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI ->RTSR1 |= (1 << GPIO_PinConfig.GPIO_PinNumber);

		    EXTI ->FTSR1 |= (1 << GPIO_PinConfig.GPIO_PinNumber);

		}




		EXTI ->IMR1 |= (1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	}


	//GPIO pin speed
	temp = 0;
	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle ->pGPIOx->OSPEEDR |= temp;

    //GPIO pull up pull down register
    temp = 0;

    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    //GPIO pin out put type

    temp = 0;

    temp = pGPIOHandle ->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle ->pGPIOx->OTYPER |= temp;




}
void GPIO_DeInint(GPIO_RegDef_t *pGPIOx)
{


}

//GPIO read and write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)

{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}


void GPIO_WriteToOutputpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);


	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);

	}

}
void GPIO_WriteToOutputport(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR == Value;

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);


}



//GPIO IRQ handle

void GPIO_IRQconfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);

