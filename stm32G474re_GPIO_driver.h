/*
 * stm32G474re_GPIO_driver.h
 *
 *  Created on: Oct 20, 2024
 *      Author: Lenovo
 */

#ifndef INC_STM32G474RE_GPIO_DRIVER_H_
#define INC_STM32G474RE_GPIO_DRIVER_H_

#include "stm32g474re.h"

//all the configurable item of GPIO pin is here
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;            //Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/* this is the handle structure for GPIO */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;             //pointer hold the GPIO base address
	GPIO_PinConfig_t GPIO_PinConfig;   //this holds  the GPIO pin configuration setting


}GPIO_Handle_t;


// @GPIO_PIN_MODES
//define all the mode of the GPIO port

#define GPIO_MODE_IN       0          //GPIO input mode found in the rm
#define GPIO_MODE_OUT      1          //GPIO output mode found in the rm
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4          //to interrupt mode **falling edge
#define GPIO_MODE_IT_RT    5          //to interrupt mode **rising edge
#define GPIO_MODE_IT_RFT   6          //to interrupt mode **rising edge, falling edge trigger

//GPIO pin output type pushpull and open drain

#define GPIO_OP_TYPE_PP    0
#define GPIO_OP_TYPE_OD    1

//GPIO pin output type speed

#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_HIGH    2
#define GPIO_SPEED_VHIGH   3

//GPIO pin pull up or pull down mode

#define GPIO_NO_PUPD       0
#define GPIO_PIN_PU        1
#define GPIO_PIN_PD        2

// GPIO pin numbers

#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15


/*   ************GPIO driver API creation****************** */

//Clock enable

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

//GPIO initialize and De_Init

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInint(GPIO_RegDef_t *pGPIOx);

//GPIO read and write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputport(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//GPIO IRQ handle

void GPIO_IRQconfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32G474RE_GPIO_DRIVER_H_ */
