/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 16 Haz 2023
 *      Author: ahmet
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_
#include "stm32f407xx.h"
#include <stdint.h>

typedef struct{
	uint8_t GPIO_PinNumber;//FOR POSSIBLE VALUES @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode; //FOR POSSIBLE VALUES @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;//FOR POSSIBLE VALUES @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;//FOR POSSIBLE VALUES @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;//FOR POSSIBLE VALUES @GPIO_PIN_OPTYPE
	uint8_t GPIO_PinAltFunMode;
}GPIOPinConfig_t;


typedef struct{
	GPIO_RegDef_t* pGPIOx;
	GPIOPinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
@GPIO_PIN_NUMBER
GPIOPIN NUMBERS MACROS
*/

#define GPIO_PIN_NO_0							0
#define GPIO_PIN_NO_1							1
#define GPIO_PIN_NO_2							2
#define GPIO_PIN_NO_3							3
#define GPIO_PIN_NO_4							4
#define GPIO_PIN_NO_5							5
#define GPIO_PIN_NO_6							6
#define GPIO_PIN_NO_7							7
#define GPIO_PIN_NO_8							8
#define GPIO_PIN_NO_9							9
#define GPIO_PIN_NO_10							10
#define GPIO_PIN_NO_11							11
#define GPIO_PIN_NO_12							12
#define GPIO_PIN_NO_13							13
#define GPIO_PIN_NO_14							14
#define GPIO_PIN_NO_15							15



/*
@GPIO_PIN_MODES
GPIOPIN MODE MACROS
*/
#define GPIO_MODE_IN							0
#define GPIO_MODE_OUT							1
#define GPIO_MODE_ALTFN							2
#define GPIO_MODE_ANALOG						3
#define GPIO_MODE_IT_FT							4
#define GPIO_MODE_IT_RT							5
#define GPIO_MODE_IT_RFT						6

/*
@GPIO_PIN_SPEED
GPIOPIN SPEED MACROS
*/
#define GPIO_SPEED_LOW							0
#define GPIO_SPEED_MEDIUM						1
#define GPIO_SPEED_FAST							2
#define GPIO_SPEED_	HIGH						3


/*
@GPIO_PIN_OPTYPE
GPIOPIN OPTYPE MACROS
*/

#define GPIO_OP_TYPE_PP						0
#define GPIO_OP_TYPE_OD						1

/*
@GPIO_PIN_PUPD
GPIOPIN PUPD MACROS
*/
#define GPIO_NO_PUPD							0
#define GPIO_PU									1
#define GPIO_PD									2


//GPIO ALTARNATE FUNCTIONALITY MACROS
#define GPIO_AF0									0
#define GPIO_AF1									1
#define GPIO_AF2									2
#define GPIO_AF3									3
#define GPIO_AF4									4
#define GPIO_AF5									5
#define GPIO_AF6									6
#define GPIO_AF7									7
#define GPIO_AF8									8
#define GPIO_AF9									9
#define GPIO_AF10								10
#define GPIO_AF11								11
#define GPIO_AF12								12
#define GPIO_AF13								13
#define GPIO_AF14								14
#define GPIO_AF15								15




//Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

//INIT AND DE-INIT
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


//Read AND Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

//ISR AND IRQ
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);





#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
