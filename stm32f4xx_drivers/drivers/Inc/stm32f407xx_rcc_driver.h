/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 11 Ağu 2023
 *      Author: ahmet
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_
#include "stm32f407xx.h"
#include <stdint.h>

//Calculating APB1 CLCK
uint32_t RCC_GetPCLK1Value(void);
//Calculating APB2 CLCK
uint32_t RCC_GetPCLK2Value(void);
//Calculating PLL CLK
uint32_t RCC_GetPLLValue(void);
#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
