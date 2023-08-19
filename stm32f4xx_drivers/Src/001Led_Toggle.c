/*
 * 001Led_Toggle.c
 *
 *  Created on: 18 Haz 2023
 *      Author: ahmet
 */

#include <stdint.h>
#include "stm32f407xx.h"



int main(void){
	GPIO_Handle_t pinA;
	pinA.pGPIOx=GPIOA;
	pinA.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	pinA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	pinA.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	pinA.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	pinA.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PD;
	GPIO_PeriClockControl(GPIOA, Enable);
	GPIO_Init(&pinA);
	GPIO_Handle_t pinD;
	pinD.pGPIOx=GPIOD;
	pinD.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	pinD.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	pinD.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	pinD.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	pinD.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, Enable);
	GPIO_Init(&pinD);
	uint8_t value=0;
	//GPIO_WriteToOutputPin(GPIOD, 15, 1);
	for(;;){
		value=GPIO_ReadFromInputPin(GPIOA, 0);
		if(value==1){
			GPIO_WriteToOutputPin(GPIOD, 15, 1);
		}else{
			GPIO_WriteToOutputPin(GPIOD, 15, 0);
		}
	}
	return 0;
}
