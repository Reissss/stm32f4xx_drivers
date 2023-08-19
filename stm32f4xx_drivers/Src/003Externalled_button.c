/*
 * 001Led_Toggle.c
 *
 *  Created on: 18 Haz 2023
 *      Author: ahmet
 */

#include <stdint.h>
#include "stm32f407xx.h"

void Time_TO_Toggle(void){
	for(uint32_t i=0;i<500000;i++){
	}
}


int main(void){
	GPIO_Handle_t pinB;
	pinB.pGPIOx=GPIOB;
	pinB.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	pinB.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	pinB.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	pinB.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	pinB.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOB, Enable);
	GPIO_Init(&pinB);
	GPIO_Handle_t pinALed;
	pinALed.pGPIOx=GPIOA;
	pinALed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_10;
	pinALed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	pinALed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	pinALed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	pinALed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, Enable);
	GPIO_Init(&pinALed);
	uint8_t value=0;
	//GPIO_WriteToOutputPin(GPIOD, 15, 1);
	for(;;){//i used pull down resistor
		value=GPIO_ReadFromInputPin(GPIOB, 12);
		if(value==1){
			GPIO_WriteToOutputPin(GPIOA, 10, 1);
		}else{
			GPIO_WriteToOutputPin(GPIOA, 10, 0);
		}
	}
	return 0;
}
