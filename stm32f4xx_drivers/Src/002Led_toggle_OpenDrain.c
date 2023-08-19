/*
 * 002Led_toggle_OpenDrain.c
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
	GPIO_Handle_t pinD;
	pinD.pGPIOx=GPIOD;
	pinD.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	pinD.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	pinD.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	pinD.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	pinD.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;//GPIO_NO_PUPD
	GPIO_PeriClockControl(pinD.pGPIOx, Enable);
	GPIO_Init(&pinD);
	for(;;){
		GPIO_ToggleOutputPin(GPIOD, 12);
		Time_TO_Toggle();
	}
	return 0;
}

