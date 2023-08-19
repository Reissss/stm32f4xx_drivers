/*
 * 004gpio_interrupt_led.c
 *
 *  Created on: 25 Haz 2023
 *      Author: ahmet
 */

#include <stdint.h>
#include<string.h>
#include "stm32f407xx.h"
void Time_TO_Toggle(void){
	for(uint32_t i=0;i<500000;i++){
	}
}

void EXTI9_5_IRQHandler(void){
	for(uint8_t i=0;i<3;i++){
	GPIO_ToggleOutputPin(GPIOA, 10);
	Time_TO_Toggle();
	}
	GPIO_IRQHandling(GPIO_PIN_NO_5);
}
int main(void){
	GPIO_Handle_t ledA;
	memset(&ledA,0,sizeof(ledA));
	ledA.pGPIOx=GPIOA;
	ledA.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_10;
	ledA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	ledA.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	ledA.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	ledA.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, Enable);
	GPIO_Init(&ledA);
	GPIO_Handle_t bttn_intD;
	memset(&bttn_intD,0,sizeof(bttn_intD));
	bttn_intD.pGPIOx=GPIOD;
	bttn_intD.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	bttn_intD.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	bttn_intD.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	bttn_intD.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	bttn_intD.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;
	GPIO_PeriClockControl(GPIOD, Enable);
	GPIO_Init(&bttn_intD);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, Enable);
	for(;;){
	}
	return 0;
}

