/*
 * 008_spi_interrupt.c
 *
 *  Created on: 9 Tem 2023
 *      Author: ahmet
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


uint8_t Message_Came=0;
char message[500];
char dummy_byte=0xff;
char read_data=0;
uint8_t rcvStop=0;
SPI_Handle_t spi2_reg;
void Time_TO_Wait(void){
	for(uint32_t i=0;i<500000;i++){}
}
void Pins_Handle(void){
	GPIO_Handle_t spi_pins;
	memset(&spi_pins,0,sizeof(spi_pins));
	spi_pins.pGPIOx=GPIOB;
	spi_pins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	spi_pins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	spi_pins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	spi_pins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	spi_pins.GPIO_PinConfig.GPIO_PinAltFunMode=AF5;

	//SCLK
	spi_pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&spi_pins);

	//MOSI
	spi_pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&spi_pins);

	//MISO
	spi_pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&spi_pins);

	//NSS
	spi_pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&spi_pins);
}
void SPI2_INITIATOR(void){
	memset(&spi2_reg,0,sizeof(spi2_reg));
	spi2_reg.pSPIx=SPI2;
	spi2_reg.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	spi2_reg.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	spi2_reg.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	spi2_reg.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;
	spi2_reg.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	spi2_reg.SPIConfig.SPI_SSM=SPI_SSW_DI;
	spi2_reg.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI_Init(&spi2_reg);

}
void FT_INITIALIZE(void){
	GPIO_Handle_t buttonD6;
	memset(&buttonD6,0,sizeof(buttonD6));
	buttonD6.pGPIOx=GPIOD;
	buttonD6.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	buttonD6.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	buttonD6.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	buttonD6.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	buttonD6.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_Init(&buttonD6);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, Enable);
}

int main(void){
	Pins_Handle();
	SPI2_INITIATOR();
	FT_INITIALIZE();
	rcvStop=0;
	for(;;){
		while(!Message_Came);
		memset(&message,NULL,sizeof(message));
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, Disable);
		SPI_PeripheralControl(SPI2, Enable);
		SPI_IRQInterruptConfig(IRQ_NO_SPI2,Enable);
		SPI_SSOEControl(SPI2, Enable);
		while(!rcvStop){
		SPI_SendDataIT(&spi2_reg, &dummy_byte, 1);
		SPI_ReceiveDataIT(&spi2_reg, &read_data, 1);
		}
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		SPI_PeripheralControl(SPI2, Disable);
		printf("Received data is %s\n",message);
		fflush(stdout);
		Message_Came=0;
		SPI_PeripheralControl(SPI2, Disable);
		rcvStop=0;
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, Enable);

	}
	return 0;
}
void SPI2_IRQHandler(void){
	SPI_IRQHandling(&spi2_reg);
}
void EXTI9_5_IRQHandler(void){
	Message_Came=1;
	GPIO_IRQHandling(GPIO_PIN_NO_6);
}
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){
	static uint32_t i = 0;//if you use static , variable is going to be permanent
	if(AppEv==SPI_EVENT_RX_CMPLT){
		message[i++]=read_data;
		if(read_data=='\0'||i>500){
			rcvStop=1;
			message[i--]='\0';
			i=0;
		}
	}
}
