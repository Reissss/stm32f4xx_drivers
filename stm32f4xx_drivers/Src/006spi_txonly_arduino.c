/*
 * 005SPI_sends_data.c
 *
 *  Created on: 3 Tem 2023
 *      Author: ahmet
 */
#include  <stdint.h>
#include  <string.h>
#include "stm32f407xx.h"

/**********
 * PB12->SPI2_NSS
 * PB13->SPI2_SCLK
 * PB14->SPI2_MISO
 * PB15->SPI2_MOSI
 *We are going to use Alternate Functionality 5
**********/

void Time_TO_Toggle(void){
	for(uint32_t i=0;i<500000;i++){}
}

void Initialize_Button(void){
	GPIO_Handle_t buttonA0;
	memset(&buttonA0,0,sizeof(buttonA0));
	buttonA0.pGPIOx=GPIOA;
	buttonA0.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	buttonA0.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	buttonA0.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	buttonA0.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	buttonA0.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_Init(&buttonA0);

}

int main(void){
	//Let's configure the Clock pin
	//PB13 , ALT5
	GPIO_Handle_t spi2_pin;
	memset(&spi2_pin,0,sizeof(spi2_pin));
	spi2_pin.pGPIOx=GPIOB;
	spi2_pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	spi2_pin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	spi2_pin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	spi2_pin.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	spi2_pin.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	spi2_pin.GPIO_PinConfig.GPIO_PinAltFunMode=AF5;
	GPIO_PeriClockControl(GPIOB, Enable);
	GPIO_Init(&spi2_pin);
	//PB15 ,ALT5 MOSI
	spi2_pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&spi2_pin);
	//PB12 ,ALT5 NSS
	spi2_pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&spi2_pin);
	//Let's initialize spi peripherals
	SPI_Handle_t spi2_reg;
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


	//Let's initialize the spi

	SPI_PeripheralControl(SPI2,Enable);

	/*
	 * IF YOU WANT TO PULL TO LOW NSS PIN INTERNALLY AND AUTOMATICALLY
	 * SSOE HAS TO BEEN 1 IN SPI_CR2 REGISTER
	 * You should enable SSOE after the enabling the spi
	 */

	SPI_SSOEControl(SPI2, Enable);

	Initialize_Button();
	//Let's send the data
	char user_data[]="Hello World";

	uint8_t datalen=strlen(user_data);

	for(;;){
		if(GPIO_ReadFromInputPin(GPIOA, 0)==1){
			SPI_PeripheralControl(SPI2,Enable);
			SPI_SendData(SPI2, &datalen, 1);
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
			Time_TO_Toggle();
		}
		if(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)==0){
		SPI_PeripheralControl(SPI2,Disable);
		}

	}

	return 0;

}

