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



	//Let's initialize spi peripherals
	SPI_Handle_t spi2_reg;
	memset(&spi2_reg,0,sizeof(spi2_reg));
	spi2_reg.pSPIx=SPI2;
	spi2_reg.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	spi2_reg.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	spi2_reg.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	spi2_reg.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;
	spi2_reg.SPIConfig.SPI_DFF=SPI_DFF_16BITS;
	spi2_reg.SPIConfig.SPI_SSM=SPI_SSW_EN;
	spi2_reg.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;

	SPI_Init(&spi2_reg);

	//lET'S initialize the ssi to prevent to be a slave

	SPI_SSIControl(SPI2, Enable);

	//Let's initialize the spi

	SPI_PeripheralControl(SPI2,Enable);


	//Let's send the data
	char user_data[]="Hello World";

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));


	SPI_PeripheralControl(SPI2,Disable);

	for(;;)
	return 0;
}

