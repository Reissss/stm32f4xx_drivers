/*
 * 007_spi_txrx_arduino.c
 *
 *  Created on: 6 Tem 2023
 *      Author: ahmet
 */


/**********
 * PB12->SPI2_NSS
 * PB13->SPI2_SCLK
 * PB14->SPI2_MISO
 * PB15->SPI2_MOSI
 *We are going to use Alternate Functionality 5
**********/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define ACK				(uint8_t)0xF5
#define NACK			(uint8_t)0xA5
uint8_t dumm_write=0XFF;
uint8_t dumm_read=0;

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
	spi_pins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_AF5;

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

void USER_BUTTON_INITIALIZE(void){
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

void SPI2_INITIATOR(void){
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
}

void CMD_LED_CTRL(int value){
	uint8_t command=(uint8_t)0x50;
	SPI_SendData(SPI2,&command,sizeof(command));
	//dummy read
	SPI_ReceiveData(SPI2, &dumm_read, sizeof(dumm_read));
	//dummy_write
	SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
	uint8_t ackornack;
	SPI_ReceiveData(SPI2, &ackornack, sizeof(ackornack));
	if(ackornack==ACK){
		uint8_t pin_no=9;
		uint8_t pin_value=value;
		SPI_SendData(SPI2,&pin_no,sizeof(pin_no));
		SPI_SendData(SPI2,&pin_value,sizeof(pin_value));

	}else{
		printf("\nThe cats flee to the wires!!!!\n");
		fflush(stdout);
	}
	Time_TO_Wait();
}
void CMD_SENSORS_READ(void){
	uint8_t command=(uint8_t)0x51;
	SPI_SendData(SPI2,&command,sizeof(command));
	//dummy read
	SPI_ReceiveData(SPI2, &dumm_read, sizeof(dumm_read));
	//dummy_write
	SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
	uint8_t ackornack;
	SPI_ReceiveData(SPI2, &ackornack, sizeof(ackornack));
	if(ackornack==ACK){
		uint8_t pin_no=0;
		uint8_t data=3;
		SPI_SendData(SPI2,&pin_no,sizeof(pin_no));
		//dummy read
		SPI_ReceiveData(SPI2, &dumm_read, sizeof(dumm_read));
		//arduino needs delay to convert adc
		Time_TO_Wait();
		//dummy_write
		SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
		SPI_ReceiveData(SPI2, &data, sizeof(data));
		printf("THE DATA WHICH WE HAVE RECEIVED = %d",data);
		fflush(stdout);
	}else{
		printf("\nThe cats flee to the wires!!!!\n");
		fflush(stdout);
	}
	Time_TO_Wait();
}

void CMD_LED_READ(void){
	uint8_t command=(uint8_t)0x52;
	SPI_SendData(SPI2,&command,sizeof(command));
	//dummy read
	SPI_ReceiveData(SPI2, &dumm_read, sizeof(dumm_read));
	//dummy_write
	SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
	uint8_t ackornack;
	SPI_ReceiveData(SPI2, &ackornack, sizeof(ackornack));
	if(ackornack==ACK){
		uint8_t pin_no=9;
		uint8_t data=3;
		SPI_SendData(SPI2,&pin_no,sizeof(pin_no));
		//dummy read
		SPI_ReceiveData(SPI2, &dumm_read, sizeof(dumm_read));
		//dummy_write
		SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
		SPI_ReceiveData(SPI2, &data, sizeof(data));
		if(data==0){
			printf("\n%d pin is LOW",pin_no);
			fflush(stdout);
		}
		else{
			printf("\n%d pin is HIGH",pin_no);
			fflush(stdout);
		}
	}else{
		printf("\nThe cats flee to the wires!!!!\n");
		fflush(stdout);
	}
	Time_TO_Wait();
}
void CMD_PRINT(void){
	uint8_t command=(uint8_t)0x53;
	SPI_SendData(SPI2,&command,sizeof(command));
	//dummy read
	SPI_ReceiveData(SPI2, &dumm_read, sizeof(dumm_read));
	//dummy_write
	SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
	uint8_t ackornack;
	SPI_ReceiveData(SPI2, &ackornack, sizeof(ackornack));
	if(ackornack==ACK){
		char message[]="Mert cinerin !!!";
		uint8_t len=strlen(message);
		SPI_SendData(SPI2,&len,sizeof(len));
		SPI_SendData(SPI2,(uint8_t*)message,strlen(message));

	}else{
		printf("\nThe cats flee to the wires!!!!\n");
		fflush(stdout);
	}
	Time_TO_Wait();
}

void CMD_ID_READ(void){
	uint8_t command=(uint8_t)0x54;
	SPI_SendData(SPI2,&command,sizeof(command));
	//dummy read
	SPI_ReceiveData(SPI2, &dumm_read, sizeof(dumm_read));
	//dummy_write
	SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
	uint8_t ackornack;
	SPI_ReceiveData(SPI2, &ackornack, sizeof(ackornack));
	if(ackornack==ACK){
		char message[10];
		//dummy_write
		for(uint8_t i=0;i<10;i++){
		//dummy_write
		SPI_SendData(SPI2,&dumm_write,sizeof(dumm_write));
		SPI_ReceiveData(SPI2, &message[i],1);
		}
		message[10]='\0';
		printf("\nBOARD ID = %s",message);
		fflush(stdout);

	}else{
		printf("\nThe cats flee to the wires!!!!\n");
		fflush(stdout);
	}
	Time_TO_Wait();
}
int main(void){
	Pins_Handle(); //HANDLES SPI PINS
	USER_BUTTON_INITIALIZE();//it's activate the user button on the board.
	SPI2_INITIATOR();
	SPI_PeripheralControl(SPI2, Enable);
	SPI_SSOEControl(SPI2, Enable);




	for(;;){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		CMD_LED_CTRL(1);
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		CMD_LED_CTRL(0);
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		CMD_SENSORS_READ();
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		CMD_LED_READ();
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		CMD_PRINT();
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		CMD_ID_READ();
	}
	return 0;
}
