/*
 * 010i2c_rxonly_arduino.c
 *
 *  Created on: 2 Ağu 2023
 *      Author: ahmet
 */

#include "stm32f407xx.h"
#include <stdint.h>
#include <stdio.h>

/*
 * PB6 -> SCL ->AF4
 * PB7 -> SDA ->AF4
 * */
I2C_Handle_t iic;
void Time_TO_Wait(void){
	for(uint32_t i=0;i<500000;i++){}
}
uint8_t rxCmlpt=0;
void UserButtonInit(){
	//USER BUTTON init
	GPIO_Handle_t buton;
	memset(&buton,0,sizeof(buton));
	buton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	buton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	buton.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	buton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	buton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	buton.pGPIOx=GPIOA;
	GPIO_Init(&buton);
}
void SCLAndSDAPinsInit(){
	GPIO_Handle_t pins;
	memset(&pins,0,sizeof(pins));
	//SCL PIN INIT PB6, AF4,
	pins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	pins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;//OPEN DRAIN FOR I2C
	pins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	pins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;
	pins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_AF4;
	pins.pGPIOx=GPIOB;
	//SCL INIT
	GPIO_Init(&pins);

	//SDA INIT
	pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIO_Init(&pins);

}
void CommunicationInit(){
	iic.I2C_Config.I2C_ACKControl=I2C_ACK_ENABLE;
	iic.I2C_Config.I2C_DeviceAddress=0X61;
	iic.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	iic.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;
	iic.pI2Cx=I2C1;
	I2C_Init(&iic);


}
int main(void){
	char dummy_byte=0x51;
	uint8_t len;
	UserButtonInit();
	SCLAndSDAPinsInit();
	CommunicationInit();
	I2C_PeripheralControl(I2C1, Enable);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, Enable);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, Enable);
	for(;;){
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		printf("Button is pressed.\n");
		fflush(stdout);
		while(I2C_MasterSendDataIT(&iic, &dummy_byte, 1, 0x68,I2C_ENABLE_SR)!=I2C_READY);
		while(I2C_MasterReceiveDataIT(&iic, &len, 1, 0x68,I2C_ENABLE_SR)!=I2C_READY);
		dummy_byte=0x52;
		while(I2C_MasterSendDataIT(&iic, &dummy_byte, 1, 0x68,I2C_ENABLE_SR)!=I2C_READY);
		char message[len+1];
		rxCmlpt=0;
		while(I2C_MasterReceiveDataIT(&iic, &message, len, 0x68,I2C_DISABLE_SR)!=I2C_READY);
		while(rxCmlpt!=1);
		message[len+1]='\0';
		printf("\nReceived data is %s",message);
		fflush(stdout);
		rxCmlpt=0;
		Time_TO_Wait();
	}
}
void I2C1_EV_IRQHandler(){
	I2C_EV_IRQHandling(&iic);
}
void I2C1_ER_IRQHandler(){
	I2C_ER_IRQHandling(&iic);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv){
	if(AppEv==I2C_EV_TX_CMPLT){
		printf("Transmission is completed.\n");
		fflush(stdout);
	}
	else if(AppEv==I2C_EV_RX_CMPLT){
		printf("Reception is completed.\n");
		fflush(stdout);
		rxCmlpt=1;
	}
	else if(AppEv==I2C_ERROR_AF){
		printf("Acknowledge bit has not come to the device. Check the connections and software.");
		fflush(stdout);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		while(1);
	}
}
