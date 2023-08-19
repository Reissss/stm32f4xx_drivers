/*
 * 009i2c_txonly_arduino.c
 *
 *  Created on: 18 Tem 2023
 *      Author: ahmet
 */


#include "stm32f407xx.h"
#include <stdint.h>

/*
 * PB6 -> SCL ->AF4
 * PB9 -> SDA ->AF4
 * */
I2C_Handle_t iic;
void Time_TO_Wait(void){
	for(uint32_t i=0;i<500000;i++){}
}
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
	pins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	pins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_AF4;
	pins.pGPIOx=GPIOB;
	//SCL INIT
	GPIO_Init(&pins);

	//SDA INIT
	pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_9;
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
	char message[]="Merhabalar, ben Recep Baltas";
	UserButtonInit();
	SCLAndSDAPinsInit();
	CommunicationInit();
	I2C_PeripheralControl(I2C1, Enable);
	for(;;){
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		I2C_MasterSendData(&iic, &message, strlen(message), 0x68);
		Time_TO_Wait();
	}
}
