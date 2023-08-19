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
uint8_t message[32]="Merhaba ben Ozan Akyol!\n";
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
	iic.I2C_Config.I2C_DeviceAddress=0X69;
	iic.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	iic.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;
	iic.pI2Cx=I2C1;
	I2C_Init(&iic);


}
int main(void){
	UserButtonInit();
	SCLAndSDAPinsInit();
	CommunicationInit();
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, Enable);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, Enable);
	I2C_EnableDisableCallbackEvents(I2C1,Enable);
	I2C_PeripheralControl(I2C1, Enable);
	for(;;);

}
void I2C1_EV_IRQHandler(){
	I2C_EV_IRQHandling(&iic);
}
void I2C1_ER_IRQHandler(){
	I2C_ER_IRQHandling(&iic);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv){
	static uint8_t commandcode=0;
	static uint8_t cnt=0;
	if(AppEv==I2C_EV_DATA_REQ){
		if(commandcode==0x51){
			I2C_SlaveSendData(I2C1, strlen((char*)message));
		}else if(commandcode==0x52){
			I2C_SlaveSendData(I2C1, message[cnt]);
			cnt++;
		}
	}
	else if(AppEv==I2C_EV_DATA_RCV){
		commandcode=I2C_SlaveReceiveData(I2C1);
	}
	else if(AppEv==I2C_ERROR_AF){
		//TRANSMISSION IS OVER FOR SLAVE
		commandcode=0xFF;
	    cnt=0;
	}
	else if(AppEv==I2C_EV_STOP_CMPLT){
		//RECEPTION IS OVER FOR SLAVE
	}
}
