/*
 * 013usart_tx_slavearduino.c
 *
 *  Created on: 13 Ağu 2023
 *      Author: ahmet
 */
#include "stm32f407xx.h"
#include <stdint.h>
#include <stdio.h>

/*
 * PA2 -> TX  ->AF7
 * PA3 -> RX  ->AF7
 * */
USART_Handle_t usrt;
void Time_TO_Wait(void){
	for(uint32_t i=0;i<500000;i++){}
}
char *msg[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};
//reply from arduino will be stored here
char rx_buf[1024] ;
uint8_t rxCmplt = Reset;
uint8_t txCmplt = Reset;
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
void TxAndRxPinsInit(){
	GPIO_Handle_t pins;
	memset(&pins,0,sizeof(pins));
	//TX PIN INIT PA2, AF7,
	pins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_2;
	pins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	pins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	pins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;
	pins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_AF7;
	pins.pGPIOx=GPIOA;
	//TX INIT
	GPIO_Init(&pins);

	//RX INIT
	pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_3;
	GPIO_Init(&pins);

}
void USARTInitiation(){
	memset(&usrt,0,sizeof(usrt));
	usrt.USART_Config.USART_Mode=USART_MODE_TXRX;
	usrt.USART_Config.USART_HWFlowControl=USART_HW_FLOW_CTRL_NONE;
	usrt.USART_Config.USART_NoOfStopBits=USART_STOPBITS_1;
	usrt.USART_Config.USART_ParityControl=USART_PARITY_DISABLE;
	usrt.USART_Config.USART_Baud=USART_STD_BAUD_115200;
	usrt.USART_Config.USART_WordLength=USART_WORDLEN_8BITS;
	usrt.pUSARTx=USART2;
	USART_Init(&usrt);

}
int main(void){
	uint32_t cnt = 0;
	UserButtonInit();
	TxAndRxPinsInit();
	USARTInitiation();
	USART_PeripheralControl(USART2, Enable);
	USART_IRQInterruptConfig(IRQ_NO_USART2, Enable);
	printf("Application is running\n");
	fflush(stdout);
	for(;;){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		cnt=cnt%3;
		Time_TO_Wait();
		while(USART_ReceiveDataIT(&usrt, rx_buf, strlen(msg[cnt]))!=USART_READY);
		while(USART_SendDataIT(&usrt, (uint8_t*)msg[cnt],strlen(msg[cnt]))!=USART_READY );
		while(txCmplt==Reset);
		printf("Transmitted : %s\n",msg[cnt]);
		fflush(stdout);
		while(rxCmplt==Reset);
		rx_buf[strlen(msg[cnt])+1]='\0';
		rxCmplt=Reset;
		txCmplt=Reset;
		printf("Received    : %s\n",rx_buf);
		fflush(stdout);
		cnt++;
	}
	return 0;
}
void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usrt);
}


void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = Set;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   txCmplt=Set;
   }
}
