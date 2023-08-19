/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: 8 Ağu 2023
 *      Author: ahmet
 */


#include "stm32f407xx_usart_driver.h"
//Peripheral clock setup

/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given USART port
 *
 * @pUSARTx[in]         - base addresses of USART peripheral
 * @EnorDi[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx,uint8_t EnorDi){
	if(EnorDi == Enable){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5){
			UART5_PCLK_EN();
		}else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}
	else{
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		}else if(pUSARTx == USART3){
			USART3_PCLK_DI();
		}else if(pUSARTx == UART4){
			UART4_PCLK_DI();
		}else if(pUSARTx == UART5){
			UART5_PCLK_DI();
		}else if(pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}

//THE OTHER API'S
/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - This function gets flag status
 *
 * @pUSARTx[in]         - base address of the USART peripheral
 * @FlagName[in]        -Enable or Disable
 *
 * @return            -  FLAG SITUATION
 *
 * @Note              -  none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint32_t FlagName){
	if(pUSARTx->SR &FlagName){return FLAG_SET;}
	else{return FLAG_RESET;}
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - This function enables CR1 bits which is peripheral enable
 *
 * @pUSARTx[in]         - base address of the USART peripheral
 * @EnorDi[in]        -Enable or Disable
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
	if(EnOrDi==Enable){pUSARTx->CR1|=(1<<USART_CR1_UE);}
	else{pUSARTx->CR1&=~(1<<USART_CR1_UE);}
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Clears flags
 *
 * @pUSARTx[in]    - base address of the USART peripheral
 * @FlagName[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - if you don't know how to clear the USARTS flag please refer to REFERENCE MANUAL
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName){
	uint8_t temp=(pUSARTx->SR>>FlagName)&0X0001;
	(void)temp;
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - sets baud rate
 *
 * @pUSARTx[in]       - base address of the USART peripheral
 * @BaudRate[in]       - Bits per second
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - it gets frequency of bus from RCC_GetPCLK2Value function then calculates
 * 						mantissa and fraction values.
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

//INIT DEINIT

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - This function setting essential bits to register and arranging the USART
 *
 * @pUSARTHandle[in]   - base address of the USART peripheral and parameters of USART options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_Init(USART_Handle_t *pUSARTHandle){
	USART_PeriClockControl(pUSARTHandle->pUSARTx, Enable);
	uint32_t temp1=0;
	//configure the USART MODE
	temp1=pUSARTHandle->USART_Config.USART_Mode;
	if(temp1==USART_MODE_TXRX){
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_TE);
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_RE);
	}else if(temp1==USART_MODE_ONLY_RX){
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_RE);
	}else if(temp1==USART_MODE_ONLY_TX){
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_TE);
	}

	//Configure the USART baud rate
	temp1=0;
	temp1=pUSARTHandle->USART_Config.USART_Baud;
	USART_SetBaudRate(pUSARTHandle->pUSARTx, temp1);

	//Configure the parity bit
	temp1=0;
	temp1=pUSARTHandle->USART_Config.USART_ParityControl;
	if(temp1==USART_PARITY_DISABLE){
		pUSARTHandle->pUSARTx->CR1&=~(1<<USART_CR1_PCE);
	}else if(temp1==USART_PARITY_EN_EVEN||temp1==USART_PARITY_EN_ODD){
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_PCE);
		if(temp1==USART_PARITY_EN_EVEN) pUSARTHandle->pUSARTx->CR1&=~(1<<USART_CR1_PS);
		else pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_PS);
	}

	//Configure the USART the word length
	temp1=0;
	temp1=pUSARTHandle->USART_Config.USART_WordLength;
	if(temp1==USART_WORDLEN_9BITS) pUSARTHandle->pUSARTx->CR1 |=(1<<USART_CR1_M);
	else pUSARTHandle->pUSARTx->CR1 &=~(1<<USART_CR1_M);

	//Configure the USART NoOfStopBits
	temp1=0;
	temp1=pUSARTHandle->USART_Config.USART_NoOfStopBits;
	pUSARTHandle->pUSARTx->CR2&=~(3<<USART_CR2_STOP);
	pUSARTHandle->pUSARTx->CR2|=(temp1<<USART_CR2_STOP);

	//Configure the USART HWFlowControl
	temp1=0;
	temp1=pUSARTHandle->USART_Config.USART_HWFlowControl;
	if(temp1>0&&temp1<4){
		if(temp1!=1){
			//enable RTS
			pUSARTHandle->pUSARTx->CR3|=(1<<USART_CR3_RTSE);
		}else if(temp1!=2){
			//enable CTS
			pUSARTHandle->pUSARTx->CR3|=(1<<USART_CR3_CTSE);
		}
	}else{
		pUSARTHandle->pUSARTx->CR3&=~(1<<USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->CR3&=~(1<<USART_CR3_CTSE);
	}


}

/*********************************************************************
 * @fn      		  - USART_DeInit
 *
 * @brief             - This function resets the USART ports
 *
 * @pUSARTx[in]         - base address of the USART peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx){
	if(pUSARTx == USART1){
		USART1_REG_RESET();
	}else if(pUSARTx == USART2){
		USART2_REG_RESET();
	}else if(pUSARTx == USART3){
		USART3_REG_RESET();
	}else if(pUSARTx == UART4){
		UART4_REG_RESET();
	}else if(pUSARTx == UART5){
		UART5_REG_RESET();
	}else if(pUSARTx == USART6){
		USART6_REG_RESET();
	}
}



/*
 * Data Send and Receive
 */
/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - This function sends data
 *
 * @pUSARTHandle[in]   - This function setting essential bits to register and arranging the USART
 * @pTxBuffer[in]     - THE DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_TXE_FLAG));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
				i++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_TC_FLAG));

}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - This function receives data
 *
 * @pUSARTHandle[in]  - This function setting essential bits to register and arranging the USART
 * @pRxBuffer[in]     - Given DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	 //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until RXNE flag is set in the SR
			while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_RXNE_FLAG));

			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//We are going to receive 9bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used. so, all 9bits will be of user data

					//read only first 9 bits. so, mask the DR with 0x01FF
					*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01ff);

					//Now increment the pRxBuffer two times
					pRxBuffer++;
					pRxBuffer++;
				}
				else
				{
					//Parity is used, so, 8bits will be of user data and 1 bit is parity
					 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					 //Increment the pRxBuffer
					 pRxBuffer++;
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data

					//read 8 bits from DR
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

				}

				//increment the pRxBuffer
				pRxBuffer++;
			}
		}

}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - This function sends data with interrupt option
 *
 * @pUSARTHandle[in]    - base address of the USART peripheral and parameters of USART options
 * @pTxBuffer[in]     - Given DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  The function just enables the essential interrupts
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer =pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_TCIE);


	}

	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - This function receives data with interrupt option
 *
 * @pUSARTHandle[in]    - base address of the USART peripheral and parameters of USART options
 * @pTxBuffer[in]     - Given DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  The function just enables the essential interrupts
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer =pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1|=(1<<USART_CR1_RXNEIE);

	}

	return rxstate;
}

/*
 * IRQ Configuration and ISR handling
 */
/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - This function refers the interrupt number to the NVIC register which is in
 * 						ARM-CORTEX-M4 processor.
 *
 * @IRQNumber[in]  	  - IRQ number of interrupt handler.
 * @EnorDi[in]        - Enable or disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi==Enable){
		if(IRQNumber<=31){
			//Program ISER0 registers
			*NVIC_ISER0|=(1<<IRQNumber);
		}else if(IRQNumber>31 && IRQNumber<64){
			//Program ISER1 registers
			*NVIC_ISER1|=(1<<IRQNumber%32);
		}else if(IRQNumber>=64 && IRQNumber<96){
			//Program ISER2 registers
			*NVIC_ISER2|=(1<<IRQNumber%64);
		}

	}else{
		if(IRQNumber<=31){
			//Program ICER0 registers
			*NVIC_ICER0|=(1<<IRQNumber);
		}else if(IRQNumber>31 && IRQNumber<64){
			//Program ICER1 registers
			*NVIC_ICER1|=(1<<IRQNumber%32);
		}else if(IRQNumber>=64 && IRQNumber<96){
			//Program ICER2 registers
			*NVIC_ICER2|=(1<<IRQNumber%64);
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - This function changes the interrupt priority of the interrupt handler from NVIC
 * 						registers which is in ARM-CORTEX-M4 processor.
 *
 * @IRQNumber[in]  	  - IRQ number of interrupt handler.
 * @IRQPriority[in]   - IRQ priority number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t intpr_number=IRQNumber/4;
	uint8_t intpsection_number=IRQNumber%4;
	uint16_t intregister_shifting=(intpsection_number*8)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR+intpr_number)|=(IRQPriority<<intregister_shifting);
}

/*********************************************************************
 * @fn      		  - USART_IRQHandling
 *
 * @brief             - This function handles events according to the coming event when interrupt happens
 *
 * @pUSARTHandle[in]    - base address of the USART peripheral and parameters of USART options
 *
 * @return            -  none
 *
 * @Note              -  Error and event flags handles in there, it is not like i2c interrupts
 * 						For I2C and USART interrupts schematic please refer to REFERENCE MANUAL
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle){
	uint32_t temp1 , temp2, temp3;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1&=~(1<<USART_CR1_TCIE);
				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer=NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen=0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					uint16_t *pdata;
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxLen--;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1&=~(1<<USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen--;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1=pUSARTHandle->pUSARTx->SR & (1<<USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 && temp3)
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR&=~(1<<USART_SR_CTS);


		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1= pUSARTHandle->pUSARTx->SR &=(1<<USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		uint32_t dummyreading=0;
		dummyreading=pUSARTHandle->pUSARTx->SR;
		dummyreading=pUSARTHandle->pUSARTx->DR;
		(void)dummyreading;
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}

	//Check for LBD
	temp1=pUSARTHandle->pUSARTx->SR&(1<<USART_SR_LBD);
	temp2=pUSARTHandle->pUSARTx->CR2&(1<<USART_CR2_LBDIE);
	if(temp1&&temp2){
		//LINE BREAK DETECTION INTERRUPT HAPPEND
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_LBD);
	}

}


