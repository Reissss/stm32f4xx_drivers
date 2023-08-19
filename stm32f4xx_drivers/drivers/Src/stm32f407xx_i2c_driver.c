/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 13 Tem 2023
 *      Author: ahmet
 */
#include "stm32f407xx_i2c_driver.h"




//other supportive functions
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseW(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseR(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ClearElementsRx(I2C_Handle_t *pI2CHandle);
static void I2C_ClearElementsTx(I2C_Handle_t *pI2CHandle);





//Peripheral clock setup

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 *
 * @pI2Cx[in]         - base addresses of I2C peripheral
 * @EnorDi[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi==Enable){
		if(pI2Cx==I2C1){I2C1_PCLK_EN();}
		else if(pI2Cx==I2C2){I2C2_PCLK_EN();}
		else if(pI2Cx==I2C3){I2C3_PCLK_EN();}
	}else{
		if(pI2Cx==I2C1){I2C1_PCLK_DI();}
		else if(pI2Cx==I2C2){I2C2_PCLK_DI();}
		else if(pI2Cx==I2C3){I2C3_PCLK_DI();}
	}
}


//INIT AND DE-INIT
/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - This function setting essential bits to register and arranging the I2C
 *
 * @pI2CHandle[in]   - base address of the I2C peripheral and parameters of I2C options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	//START CLOCK
	I2C_PeriClockControl(pI2CHandle->pI2Cx, Enable);
	//Configure ACK
	//uint32_t temp=pI2CHandle->I2C_Config.I2C_ACKControl;
	//pI2CHandle->pI2Cx->CR1|=(temp<<I2C_CR1_ACK);

	//Configure FREQ
	uint32_t temp=0;
	temp|=RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2=(temp&0x3F);

	//program the device own address
	temp=pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
	pI2CHandle->pI2Cx->OAR1|=(1<<14);
	pI2CHandle->pI2Cx->OAR1|=temp;

	//CALCULATE CCR
	uint16_t ccr_value=0;
	if(pI2CHandle->I2C_Config.I2C_FMDutyCycle<=I2C_SCL_SPEED_SM){
		ccr_value=RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		pI2CHandle->pI2Cx->CCR|=(ccr_value&0xfff);
	}else{
		pI2CHandle->pI2Cx->CCR|=(1<<15);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_FM_DUTY_2){
			pI2CHandle->pI2Cx->CCR&=~(1<<14);
			ccr_value=RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed);
			pI2CHandle->pI2Cx->CCR|=(ccr_value&0xfff);
		}else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_FM_DUTY_16_9){
			pI2CHandle->pI2Cx->CCR|=(1<<14);
			ccr_value=RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed);
			pI2CHandle->pI2Cx->CCR|=(ccr_value&0xfff);
		}
	}
	//configure TRISE
	if(pI2CHandle->I2C_Config.I2C_FMDutyCycle<=I2C_SCL_SPEED_SM){
		temp=0;
		temp=(RCC_GetPCLK1Value()/1000000U)+1;
	}else{
		temp=0;
		temp=(RCC_GetPCLK1Value()*3/10000000U)+1;
	}
	pI2CHandle->pI2Cx->TRISE|=(temp<<0);

}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - This function resets the I2C ports
 *
 * @pI2Cx[in]         - base address of the I2C peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}


//Send and Receive DATA

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - This function sends data if device is master
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 * @pTxbuffer[in]     - THE DATA
 * @Len[in]        	  - Data Length
 * @SlaveAddr[in]     - Slave address
 * @Sr[in]        	  - Repeated start attribute
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){
	//Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Wait until communication has started
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG)));

	//Send address of slave and r/nw bit

	I2C_ExecuteAddressPhaseW(pI2CHandle->pI2Cx,SlaveAddr);

	//Confirm the ADDR to confirm the address phase

	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG)));

	//CLEAR ADDR BIT TO PREVENT CLOCK STRETCHING

	I2C_ClearADDRFlag(pI2CHandle);

	//SENDING THE DATA

	while(Len>0){
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG)));
		pI2CHandle->pI2Cx->DR=*pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//WAITING OF SETTED BTF AND TXE FLAGS TO CLOSE THE COMMUNICATION
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG)));

	//Generate Stop Condition
	if(Sr==I2C_DISABLE_SR){
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - This function receives data if device is master
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 * @pTxbuffer[in]     - THE DATA
 * @Len[in]        	  - Data Length
 * @SlaveAddr[in]     - Slave address
 * @Sr[in]        	  - Repeated start attribute
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){
	//Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Wait until communication has started
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG)));

	//Send address of slave and r/nw bit
	I2C_ExecuteAddressPhaseR(pI2CHandle->pI2Cx,SlaveAddr);

	//Confirm the ADDR to confirm the address phase
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG)));

	if(Len==1){
		//GENERATE NACK
		pI2CHandle->pI2Cx->CR1&=~(1<<I2C_CR1_ACK);

		//CLEAR ADDR BIT TO PREVENT CLOCK STRETCHING
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE=1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG)));

		//Generate Stop Condition
		if(Sr==I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Get the data
		*pRxbuffer=pI2CHandle->pI2Cx->DR;
	}

	//Reading data
	if(Len>1){
		//CLEAR ADDR BIT TO PREVENT CLOCK STRETCHING
		I2C_ClearADDRFlag(pI2CHandle);
		while(Len>0){
			//wait until RXNE=1
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG)));

			*pRxbuffer=pI2CHandle->pI2Cx->DR;
			pRxbuffer++;
			if(Len==2){
			//GENERATE NACK
			pI2CHandle->pI2Cx->CR1&=~(1<<I2C_CR1_ACK);
			//Generate Stop Condition
			if(Sr==I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			}
			Len--;
		}
	}

	//Re-enable ACK
	if(pI2CHandle->I2C_Config.I2C_ACKControl==I2C_ACK_ENABLE){pI2CHandle->pI2Cx->CR1|=(1<<I2C_CR1_ACK);}

}

//SEND&RECEIVE DATA IN SLAVE MODE

/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief             - This function sends data if device is slave
 *
 * @pI2Cx[in]    - base address of the I2C peripheral
 * @data[in]     - THE DATA
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data){
	pI2Cx->DR=data;
}

/*********************************************************************
 * @fn      		  - I2C_SlaveReceiveData
 *
 * @brief             - This function receives data if device is slave
 *
 * @pI2Cx[in]    - base address of the I2C peripheral
 * @data[in]     - THE DATA
 *
 * @return            -  data
 *
 * @Note              -  none
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return (uint8_t)pI2Cx->DR;
}

//SEND&RECEIVE DATA WITH IT OPTION
/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - This function sends data if device is master and arranges the required options
 * 						for interrupt
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 * @pTxbuffer[in]     - THE DATA
 * @Len[in]        	  - Data Length
 * @SlaveAddr[in]     - Slave address
 * @Sr[in]        	  - Repeated start attribute
 *
 * @return            -  state
 *
 * @Note              -  none
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}
/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             - This function receives data if device is master and arranges the required options
 * 						for interrupt
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 * @pTxbuffer[in]     - THE DATA
 * @Len[in]        	  - Data Length
 * @SlaveAddr[in]     - Slave address
 * @Sr[in]        	  - Repeated start attribute
 *
 * @return            -  state
 *
 * @Note              -  none
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}



/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             - This function handles events according to the coming event when interrupt happens
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//I2C EVENT IRQ HANDLING
	//FIRST CHECK THE FLAGS
	uint32_t itevnt,itbuf,ev_flags;
	itevnt=pI2CHandle->pI2Cx->CR2&(1<<I2C_CR2_ITEVTEN);
	itbuf=pI2CHandle->pI2Cx->CR2&(1<<I2C_CR2_ITBUFEN);

	//Firstly check the SB(START BIT) flag
	ev_flags=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_SB);

	if(ev_flags&&itevnt){
		//SB FLAG IS SET
		//This section will work on master mode
		//let send the address
		//But we have to be careful to be a write mode or read mode
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseW(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseR(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//Then check the ADDR flag
	ev_flags=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_ADDR);
	if(ev_flags&&itevnt){
		//ADDR FLAG IS SET
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//Then check the BTF flag
	ev_flags=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_BTF);
	if(ev_flags&&itevnt){
		//BTF FLAG IS SET
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			if(pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_TxE)){
				//BTF=1,TXE=1
				if(pI2CHandle->TxLen==0){
					//GENERATE STOP CONDITON
					if(pI2CHandle->Sr==I2C_DISABLE_SR){I2C_GenerateStopCondition(pI2CHandle->pI2Cx);}
					//Clear all the members of struct
					I2C_ClearElementsTx(pI2CHandle);
					//notify application about transmission is completed
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		//DONT HAVE TO CHECK THE RX STATE WHEN BTF FLAG IS SET->
		//CAUSE ACTUALLY WE HAVE NOTHING TO DO ANYTHING
	}

	//Then check the STOPF flag
	//STOPF IS ONLY SET WHEN DEVICE IN SLAVE MODE
	ev_flags=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_STOPF);
	if(ev_flags&&itevnt){
		//STOPF FLAG IS SET
		//STOPF FLAG İS CLEARED BY READING SR1 AND WRITING THE CR1 REGISTER
		//WE HAVE ALLREADY READ THE SR1 REGISTER
		pI2CHandle->pI2Cx->CR1|=0X0000;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP_CMPLT);
	}

	//Then check the RXNE flag
	ev_flags=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_RxNE);
	if(ev_flags&&itevnt&&itbuf){
		//RXNE FLAG IS SET
		if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_MSL)){
			//Device is master
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
				if(pI2CHandle->RxSize==1){
					*(pI2CHandle->pRxBuffer)=pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxSize>1){
					if(pI2CHandle->RxLen==2){
						//GENERATE NACK
						pI2CHandle->pI2Cx->CR1&=~(1<<I2C_CR1_ACK);
					}
					*(pI2CHandle->pRxBuffer)=pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
					pI2CHandle->pRxBuffer++;
				}
				if(pI2CHandle->RxLen==0){
					//GENERATE STOP CONDITON
					if(pI2CHandle->Sr==I2C_DISABLE_SR){I2C_GenerateStopCondition(pI2CHandle->pI2Cx);}
					//Clear all the members of struct
					I2C_ClearElementsRx(pI2CHandle);
					//notify application about transmission is completed
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}else{
				if(!(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_TRA))){
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
				}
			}
		}
	}

	//Then check the TXE flag
	ev_flags=pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_TxE);
	if(ev_flags&&itevnt&&itbuf){
		//TXE FLAG IS SET
		if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
				if(pI2CHandle->TxLen>0){
					pI2CHandle->pI2Cx->DR=*(pI2CHandle->pTxBuffer);
					pI2CHandle->TxLen--;
					pI2CHandle->pTxBuffer++;
				}
			}
		}else{
			if((pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             - This function handles  error events according to the coming error event
 * 						when interrupt happens
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_ARLO );
		//Implement the code to notify the application about the error
	      I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}




/*********************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseW
 *
 * @brief             - sends address of slave and information of writing event
 *
 * @pI2Cx[in]        - base address of the I2C peripheral
 * @SlaveAddr[in]    - Slave address
 *
 * @return            -  none
 *
 * @Note              -  none
 */
static void I2C_ExecuteAddressPhaseW(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr=SlaveAddr<<1;
	SlaveAddr&=~(1);
	pI2Cx->DR=SlaveAddr;
}
/*********************************************************************
 * @fn      		  - I2C_ExecuteAddressPhaseR
 *
 * @brief             - sends address of slave and information of reading event
 *
 * @pI2Cx[in]        - base address of the I2C peripheral
 * @SlaveAddr[in]    - Slave address
 *
 * @return            -  none
 *
 * @Note              -  none
 */
static void I2C_ExecuteAddressPhaseR(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr=SlaveAddr<<1;
	SlaveAddr|=(1);
	pI2Cx->DR=SlaveAddr;
}


/*********************************************************************
 * @fn      		  - I2C_GenerateStartCondition
 *
 * @brief             - generates start condition
 *
 * @pI2Cx[in]        - base address of the I2C peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1|=(1<<I2C_CR1_START);

}

/*********************************************************************
 * @fn      		  - I2C_GenerateStopCondition
 *
 * @brief             - generates stop condition
 *
 * @pI2Cx[in]        - base address of the I2C peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1|=(1<<I2C_CR1_STOP);
}

/*********************************************************************
 * @fn      		  - I2C_ClearADDRFlag
 *
 * @brief             - clear ADDR flag according to events and situations
 *
 * @pI2CHandle[in]        - base address of the I2C peripheral and parameters of I2C options
 *
 * @return            -  none
 *
 * @Note              -  ADDR flags is set when address phase is completed successfully
 * 						FOR MORE INFORMATİON PLEASE REFER TO REFERENCE MANUAL
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummyread=0;
	if(pI2CHandle->pI2Cx->SR2&(1<<I2C_SR2_MSL)){
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize==1){
				//DISABLE ACK
				pI2CHandle->pI2Cx->CR1&=~(1<<I2C_CR1_ACK);
				//CLEAR ADDR
				dummyread=pI2CHandle->pI2Cx->SR1;
				dummyread=pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			}
			else{
				dummyread=pI2CHandle->pI2Cx->SR1;
				dummyread=pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			}
		}else{
			dummyread=pI2CHandle->pI2Cx->SR1;
			dummyread=pI2CHandle->pI2Cx->SR2;
			(void)dummyread;
		}
	}else{
	dummyread=pI2CHandle->pI2Cx->SR1;
	dummyread=pI2CHandle->pI2Cx->SR2;
	(void)dummyread;
	}
}

//ISR AND IRQ
/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
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
 * @fn      		  - SPI_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	uint8_t intpr_number=IRQNumber/4;
	uint8_t intpsection_number=IRQNumber%4;
	uint16_t intregister_shifting=(intpsection_number*8)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR+intpr_number)|=(IRQPriority<<intregister_shifting);
}





//other api's
/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - This function enables PE bits which is peripheral enable
 *
 * @pI2Cx[in]         - base address of the I2C peripheral
 * @EnorDi[in]        -Enable or Disable
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi==Enable){pI2Cx->CR1|=(1<<I2C_CR1_PE);}
	else{pI2Cx->CR1&=~(1<<I2C_CR1_PE);}
	//Configure ACK
	pI2Cx->CR1|=(1<<I2C_CR1_ACK);
}
/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
 *
 * @brief             - This function gets flag status
 *
 * @pI2Cx[in]         - base address of the I2C peripheral
 * @FlagName[in]        -Enable or Disable
 *
 * @return            -  FLAG SITUATION
 *
 * @Note              -  none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName){
	if(pI2Cx->SR1 &FlagName){return FLAG_SET;}
	else{return FLAG_RESET;}
}

/*********************************************************************
 * @fn      		  - I2C_ClearElementsRx
 *
 * @brief             - This function clears interrupt and elements of reception event
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 *
 * @return            -
 *
 * @Note              -  none
 */
static void I2C_ClearElementsRx(I2C_Handle_t *pI2CHandle){
	//Close interrupt
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Close interrupt
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState=I2C_READY;
	pI2CHandle->RxLen=0;
	pI2CHandle->RxSize=0;
	pI2CHandle->pRxBuffer=NULL;

	//Configure ACK
	if(pI2CHandle->I2C_Config.I2C_ACKControl==I2C_ACK_ENABLE){
	pI2CHandle->pI2Cx->CR1|=(1<<I2C_CR1_ACK);
	}

}

/*********************************************************************
 * @fn      		  - I2C_ClearElementsTx
 *
 * @brief             - This function clears interrupt and elements of transmission event
 *
 * @pI2CHandle[in]    - base address of the I2C peripheral and parameters of I2C options
 *
 * @return            -
 *
 * @Note              -  none
 */
static void I2C_ClearElementsTx(I2C_Handle_t *pI2CHandle){
	//Close interrupt
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Close interrupt
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState=I2C_READY;
	pI2CHandle->TxLen=0;
	pI2CHandle->pTxBuffer=NULL;

}

/*********************************************************************
 * @fn      		  - I2C_EnableDisableCallbackEvents
 *
 * @brief             - This function clears interrupts or sets interrupts
 *
 * @pI2Cx[in]    - base address of the I2C peripheral
 * @EnorDi[in]    - Enable or disable
 * @return            -
 *
 * @Note              -  none
 */
void I2C_EnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi==Enable){
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}else if(EnorDi==Disable){
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}

}
