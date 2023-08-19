/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 2 Tem 2023
 *      Author: ahmet
 */
#include "stm32f407xx_spi_driver.h"

//Peripheral clock setup
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI buses
 *
 * @pSPIx[in]         - base address of the spi peripheral
 * @EnorDi[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);




/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function gets the flag status.
 *
 * @pSPIx[in]  	      - base addresses of SPI peripheral
 * @FlagName[in]      - Flag name which is in the SR register.
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){
	if(pSPIx->SR&FlagName){return FLAG_SET;}
	return FLAG_RESET;
}


/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @pSPIx[in]         - base addresses of SPI peripheral
 * @EnorDi[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == Enable){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}

//INIT AND DE-INIT
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function setting essential bits to register and arranging the SPI
 *
 * @pSPIHandle[in]   - base address of the SPI peripheral and parameters of SPI options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){


	SPI_PeriClockControl(pSPIHandle->pSPIx, Enable);
	//configure SPI_CR register
	uint32_t tempreg=0;
	//configure MASTER OR SLAVE SELECTION
	tempreg|=pSPIHandle->SPIConfig.SPI_DeviceMode<<2;
	//configure bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD){
	// clear BIDIMODE

		tempreg&=~(1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD){
		//SET BIDIMODE
		tempreg|=(1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//clear BIDIMODE
		tempreg&=~(1<<SPI_CR1_BIDIMODE);
		//SET RXONLY
		tempreg|=(1<<SPI_CR1_RXONLY);
	}
	//configure SCLK
	tempreg|=(pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR);
	//configure DFF
	tempreg|=(pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF);
	//configure CPOL
	tempreg|=(pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL);
	//configure CPHA
	tempreg|=(pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA);
	//configure SSM
	tempreg|=(pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM);
	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function resets the SPI ports
 *
 * @pSPIx[in]         - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}



//SENT DATA AND RECEIVE DATA
//Sending data
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data
 *
 * @pSPIx[in]         - base address of the spi peripheral
 * @pTxBuffer[in]     - THE DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len){
	if(Len!=0){
		while(Len>0){
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)==0){}

		if(pSPIx->CR1&(1<<SPI_CR1_DFF)){
			pSPIx->DR=*(uint16_t*)pTxBuffer;
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			pSPIx->DR=*pTxBuffer;
		    Len--;
		    pTxBuffer++;
		    }

		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data
 *
 * @pSPIx[in]         - base address of the SPI peripheral
 * @pRxBuffer[in]     - Given DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len){
	if(Len!=0){
		while(Len>0){
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)==0){}

		if(pSPIx->CR1&(1<<SPI_CR1_DFF)){
			*(uint16_t*)pRxBuffer=pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			*pRxBuffer=pSPIx->DR;
		    Len--;
		    pRxBuffer++;
		    }

		}
	}
}


//ISR AND IRQ
/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	uint8_t intpr_number=IRQNumber/4;
	uint8_t intpsection_number=IRQNumber%4;
	uint16_t intregister_shifting=(intpsection_number*8)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR+intpr_number)|=(IRQPriority<<intregister_shifting);
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function handles when interrupt happens
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1,temp2;
	//check for TXE
	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_TXE);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//TXE HANDLE
		spi_txe_interrupt_handle(pSPIHandle);
	}
	//check for RXNE
	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_RXNE);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2){
		//TXE HANDLE
		spi_rxne_interrupt_handle(pSPIHandle);
	}
	//check for OVR
	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_OVR);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_ERRIE);
	if(temp1 && temp2){
		//TXE HANDLE
		spi_ovr_interrupt_handle(pSPIHandle);
	}


}



/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function enables PE bits which is peripheral enable
 *
 * @pSPIx[in]         - base address of the SPI peripheral
 * @EnorDi[in]        -Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi==Enable){
		pSPIx->CR1|=(1<<SPI_CR1_SPE);
	}else{
		pSPIx->CR1&=~(1<<SPI_CR1_SPE);
	}
}


/*********************************************************************
 * @fn      		  - SPI_SSIControl
 *
 * @brief             - This function enables SSI bits which is slave select internal
 *
 * @pSPIx[in]         - base address of the SPI peripheral
 * @EnorDi[in]        -Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  FOR MORE INFORMATION , REFER TO REFERENCE MANUAL
 */
void SPI_SSIControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi==Enable){
		pSPIx->CR1|=(1<<SPI_CR1_SSI);
	}else{
		pSPIx->CR1&=~(1<<SPI_CR1_SSI);
	}
}
/*********************************************************************
 * @fn      		  - SPI_SSOEControl
 *
 * @brief             - This function enables SSOE bits which is SS outpu enable
 *
 * @pSPIx[in]         - base address of the SPI peripheral
 * @EnorDi[in]        -Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  FOR MORE INFORMATION , REFER TO REFERENCE MANUAL
 */
void SPI_SSOEControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi==Enable){
			pSPIx->CR2|=(1<<SPI_CR2_SSOE);
	}else{
			pSPIx->CR2&=~(1<<SPI_CR2_SSOE);
	}
}


//INTERRUPT SEND AND RECEIVE DATA
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function sends data with interrupt option
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @pRxBuffer[in]     - Given DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  The function just enables the essential interrupts
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len){
	uint8_t state=pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		//1. SAVE THE RX BUFFER ADDRESS AND LEN INFORMATION IN SOME GLOBAL VARIABLES
		pSPIHandle->pTxBuffer=pTxBuffer;
		pSPIHandle->TxLen=Len;
		//2. Mark the SPI state as busy in transmission so that
		//No other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState=SPI_BUSY_IN_TX;
		//3. Enable the  TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_TXEIE);
	}
	return state;


}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - This function sends data with interrupt option
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @pRxBuffer[in]     - Given DATA
 * @Len[in]        	  - Data Length
 *
 * @return            -  none
 *
 * @Note              -  The function just enables the essential interrupts
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len){
	uint8_t state=pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		//1. SAVE THE RX BUFFER ADDRESS AND LEN INFORMATION IN SOME GLOBAL VARIABLES
		pSPIHandle->pRxBuffer=pRxBuffer;
		pSPIHandle->RxLen=Len;
		//2. Mark the SPI state as busy in transmission so that
		//No other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState=SPI_BUSY_IN_RX;
		//3. Enable the  TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_RXNEIE);
	}
	return state;
}

//some helper functions

/*********************************************************************
 * @fn      		  - spi_txe_interrupt_handle
 *
 * @brief             - This function sends data and information that transmission has over
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1&(1<<SPI_CR1_DFF)){
		pSPIHandle->pSPIx->DR=(uint16_t*)*pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
	    (uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else{
		pSPIHandle->pSPIx->DR=*pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
	    pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen){
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - spi_rxne_interrupt_handle
 *
 * @brief             - This function reads data and information that reception has over
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1&(1<<SPI_CR1_DFF)){
		*((uint16_t*)pSPIHandle->pRxBuffer)=(uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
	    (uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else{
		*pSPIHandle->pRxBuffer=pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
	    pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen){
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - spi_ovr_interrupt_handle
 *
 * @brief             - This function clears overrun error and report to the application
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//CLEAR OVR FLAG
	if(pSPIHandle->TxState!=SPI_BUSY_IN_TX){
		temp=pSPIHandle->pSPIx->DR;
		temp=pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


/*********************************************************************
 * @fn      		  - SPI_ClearOVRFlag
 *
 * @brief             - Clears overrun flag
 *
 * @pSPIx[in]    - base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - if you don't know how to clear the OVR flag please refer to REFERENCE MANUAL
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	//CLEAR OVR FLAG
	temp=pSPIx->DR;
	temp=pSPIx->SR;
	(void)temp;
}
/*********************************************************************
 * @fn      		  - SPI_CloseTransmission
 *
 * @brief             - Clears interrupts and buffers which are about to transmission
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_TXEIE);
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_READY;
	pSPIHandle->pTxBuffer=NULL;
}

/*********************************************************************
 * @fn      		  - SPI_CloseReception
 *
 * @brief             - Clears interrupts and buffers which are about to reception
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_READY;
	pSPIHandle->pRxBuffer=NULL;
}

/*********************************************************************
 * @fn      		  - SPI_CloseReception
 *
 * @brief             - Clears interrupts and buffers which are about to reception
 *
 * @pSPIHandle[in]    - base address of the SPI peripheral and parameters of SPI options
 * @AppEv[in]         - application event number. it reports which event has come.
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - This function has weak attribute that means you can override on this function
 * 						Hence, User of applications can use transmission and reception events according to
 * 						the their applications
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){
	//This is a weak function.User application may override this function
}
