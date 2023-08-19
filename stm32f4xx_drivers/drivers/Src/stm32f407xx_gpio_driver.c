/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 16 Haz 2023
 *      Author: ahmet
 */

#include "stm32f407xx_gpio_driver.h"


//Peripheral clock setup
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @pGPIOx[in]         - base address of the gpio peripheral
 * @EnorDi[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){

	if(EnorDi == Enable){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}

}

//INIT AND DE-INIT
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function setting essential bits to register and arranging the pins
 *
 * @pGPIOHandle[in]   - base address of the GPIO peripheral and parameters of GPIO pins
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, Enable);
	uint32_t temp=0;

	//1.Configure the GPIO MODE PIN

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER&=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER|=temp;
		temp=0;
	}else{
		SYSCFG_PCLK_EN();//ENABLES THE APB2 BUS
		//This part will contain the interrupts part
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
			//Configure FTSR
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){
			//Configure RTSR
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){
			//Configure FTSR AND RTSR
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// CONFIGURE SYSCONFIG
		uint8_t exticfg_num=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4);
		uint8_t exticfg_prt=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4);
		uint8_t portcode= GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[exticfg_num]|=(portcode<<exticfg_prt*4);

		//CONFIGURE EXTI IMR
		EXTI->IMR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2.Configure the speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR&=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;

	//3.Configure the PUPDR

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR&=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR|=temp;
	temp=0;

	//4.Configure the optype
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER&=~(0x1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER|=temp;
	temp=0;

	//5.Configure the altfn
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN){
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8)));
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8]&=~(0xF<<4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8));
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8]|=temp;
		temp=0;

	}



}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function resets the gpio ports
 *
 * @pGPIOx[in]   - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}


//Read AND Write
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the value of the pin
 *
 * @pGPIOx[in]   	  - base address of the GPIO peripheral
 * @PinNumber[in]     - Pin number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value=0;

	/*uint32_t temp=0;
	temp=(pGPIOx->IDR>>PinNumber);
	value=temp&0x1;*/


	value=(uint8_t)((pGPIOx->IDR>>PinNumber)&0x00000001);
	return value;

}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads the value of the port
 *
 * @pGPIOx[in]   	  - base address of the GPIO peripheral
 * @param[in]     -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value=0;
	value=pGPIOx->IDR;
	return value;

}



/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes the value to the pin
 *
 * @pGPIOx[in]   	  - base address of the GPIO peripheral
 * @PinNumber[in]     - pin number
 * @Value[in]         - value
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
	if(Value==GPIO_PIN_SET){
	pGPIOx->ODR|=(1<<PinNumber);
	}else{
		pGPIOx->ODR&=~(1<<PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes the value to the pin
 *
 * @pGPIOx[in]   	  - base address of the GPIO peripheral
 * @Value[in]         - value
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){
	pGPIOx->ODR=Value;

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function toggles the value of output pin
 *
 * @pGPIOx[in]   	  - base address of the GPIO peripheral
 * @PinNumber[in]      - pin number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR=pGPIOx->ODR^(1<<PinNumber);
}

//ISR AND IRQ
/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){

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
 * @fn      		  - GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	uint8_t intpr_number=IRQNumber/4;
	uint8_t intpsection_number=IRQNumber%4;
	uint16_t intregister_shifting=(intpsection_number*8)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR+intpr_number)|=(IRQPriority<<intregister_shifting);
}



/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function handles when interrupt happens
 *
 * @PinNumber[in]  	  - Pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & ( 1 << PinNumber))
	{
			//clear
			EXTI->PR |= ( 1 << PinNumber);
	}

}
