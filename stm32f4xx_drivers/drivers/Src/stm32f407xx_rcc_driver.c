/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 11 Ağu 2023
 *      Author: ahmet
 */
#include "stm32f407xx_rcc_driver.h"


//BUS PRESCALERS
uint16_t AHB_PreScaler[8]={2,4,8,16,64,128,256,512};
uint8_t APBLow_PreScaler[4]={2,4,8,16};

uint32_t RCC_GetPLLValue(void){
	return 0;
}
/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             - This function gets frequency of APB1 bus
 *
 * @param    			-
 *
 * @return            -  Frequency of APB1 bus
 *
 * @Note              -  If you want to learn calculations of getting frequency from bus
 * 						Please refer to REFERENCE MANUAL PAGE 216, CLOCK TREE
 */
uint32_t RCC_GetPCLK1Value(void){
	uint32_t baseclk=0;
	uint32_t pclk1;
	uint8_t clksws=0,ahbpr,temp,apbpr;
	clksws=(RCC->CFGR>>2)&0x3;
	if(clksws==0){baseclk=16000000;}
	else if(clksws==1){baseclk=8000000;}
	else if(clksws==2){baseclk=RCC_GetPLLValue();}
	//calculate ahb prescaler
	temp=(RCC->CFGR>>4)&0xF;
	if(temp<8){ahbpr=1;}
	else{ahbpr=AHB_PreScaler[temp-8];}

	//calculate apb1 prescaler
	temp=(RCC->CFGR>>10)&0x7;
	if(temp<4){apbpr=1;}
	else{apbpr=AHB_PreScaler[temp-4];}
	//lets calculate pclk1
	pclk1=(baseclk/ahbpr)/apbpr;
	return pclk1;

}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - This function gets frequency of APB2 bus
 *
 * @param[in]         -
 *
 * @return            -  Frequency of APB2 bus
 *
 * @Note              -  If you want to learn calculations of getting frequency from bus
 * 						Please refer to REFERENCE MANUAL PAGE 216, CLOCK TREE
 */
uint32_t RCC_GetPCLK2Value(void){
	uint32_t baseclk=0;
	uint32_t pclk1;
	uint8_t clksws=0,ahbpr,temp,apbpr;
	clksws=(RCC->CFGR>>2)&0x3;
	if(clksws==0){baseclk=16000000;}
	else if(clksws==1){baseclk=8000000;}
	else if(clksws==2){baseclk=RCC_GetPLLValue();}
	//calculate ahb prescaler
	temp=(RCC->CFGR>>4)&0xF;
	if(temp<8){ahbpr=1;}
	else{ahbpr=AHB_PreScaler[temp-8];}

	//calculate apb2 prescaler
	temp=(RCC->CFGR>>13)&0x7;
	if(temp<4){apbpr=1;}
	else{apbpr=AHB_PreScaler[temp-4];}
	//lets calculate pclk1
	pclk1=(baseclk/ahbpr)/apbpr;
	return pclk1;
}
