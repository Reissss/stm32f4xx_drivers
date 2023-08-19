/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 2 Tem 2023
 *      Author: ahmet
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include "stm32f407xx.h"
#include <stdint.h>
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct{
	SPI_RegDef_t* pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t		 *pTxBuffer;/*|< To store the app. Tx buffer address >*/
	uint8_t		 *pRxBuffer;/*|< To store the app. Rx buffer address >*/
	uint32_t	 TxLen;		/*|< To store the Tx len >*/
	uint32_t	 RxLen;		/*|< To store the Rx len >*/
	uint8_t		 TxState;	/*|< To store the Tx state >*/
	uint8_t		 RxState;	/*|< To store the Rx state >*/

}SPI_Handle_t;



//Device Modes
#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE 			0

//BUS ROADS
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

//SPI CLOCK SPEED
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


//SPI DATA FRAME FORMAT
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1



//SPI CPHA
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

//SPI CPOL
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

//SPI SOFTWARE SLAVE MANAGEMENT
#define SPI_SSW_EN						1
#define SPI_SSW_DI						0


//SPI RELATED FLAGS
#define	SPI_TXE_FLAG                    (1<<SPI_SR_TXE)
#define	SPI_BSY_FLAG 					(1<<SPI_SR_BSY)
#define	SPI_RXNE_FLAG 					(1<<SPI_SR_RXNE)

//POSSIBLE SPI APPLICATION STATES
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

//POSSIBLE SPI EVENTS
#define SPI_EVENT_TX_CMPLT				1
#define	SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRR_ERR				4



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
//Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

//INIT AND DE-INIT
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);



//SENT DATA AND RECEIVE DATA

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);

//SENT DATA AND RECEIVE DATA WITH INTERRUPT OPTION

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len);


//ISR AND IRQ
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

//other api's
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//application callback function
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

//some helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
