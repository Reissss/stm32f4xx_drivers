/*
 * ds1307.h
 *
 *  Created on: 16 Ağu 2023
 *      Author: ahmet
 */

#ifndef DS1307_H_
#define DS1307_H_
#include "stm32f407xx.h"

/*Application configurable items*/
#define DS1307_I2C					I2C1
#define DS1307_I2C_GPIO_PORT		GPIOB
#define DS1307_I2C_SDA_PIN			GPIO_PIN_NO_7
#define DS1307_I2C_SCL_PIN			GPIO_PIN_NO_6
#define DS1307_I2C_SPEED			I2C_SCL_SPEED_SM //FM mode can not be used because ds1307 board does not support FM mode.
#define DS1307_I2C_PUPD				GPIO_PU


//Register Addresses
#define DS1307_ADDR_SEC				0X00
#define DS1307_ADDR_MIN				0X01
#define DS1307_ADDR_HRS				0X02
#define DS1307_ADDR_DAY				0X03
#define DS1307_ADDR_DATE			0X04
#define DS1307_ADDR_MONTH			0X05
#define DS1307_ADDR_YEAR			0X06


//Some macros
#define TIME_FORMAT_12HRS_AM		0
#define TIME_FORMAT_12HRS_PM		1
#define TIME_FORMAT_24HRS			2


#define DS1307_I2C_ADDRESS			0X68


#define MONDAY						1
#define TUESDAY						2
#define WEDNASDAY					3
#define THURSDAY					4
#define FRIDAY						5
#define SATURDAY					6
#define SUNDAY						7


typedef struct{
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}RTC_Date_t;

typedef struct{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_Time_t;

//Functions
uint8_t DS1307_Init(void);

void DS1307_Set_Current_Time(RTC_Time_t* rtc_time);
void DS1307_Get_Current_Time(RTC_Time_t* rtc_time);

void DS1307_Set_Current_Date(RTC_Date_t* rtc_date);
void DS1307_Get_Current_Date(RTC_Date_t* rtc_date);

#endif /* DS1307_H_ */
