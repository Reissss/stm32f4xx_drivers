/*
 * ds1307.c
 *
 *  Created on: 16 Ağu 2023
 *      Author: ahmet
 */

#include "ds1307.h"
#include <string.h>

I2C_Handle_t g_ds1307I2CHandle;

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t data,uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr );
static uint8_t bcd_to_binary(uint8_t data);
static uint8_t binary_to_bcd(uint8_t data);


//if returns 1 ,CH=1, so it is failed,if returns 0 ,CH=0, so it is succeed
uint8_t DS1307_Init(void){
	// initialize the i2c pins
	ds1307_i2c_pin_config();
	//initialize the i2c peripheral
	ds1307_i2c_config();
	//Enable I2C peripheral
	I2C_PeripheralControl(DS1307_I2C, Enable);
	//make clock halt =0
	ds1307_write(0x00,DS1307_ADDR_SEC);
	//check clock halt
	uint8_t clock_state=ds1307_read(DS1307_ADDR_SEC);
	return clock_state;
}

void DS1307_Set_Current_Time(RTC_Time_t* rtc_time){
	uint8_t seconds,hours;
	seconds=binary_to_bcd(rtc_time->seconds);
	seconds&=~(1<<7);
	ds1307_write(seconds, DS1307_ADDR_SEC);
	ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);
	hours=binary_to_bcd(rtc_time->hours);
	if(rtc_time->time_format==TIME_FORMAT_24HRS){
		hours&=~(1<<6);
	}else{
		hours|=(1<<6);
		hours=(rtc_time->time_format==TIME_FORMAT_12HRS_PM) ? hours|(1<<5) : hours&~(1<<5);
	}
	ds1307_write(hours, DS1307_ADDR_HRS);
}
void DS1307_Get_Current_Time(RTC_Time_t* rtc_time){
	uint8_t seconds,minutes,hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds&=~(1<<7);
	rtc_time->seconds=bcd_to_binary(seconds);
	minutes=ds1307_read(DS1307_ADDR_MIN);
	minutes&=~(1<<7);
	rtc_time->minutes=bcd_to_binary(minutes);
	hrs=ds1307_read(DS1307_ADDR_HRS);
	if(hrs&(1<<6)){
		//12 hour format
		rtc_time->time_format=!((hrs&(1<<5))==0);
		hrs&=~(0x7<<5);

	}else{
		rtc_time->time_format=TIME_FORMAT_24HRS;
	}
	rtc_time->hours=bcd_to_binary(hrs);
}

void DS1307_Set_Current_Date(RTC_Date_t* rtc_date){
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
}
void DS1307_Get_Current_Date(RTC_Date_t* rtc_date){
	rtc_date->date=bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	rtc_date->day=bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
	rtc_date->month=bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	rtc_date->year=bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}

static void ds1307_i2c_pin_config(void){
	GPIO_Handle_t iic_pins;
	memset(&iic_pins,0,sizeof(iic_pins));
	iic_pins.GPIO_PinConfig.GPIO_PinNumber=DS1307_I2C_SDA_PIN;//FOR SDA
	iic_pins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	iic_pins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	iic_pins.GPIO_PinConfig.GPIO_PinPuPdControl=DS1307_I2C_PUPD;
	iic_pins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	iic_pins.pGPIOx=DS1307_I2C_GPIO_PORT;
	iic_pins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_AF4;
	//SDA
	GPIO_Init(&iic_pins);
	//SCL
	iic_pins.GPIO_PinConfig.GPIO_PinNumber=DS1307_I2C_SCL_PIN;
	GPIO_Init(&iic_pins);
}
static void ds1307_i2c_config(void){
	memset(&g_ds1307I2CHandle,0,sizeof(g_ds1307I2CHandle));
	g_ds1307I2CHandle.pI2Cx=DS1307_I2C;
	g_ds1307I2CHandle.I2C_Config.I2C_ACKControl=I2C_ACK_ENABLE;
	g_ds1307I2CHandle.I2C_Config.I2C_SCLSpeed=DS1307_I2C_SPEED;
	I2C_Init(&g_ds1307I2CHandle);
}
static void ds1307_write(uint8_t data,uint8_t reg_addr){
	uint8_t tx[2];
	tx[0]=reg_addr;
	tx[1]=data;
	I2C_MasterSendData(&g_ds1307I2CHandle, tx, 2, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
}
static uint8_t ds1307_read(uint8_t reg_addr ){
	uint8_t data;
	I2C_MasterSendData(&g_ds1307I2CHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
	I2C_MasterReceiveData(&g_ds1307I2CHandle, &data, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
	return data;
}

static uint8_t bcd_to_binary(uint8_t data){
	uint8_t units_digit=0,tens_digit=0,result=0;
	units_digit=data&0x0f;
	tens_digit=(data>>4)&0x0f;
	result=(tens_digit*10)+units_digit;
	return result;
}
static uint8_t binary_to_bcd(uint8_t data){
	uint8_t units_digit=0,tens_digit=0,result=0;
	units_digit=data%10;
	tens_digit=data/10;
	result=units_digit;
	tens_digit=(tens_digit<<4)&0xf0;
	result|=tens_digit;
	return result;
}

