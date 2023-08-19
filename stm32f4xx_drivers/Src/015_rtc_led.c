/*
 * rtc_led.c
 *
 *  Created on: 17 Ağu 2023
 *      Author: ahmet
 */
#include <stdint.h>
#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK 16000000

char* get_string_datename(uint8_t i){
	char* days[]={"Monday","Tuesday","Wednesday","Thursday","Friday","Saturday", "Sunday"};
	return days[i-1];
}

void get_string(uint8_t data,char* chngdata){
	if(data<10){
		*chngdata='0';
		*(chngdata+1)=data+48;
	}else{
		*chngdata=(data/10)+48;
		*(chngdata+1)=(data%10)+48;
	}
}
char* get_string_time(RTC_Time_t *rtc_time){
	//hh:mm:ss
	static char data[9];
	data[2]=':';
	data[5]=':';
	get_string(rtc_time->hours,data);
	get_string(rtc_time->minutes,&data[3]);
	get_string(rtc_time->seconds,&data[6]);
	data[8]='\0';
	return data;
}
char* get_string_date(RTC_Date_t *rtc_date){
	//dd/mm/yy
	static char data[9];
	data[2]='/';
	data[5]='/';
	get_string(rtc_date->date,data);
	get_string(rtc_date->month,&data[3]);
	get_string(rtc_date->year,&data[6]);
	data[8]='\0';
	return data;
}
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}
int main(void){
	RTC_Time_t rtc_time;
	RTC_Date_t rtc_date;
	lcd_init();

	lcd_display_clear();
	lcd_display_return_home();

	rtc_date.day=FRIDAY;
	rtc_date.date=18;
	rtc_date.month=8;
	rtc_date.year=23;

	rtc_time.hours=11;
	rtc_time.minutes=31;
	rtc_time.seconds=50;
	rtc_time.time_format=TIME_FORMAT_12HRS_PM;

	if(DS1307_Init()){
		printf("Clock has not been initialized\n");
		fflush(stdout);
		while(1);
	}
	init_systick_timer(1);
	printf("Clock has been initialized\n");
	fflush(stdout);
	DS1307_Set_Current_Time(&rtc_time);
	DS1307_Set_Current_Date(&rtc_date);

	DS1307_Get_Current_Time(&rtc_time);
	DS1307_Get_Current_Date(&rtc_date);

	if(rtc_time.time_format==TIME_FORMAT_24HRS){
		//printf("The current time %s\n",get_string_time(&rtc_time));
		lcd_print_string(get_string_time(&rtc_time));

	}else{
		char * am_pm;
		am_pm=(rtc_time.time_format==TIME_FORMAT_12HRS_AM)? "AM": "PM";

		//printf("The current time is %s  %s\n",get_string_time(&rtc_time),am_pm);
		lcd_print_string(get_string_time(&rtc_time));
		lcd_print_string(am_pm);
		//fflush(stdout);
	}
	//printf("The current date is  %s <%s>\n",get_string_date(&rtc_date),get_string_datename(rtc_date.day));
	lcd_set_cursor(2, 1);
	lcd_print_string(get_string_date(&rtc_date));
	lcd_print_string(get_string_datename(rtc_date.day));
	//fflush(stdout);
	while(1);
	return 0;
}


void SysTick_Handler(void){
	RTC_Time_t rtc_time;
	RTC_Date_t rtc_date;
	DS1307_Get_Current_Time(&rtc_time);
	DS1307_Get_Current_Date(&rtc_date);
	lcd_display_clear();
	lcd_display_return_home();

	if(rtc_time.time_format==TIME_FORMAT_24HRS){
		//printf("The current time %s\n",get_string_time(&rtc_time));
		lcd_print_string(get_string_time(&rtc_time));

	}else{
		char * am_pm;
		am_pm=(rtc_time.time_format==TIME_FORMAT_12HRS_AM)? "AM": "PM";

		//printf("The current time is %s  %s\n",get_string_time(&rtc_time),am_pm);
		//fflush(stdout);
		lcd_print_string(get_string_time(&rtc_time));
		lcd_print_string(" ");
		lcd_print_string(am_pm);
	}
	//printf("The current date is  %s <%s>\n",get_string_date(&rtc_date),get_string_datename(rtc_date.day));
	//fflush(stdout);
	lcd_set_cursor(2, 1);
	lcd_print_string(get_string_date(&rtc_date));
	lcd_print_string("<");
	lcd_print_string(get_string_datename(rtc_date.day));
	lcd_print_string(">");
}
