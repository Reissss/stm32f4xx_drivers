/*
 * lcd.c
 *
 *  Created on: 16 Ağu 2023
 *      Author: ahmet
 */


#include "lcd.h"
#include <string.h>

static void write_4_bits(uint8_t data);
static void lcd_enable(void);
static void mdelay(uint32_t value);
static void udelay(uint32_t value);


void lcd_send_command(uint8_t data){
	//make RS 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	//make RW 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(data >> 4);
	write_4_bits(data & 0x0F);
}

void lcd_send_char(uint8_t data){
	//make RS 1
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
	//make RW 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(data >> 4);
	write_4_bits(data & 0x0F);
}

void lcd_print_string(char *message){
	do{
		lcd_send_char((uint8_t)*message++);
	}while(*message!='\0');
}

void lcd_init(void){
	GPIO_Handle_t lcd_signal;
	memset(&lcd_signal,0,sizeof(lcd_signal));
	lcd_signal.pGPIOx=LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_RS;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	//RS PIN
	GPIO_Init(&lcd_signal);
	//RW PIN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);
	//EN PIN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);
	//D4 PIN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);
	//D5 PIN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);
	//D6 PIN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);
	//D7 PIN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	/**************************LCD INITIALIZING******************************/

	mdelay(40);

	//RS=0 FOR LCD COMMAND
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	//RW SET 0 TO WRINTG TO LCD
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x3);

	mdelay(5);

	write_4_bits(0x3);

	udelay(150);

	write_4_bits(0x3);

	write_4_bits(0x2);

	//function set command
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	//display on and cursor on
	lcd_send_command(LCD_CMD_DON_CURON);

	//display clear
	lcd_display_clear();

	//entry mode set
	lcd_send_command(LCD_CMD_INCADD);
}
void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
      lcd_send_command((column |= 0x80));
      break;
    case 2:
      /* Set cursor to 2nd row address and add index*/
        lcd_send_command((column |= 0xC0));
      break;
    default:
      break;
  }
}


static void write_4_bits(uint8_t data){
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((data>>0)&0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((data>>1)&0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((data>>2)&0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((data>>3)&0x01));

	lcd_enable();
}

static void lcd_enable(void){
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100);
}

void lcd_display_clear(void){
	lcd_send_command(LCD_CMD_DIS_CLEAR);
	mdelay(2);
}

static void mdelay(uint32_t value){
	for(uint32_t i=0;i<(value*1000);i++);
}
static void udelay(uint32_t value){
	for(uint32_t i=0;i<(value*1);i++);
}

void lcd_display_return_home(void){
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);
}





