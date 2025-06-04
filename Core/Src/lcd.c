/*
 * lcd.c
 *
 *  Created on: Dec 1, 2020
 *      Author: fatay
 */
#include <stdint.h>
#include "lcd.h"
#include "lcd_config.h"


#define SET_IF(expr)  ((expr) ? GPIO_PIN_SET : GPIO_PIN_RESET)
char display_settings;

//Sending falling edge signal to EPin for waking up LCD
static void fallingEdge(void)
{
    HAL_GPIO_WritePin(EN_Pin.port, EN_Pin.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_Pin.port, EN_Pin.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_Pin.port, EN_Pin.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

#ifndef LCD8Bit
	static void send4Bits(char data)
	{
		HAL_GPIO_WritePin(DATA5_Pin.port, DATA5_Pin.pin, SET_IF(data&0x01));
		HAL_GPIO_WritePin(DATA6_Pin.port, DATA6_Pin.pin, SET_IF(data&0x02));
		HAL_GPIO_WritePin(DATA7_Pin.port, DATA7_Pin.pin, SET_IF(data&0x04));
		HAL_GPIO_WritePin(DATA8_Pin.port, DATA8_Pin.pin, SET_IF(data&0x08));

		fallingEdge();
	}
#endif

#ifdef LCD8Bit
	static void send8Bits(char val)
	{

		HAL_GPIO_WritePin(DATA1_Pin.port, DATA1_Pin.pin, SET_IF(val&0x01));
		HAL_GPIO_WritePin(DATA2_Pin.port, DATA2_Pin.pin, SET_IF(val&0x02));
		HAL_GPIO_WritePin(DATA3_Pin.port, DATA3_Pin.pin, SET_IF(val&0x04));
		HAL_GPIO_WritePin(DATA4_Pin.port, DATA4_Pin.pin, SET_IF(val&0x08));
		HAL_GPIO_WritePin(DATA5_Pin.port, DATA5_Pin.pin, SET_IF(val&0x10));
		HAL_GPIO_WritePin(DATA6_Pin.port, DATA6_Pin.pin, SET_IF(val&0x20));
		HAL_GPIO_WritePin(DATA7_Pin.port, DATA7_Pin.pin, SET_IF(val&0x40));
		HAL_GPIO_WritePin(DATA8_Pin.port, DATA8_Pin.pin, SET_IF(val&0x80));

		fallingEdge();
	}
#endif

static void sendCommand(char cmd)
{
	#ifdef LCD8Bit
    	HAL_GPIO_WritePin(GPIO_PORT, RS_Pin, GPIO_PIN_RESET);
		send8Bits(cmd);
	#else
	    HAL_GPIO_WritePin(RS_Pin.port, RS_Pin.pin, GPIO_PIN_RESET);
		send4Bits(cmd >> 4);
		send4Bits(cmd);
	#endif
}

static void sendData(char data)
{
	#ifdef LCD8Bit
    	HAL_GPIO_WritePin(GPIO_PORT, RS_Pin, GPIO_PIN_SET);
		send8Bits(data);
	#else
	    HAL_GPIO_WritePin(RS_Pin.port, RS_Pin.pin, GPIO_PIN_SET);
		send4Bits(data >> 4);
		send4Bits(data);
	#endif
}

void clearLCD(void)
{
	sendCommand(LCD_CLEARDISPLAY);
	HAL_Delay(5);
}

void putLCD(char c)
{
	sendData(c);
}

void writeLCD (char *str)
{
	for(; *str != 0; ++str)
	{
		sendData(*str);
	}
}

void initLCD(void)
{
    HAL_GPIO_WritePin(EN_Pin.port, EN_Pin.pin,  	 GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS_Pin.port, RS_Pin.pin, 	 GPIO_PIN_RESET);

	HAL_Delay(50);

	#ifdef LCD8Bit
		display_settings = LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS;
		sendCommand(LCD_FUNCTIONSET | display_settings);
		HAL_Delay(5);
		sendCommand(LCD_FUNCTIONSET | display_settings);
		HAL_Delay(5);
		sendCommand(LCD_FUNCTIONSET | display_settings);
		HAL_Delay(5);

	#else
		display_settings = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
		send4Bits(0x03);
		HAL_Delay(5);
		send4Bits(0x03);
		HAL_Delay(5);
		send4Bits(0x03);
		HAL_Delay(2);
		send4Bits(0x02);
		HAL_Delay(2);
	#endif
		sendCommand(LCD_FUNCTIONSET | display_settings);
		display_settings = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
		sendCommand(LCD_DISPLAYCONTROL | display_settings);
		HAL_Delay(2);

		clearLCD();
		display_settings =  LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
		sendCommand(LCD_ENTRYMODESET | display_settings);
		HAL_Delay(2);
}


void setCursor(char x, char y)
{
	uint8_t base = 0;

	if(y == 1)
	{
		base = 0x40;
	}
	else
	{
		base = 0;
	}

	sendCommand( 0x80 | (base + x));
}

void cursorOn(void)
{
	sendCommand(0x08 | 0x04 | 0x02);
}

void blinkOn(void)
{
	sendCommand(0x08 | 0x04 | 0x01);
}

void clearDisp(void)
{
	sendCommand(0x08 | 0x04 | 0x00);
}

void setDisplay(lcdDispSetting_t dispSetting)
{
	sendCommand(0x08 | (dispSetting & 0x07));
}
