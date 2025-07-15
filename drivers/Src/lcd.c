/*
 * lcd.c
 *
 *  Created on: May 8, 2025
 *      Author: minhh
 */


#include "stm32f103xx.h"
#include "lcd.h"
#include "i2c.h"
#include "stdint.h"
#include "systemtick.h"

LCD_Data lcdData ;
LCD_State lcdState = {.bit = {.__r = 1}};
uint8_t pcfAddress = 0x27;

uint8_t LCD_CheckBF(I2C_Handle_t* i2c){
	uint16_t BF = 0;

	lcdData.get.RS = 0; //RS
	lcdData.get.RW = 1; //RW
	lcdData.get.E = 1; //E
	I2C_MasterSendPolling(i2c, &(lcdData.full), 1, pcfAddress);
	lcdData.get.E = 0; //E
	I2C_MasterSendPolling(i2c, &(lcdData.full), 1, pcfAddress);
	I2C_MasterRecivePolling(i2c, &BF, 1, pcfAddress);

	return BF;
}

void LCD_ClearScreen(I2C_Handle_t* i2c){
	uint8_t cmd = 0x01;
	/*----------Clear Screen----------*/

    // Gửi theo chuẩn 4-bit (sau khi đã vào 4-bit mode)
    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);         // LSB

    delay_ms(2);
}
void LCD_BackLightControl(I2C_Handle_t* i2c, uint8_t condition){
	if(condition==ENABLE){
		lcdData.get.BackLight = 1; //BACKLIGHT
	}else if(condition==DISABLE){
		lcdData.get.BackLight = 0; //BACKLIGHT
	}
}

void LCD_DisplaySetting(I2C_Handle_t* i2c, uint8_t display, uint8_t cursor, uint8_t blinking){
    uint8_t cmd = 0x0f;  // Mặc định 4-bit, 1 dòng
    if(cursor){
    	cmd |= (1<<1);
    }else if(!cursor){
    	cmd &= ~(1<<1);
    }
    if(blinking){
    	cmd |= (1<<0);
    }else if(!blinking){
    	cmd &= ~(1<<0);
    }
    if(display){
    	cmd |= (1<<2);
    }else if(!display){
    	cmd &= ~(1<<2);
    }
    lcdState.full = cmd;
    // Gửi theo chuẩn 4-bit (sau khi đã vào 4-bit mode)
    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);         // LSB

    delay_ms(1);
}

void LCD_Reset(I2C_Handle_t* i2c){
    uint8_t cmd = 0x03;  // Mặc định 4-bit, 1 dòng
    LCD_Send4bit(i2c,0,0, 0);
    // Gửi theo chuẩn 4-bit (sau khi đã vào 4-bit mode)
    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);         // LSB
    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);         // LSB
    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);         // LSB
}

void LCD_Send4bit(I2C_Handle_t* i2c, uint8_t RS, uint8_t RW, uint8_t data){
	lcdData.get.RS = RS; //RS
	lcdData.get.RW = RW; //RW
	lcdData.get.data = data;

	lcdData.get.E = 1; //E
	I2C_MasterSendPolling(i2c, &(lcdData.full), 1, pcfAddress);
	delay_ms(1);
	lcdData.get.E = 0; //E
	I2C_MasterSendPolling(i2c, &(lcdData.full), 1, pcfAddress);

	delay_ms(10);
}

void LCD_FucntionSet(I2C_Handle_t* i2c, uint8_t dataLength, uint8_t lineDisplay){
    uint8_t cmd = 0x20;
    if(dataLength==BIT_8_MODE){
    	cmd &= (1<<4);
    }
    if(lineDisplay==2){
    	cmd &= (1<<3);
    }
    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);

    delay_ms(1);
}

void LCD_SetEntryMode(I2C_Handle_t* i2c){
    uint8_t cmd = 0x06;

    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);

    delay_ms(1);
}

void LCD_ReturnHome(I2C_Handle_t* i2c){
    uint8_t cmd = 0x02;

    LCD_Send4bit(i2c,0,0, (cmd >> 4) & 0x0F);
    LCD_Send4bit(i2c,0,0, cmd & 0x0F);

    delay_ms(1);
}

void LCD_Init(I2C_Handle_t* i2c){
	delay_ms(1000);
	LCD_Reset(i2c);
	LCD_Send4bit(i2c,0,0, 2);
	LCD_BackLightControl(i2c, ENABLE);
	LCD_FucntionSet(i2c, BIT_4_MODE, 2);
	LCD_ClearScreen(i2c);
	LCD_Display(i2c, ENABLE);
	LCD_Cursor(i2c, DISABLE);
	LCD_Blinking(i2c, DISABLE);
//	LCD_DisplaySetting(i2c, ENABLE, DISABLE, DISABLE);
	LCD_SetEntryMode(i2c);
}

void LCD_SendData(I2C_Handle_t* i2c, uint8_t text){

    uint8_t cmd = text;

    LCD_Send4bit(i2c,1,0, (cmd >> 4) & 0x0F);
    LCD_Send4bit(i2c,1,0, cmd & 0x0F);

    delay_ms(1);
}

void LCD_Print(I2C_Handle_t* i2c, char* text){
	while(*text!='\0'){
		LCD_SendData(i2c,*text);
		text++;
	}
}

/**
 * @brief  Đặt vị trí con trỏ hiển thị trên LCD 16x2.
 *
 * LCD có 2 dòng hiển thị:
 *   - Dòng 0 (trên): địa chỉ DDRAM bắt đầu từ 0x00
 *   - Dòng 1 (dưới): địa chỉ DDRAM bắt đầu từ 0x40
 *
 * Lệnh điều khiển có định dạng 0b1xxxxxxx, trong đó 7 bit thấp (xxxxxxx)
 * chính là địa chỉ cần đặt trong DDRAM (0x00 - 0x4F).
 *
 * Lệnh gốc: 0x80 tương ứng với địa chỉ DDRAM = 0x00
 *
 * Tham số truyền vào:
 *   - `line`: 0 (dòng trên) hoặc 1 (dòng dưới)
 *   - `position`: cột hiển thị, từ 0 đến 15 (vị trí con trỏ trong dòng)
 *
 * Cách tính địa chỉ:
 *   - Dòng 0: địa chỉ = position
 *   - Dòng 1: địa chỉ = 0x40 + position
 *
 * Do đó, đoạn mã:
 *   cmd = 0x80;
 *   cmd |= (line << 6);     // Nếu line = 1 thì cộng thêm 0x40 (0100 0000)
 *   cmd |= (position);      // Cộng thêm vị trí trong dòng (0..15)
 *
 * sẽ cho ra địa chỉ DDRAM đúng với vị trí mong muốn.
 */
void LCD_setCuror(I2C_Handle_t* i2c, uint8_t position, uint8_t line){
	uint8_t cmd = 0x80;
	cmd |= (line<<6);
	cmd |= (position);
	LCD_Send4bit(i2c, 0, 0, (cmd >> 4) & 0x0F);
	LCD_Send4bit(i2c, 0, 0, cmd&0x0f);
	delay_ms(1);
}

void LCD_Read4bit(I2C_Handle_t* i2c, uint16_t* data){
	lcdData.get.RS = 1; //RS
	lcdData.get.RW = 1; //RW
//	lcdData.get.data = 0;
	I2C_MasterSendPolling(i2c, &(lcdData.full), 1, pcfAddress);

	lcdData.get.E = 1; //E
	I2C_MasterSendPolling(i2c, &(lcdData.full), 1, pcfAddress);
//	delay_ms(1);
	lcdData.get.E = 0; //E
	I2C_MasterSendPolling(i2c, &(lcdData.full), 1, pcfAddress);

	I2C_MasterRecivePolling(i2c, data, 4, pcfAddress);


//	delay_ms(1);
}

void LCD_Cursor(I2C_Handle_t* i2c, uint8_t condition){
	if(condition){
		lcdState.bit.cursor = 1;
	}else{
		lcdState.bit.cursor = 0;
	}

    LCD_Send4bit(i2c,0,0, (lcdState.full >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, lcdState.full & 0x0F);         // LSB

    delay_ms(1);

}

void LCD_Blinking(I2C_Handle_t* i2c, uint8_t condition){
	if(condition){
		lcdState.bit.blinking = 1;
	}else{
		lcdState.bit.blinking = 0;
	}
    LCD_Send4bit(i2c,0,0, (lcdState.full >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, lcdState.full & 0x0F);         // LSB

    delay_ms(1);
}

void LCD_Display(I2C_Handle_t* i2c, uint8_t condition){
	if(condition){
		lcdState.bit.display = 1;
	}else{
		lcdState.bit.display = 0;
	}
    LCD_Send4bit(i2c,0,0, (lcdState.full >> 4) & 0x0F);  // MSB
    LCD_Send4bit(i2c,0,0, lcdState.full & 0x0F);         // LSB

    delay_ms(1);
}
