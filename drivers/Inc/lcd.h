

#include "stm32f103xx.h"
#include "i2c.h"
#include "stdint.h"

#define BIT_4_MODE 0
#define BIT_8_MODE 1

typedef union{
	struct{
		uint8_t RS : 1;
		uint8_t RW : 1;
		uint8_t E : 1;
		uint8_t BackLight : 1;
		uint8_t data : 4;
	}get;
	uint16_t full;
}LCD_Data;


uint8_t LCD_CheckBF();
void LCD_Init(I2C_Handle_t* i2c);
void LCD_BackLightControl(I2C_Handle_t* i2c, uint8_t condition);
void LCD_Print(I2C_Handle_t* i2c, char* text);
void LCD_SendData(I2C_Handle_t* i2c, uint8_t text);
void LCD_ClearScreen(I2C_Handle_t* i2c);
void LCD_Reset(I2C_Handle_t* i2c);
void LCD_SetEntryMode(I2C_Handle_t* i2c);
void LCD_FucntionSet(I2C_Handle_t* i2c, uint8_t dataLength, uint8_t lineDisplay);
void LCD_DisplaySetting(I2C_Handle_t* i2c, uint8_t display, uint8_t cursor, uint8_t blinking);
void LCD_Send4bit(I2C_Handle_t* i2c, uint8_t RS, uint8_t RW, uint8_t data);
void LCD_Read4bit(I2C_Handle_t* i2c, uint16_t* data);
void LCD_setCuror(I2C_Handle_t* i2c, uint8_t position, uint8_t line);
void LCD_ReturnHome(I2C_Handle_t* i2c);
