#ifndef __LCD12832_H
#define __LCD12832_H

#include "stm32l4xx_hal.h"   
#include "user_util.h"  

#define LCD_WIDTH   128
#define LCD_HEIGHT  32
#define LCD_PAGES   (LCD_HEIGHT/8)

#define FONT_WIDTH  6
#define FONT_HEIGHT 8

// Khai báo SPI handle t? main.c
extern SPI_HandleTypeDef hspi2;

#define SPI_CONNECT      hspi2

//// Ð?nh nghia chân di?u khi?n (s?a l?i theo board c?a b?n)
//#define LCD_CS_GPIO_Port   LCD_CS_GPIO_Port
//#define LCD_CS_Pin         LCD_CS_Pin
//
//#define LCD_A0_GPIO_Port   LCD_A0_GPIO_Port
//#define LCD_A0_Pin         LCD_A0_Pin
//
//#define LCD_RST_GPIO_Port  LCD_RST_GPIO_Port
//#define LCD_RST_Pin        LCD_RST_Pin

// Macro ti?n
#define LCD_CS_LOW()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_HIGH()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_A0_CMD()   HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET)
#define LCD_A0_DATA()  HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_SET)
#define LCD_RST_LOW()  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RST_HIGH() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)

// Buffer hien thi (512 byte = 128×32/8)
extern uint8_t lcd_buffer[LCD_PAGES][LCD_WIDTH];

// Các hàm chính
void LCD_Init(void);
void LCD_Clear(void);
void LCD_Update(void);
void LCD_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void LCD_DrawChar(uint8_t x, uint8_t page, char c);
void LCD_Print(uint8_t x, uint8_t page, const char *str);
void LCD_DrawPixel_Instant(uint8_t x, uint8_t y, uint8_t color);
void LCD_DrawHLine_Instant(uint8_t y, uint8_t x0, uint8_t x1, uint8_t color);
void LCD_DrawChar_Instant(uint8_t x, uint8_t page, char c);
void LCD_Print_Instant(uint8_t x, uint8_t page, const char *str);
#endif