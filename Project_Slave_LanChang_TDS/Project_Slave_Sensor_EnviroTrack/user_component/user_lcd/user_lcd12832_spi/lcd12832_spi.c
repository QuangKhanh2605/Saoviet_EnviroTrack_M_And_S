#include "lcd12832_spi.h"
#include "font5x7.h"

uint8_t lcd_buffer[LCD_PAGES][LCD_WIDTH];
uint8_t LCD_Buffer[128 * 4];  // 128 cot × 4 page = 512 byte

// Gui 1 byte SPI
static void LCD_SendByte(uint8_t data)
{
    HAL_SPI_Transmit(&SPI_CONNECT, &data, 1, 10);
}

// Gui lenh
static void LCD_Cmd(uint8_t cmd)
{
    LCD_A0_CMD();
    LCD_CS_LOW();
    LCD_SendByte(cmd);
    LCD_CS_HIGH();
}

// Gui du lieu
static void LCD_Data(uint8_t data)
{
    LCD_A0_DATA();
    LCD_CS_LOW();
    LCD_SendByte(data);
    LCD_CS_HIGH();
}

// Reset phan cung
static void LCD_Reset(void)
{
    LCD_RST_HIGH();
    HAL_Delay(10);
    LCD_RST_LOW();
    HAL_Delay(20);
    LCD_RST_HIGH();
    HAL_Delay(50);
}

// --- KHOI TAO LCD ---
void LCD_Init(void)
{
    LCD_Reset();

    LCD_Cmd(0xAE); // Display OFF
    LCD_Cmd(0xA2); // Bias 1/9
    LCD_Cmd(0xA0); // ADC normal
    LCD_Cmd(0xC8); // COM output reverse
    LCD_Cmd(0x2F); // Power control: booster, regulator, follower ON
    LCD_Cmd(0x21); // Internal resistor ratio
    LCD_Cmd(0x81); // Set contrast
    LCD_Cmd(0x1C); // Contrast value
    LCD_Cmd(0xAF); // Display ON

    LCD_Clear();
    LCD_Update();
}

// Xóa buffer
void LCD_Clear(void)
{
    for (uint8_t p = 0; p < LCD_PAGES; p++)
        for (uint8_t x = 0; x < LCD_WIDTH; x++)
            lcd_buffer[p][x] = 0x00;
}

// Cap nhat toàn màn hình
void LCD_Update(void)
{
    for (uint8_t page = 0; page < LCD_PAGES; page++)
    {
        LCD_Cmd(0xB0 | page);  // Page address
        LCD_Cmd(0x00);         // Lower column = 0
        LCD_Cmd(0x10);         // Higher column = 0

        LCD_A0_DATA();
        LCD_CS_LOW();
        HAL_SPI_Transmit(&SPI_CONNECT, lcd_buffer[page], LCD_WIDTH, 100);
        LCD_CS_HIGH();
    }
}

// Ve 1 pixel
void LCD_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;

    uint8_t page = y / 8;
    uint8_t bit  = 1 << (y % 8);

    if (color)
        lcd_buffer[page][x] |= bit;
    else
        lcd_buffer[page][x] &= ~bit;
}

// Hien thi ký tu (font 5x7)
void LCD_DrawChar(uint8_t x, uint8_t page, char c)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *bitmap = font5x7[c - 32];

    for (uint8_t i = 0; i < 5; i++)
        lcd_buffer[page][x + i] = bitmap[i];

    lcd_buffer[page][x + 5] = 0x00; // kho?ng tr?ng
}

// In chuoi ký tu
void LCD_Print(uint8_t x, uint8_t page, const char *str)
{
    while (*str)
    {
        LCD_DrawChar(x, page, *str++);
        x += 6;
        if (x > (LCD_WIDTH - 6)) break;
    }
}

void LCD_DrawPixel_Instant(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= 128 || y >= 32) return;  // Ngoài vùng hi?n th?

    uint8_t page = y / 8;
    uint8_t bit  = y % 8;

    uint16_t index = page * 128 + x;

    if (color)
        LCD_Buffer[index] |= (1 << bit);   // B?t pixel
    else
        LCD_Buffer[index] &= ~(1 << bit);  // T?t pixel

    // Ghi ngu?c l?i vào LCD (ch? 1 byte)
    LCD_Cmd(0xB0 | page);                 // Ch?n page
    LCD_Cmd(0x00 | (x & 0x0F));           // C?t th?p
    LCD_Cmd(0x10 | (x >> 4));             // C?t cao
    LCD_Data(LCD_Buffer[index]);
}

void LCD_DrawHLine_Instant(uint8_t y, uint8_t x0, uint8_t x1, uint8_t color)
{
    if (y >= LCD_HEIGHT) return;
    if (x1 >= LCD_WIDTH) x1 = LCD_WIDTH - 1;

    uint8_t page = y / 8;
    uint8_t bit  = y % 8;
    uint8_t mask = 1 << bit;

    for (uint8_t x = x0; x <= x1; x++)
    {
        if (color)
            lcd_buffer[page][x] |= mask;
        else
            lcd_buffer[page][x] &= ~mask;

        // --- Ghi tr?c ti?p ra LCD d? hi?n th? ngay ---
        LCD_Cmd(0xB0 | page);
        LCD_Cmd(0x00 | (x & 0x0F));
        LCD_Cmd(0x10 | (x >> 4));
        LCD_Data(lcd_buffer[page][x]);
    }
}

void LCD_DrawChar_Instant(uint8_t x, uint8_t page, char c)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *bitmap = font5x7[c - 32];

    // Chon vi tri con tro hien thi
    LCD_Cmd(0xB0 | page);          // Chon page (0–3)
    LCD_Cmd(0x00 | (x & 0x0F));    // Cot thap
    LCD_Cmd(0x10 | (x >> 4));      // Cot cao

    // Gui du lieu font (5 byte + 1 cot trong)
    LCD_A0_DATA();
    LCD_CS_LOW();
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t data = bitmap[i];
        HAL_SPI_Transmit(&SPI_CONNECT, &data, 1, 10);
    }
    uint8_t space = 0x00;
    HAL_SPI_Transmit(&SPI_CONNECT, &space, 1, 10);
    LCD_CS_HIGH();
}

void LCD_Print_Instant(uint8_t x, uint8_t page, const char *str)
{
    while (*str)
    {
        LCD_DrawChar_Instant(x, page, *str++);
        x += 6;  // Moi ky tu chiem 6 cot (5 font + 1 trong)
        if (x > (LCD_WIDTH - 6)) break; // tránh tràn màn
    }
}