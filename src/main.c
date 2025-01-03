#include "stm32f1xx.h"
#include "st7735.h"

#define SYSTICK_LOAD_VALUE (SystemCoreClock / 1000) // 1ms interrupt

// Function prototypes
void SysTick_Init(void);
void SysTick_Handler(void);

// Global variable to track system ticks
volatile uint32_t systick_count = 0;



// Define GPIO and SPI registers and pins
#define LCD_CS_PORT       GPIOA
#define LCD_CS_PIN        (1 << 4)  // Replace with actual CS pin
#define LCD_DC_PORT       GPIOA
#define LCD_DC_PIN        (1 << 2)  // Replace with actual DC pin
#define LCD_RESET_PORT    GPIOA
#define LCD_RESET_PIN     (1 << 3)  // Replace with actual RESET pin

#define LCD_SPI           SPI1

// Function prototypes
void SPI_Init(void);
void GPIO_Init(void);
void Delay_ms(uint32_t ms);
void GPIO_Write(GPIO_TypeDef *port, uint32_t pin, uint8_t state);
void SPI_Write(uint8_t data);
int32_t LCD_IO_Init(void);
int32_t LCD_IO_WriteReg(uint8_t reg, uint8_t *data, uint32_t length);
int32_t LCD_IO_SendData(uint8_t *data, uint32_t length);
int32_t LCD_IO_GetTick(void);
void draw_circle(ST7735_Object_t *lcd, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void draw_rectangle(ST7735_Object_t *lcd, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

ST7735_Object_t LCD_Object;

ST7735_IO_t LCD_IO = {
    .Init = LCD_IO_Init,
    .DeInit = NULL,
    .WriteReg = LCD_IO_WriteReg,
    .ReadReg = NULL,
    .SendData = LCD_IO_SendData,
    .RecvData = NULL,
    .GetTick = LCD_IO_GetTick,
};



int main(void)
{
	
    GPIO_Init();
    SPI_Init();
    SysTick_Init();
    

    // Initialize LCD
    LCD_IO_Init();
    if (ST7735_RegisterBusIO(&LCD_Object, &LCD_IO) != ST7735_OK)
    {
        while (1); // Error loop
    }

    if (ST7735_Init(&LCD_Object, ST7735_FORMAT_RBG565, ST7735_ORIENTATION_PORTRAIT) != ST7735_OK)
    {
        while (1); // Error loop
    }

    // Turn on the display
    if (ST7735_DisplayOn(&LCD_Object) != ST7735_OK)
    {
        while (1); // Error loop
    }

    // Clear the display
    ST7735_FillRect(&LCD_Object, 0, 0, 128, 160, 0x0000); // Black background

    // Initial circle and rectangle
    draw_circle(&LCD_Object, 64, 80, 30, 0xF800); // Red circle
    draw_rectangle(&LCD_Object, 34, 50, 60, 60, 0x07E0); // Green rectangle

    uint16_t x = 64, y = 80; // Circle center
    uint16_t radius = 30;
    uint16_t dx = 1, dy = 1; // Movement direction

    while (1)
    {
        // Clear previous circle and rectangle
        draw_circle(&LCD_Object, x, y, radius, 0x0000); // Clear circle
        draw_rectangle(&LCD_Object, x - radius, y - radius, 2 * radius, 2 * radius, 0x0000); // Clear rectangle

        // Update circle position
        x += dx;
        y += dy;

        // Bounce off edges
        if (x - radius <= 0 || x + radius >= 128)
            dx = -dx;
        if (y - radius <= 0 || y + radius >= 160)
            dy = -dy;

        // Redraw circle and rectangle at new position
        draw_circle(&LCD_Object, x, y, radius, 0xF800); // Red circle
        draw_rectangle(&LCD_Object, x - radius, y - radius, 2 * radius, 2 * radius, 0x07E0); // Green rectangle

        // Delay for smooth animation
        Delay_ms(50);
    }
}

void SysTick_Init(void)
{
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);          // Выключаем счётчик
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);           // Разрешение прерывания
    SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);         // Источник тактирования AHB
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 72000-1); // Частота прерываний 1 кГц
    MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, 72000-1);  // Установка текущего значения
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);            // Включение счётчика
}

// SysTick interrupt handler
void SysTick_Handler(void)
{
    systick_count++;
}

// Draw a circle using the midpoint circle algorithm
void draw_circle(ST7735_Object_t *lcd, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
    int16_t x = r;
    int16_t y = 0;
    int16_t err = 0;

    while (x >= y)
    {
        ST7735_SetPixel(lcd, x0 + x, y0 + y, color);
        ST7735_SetPixel(lcd, x0 + y, y0 + x, color);
        ST7735_SetPixel(lcd, x0 - y, y0 + x, color);
        ST7735_SetPixel(lcd, x0 - x, y0 + y, color);
        ST7735_SetPixel(lcd, x0 - x, y0 - y, color);
        ST7735_SetPixel(lcd, x0 - y, y0 - x, color);
        ST7735_SetPixel(lcd, x0 + y, y0 - x, color);
        ST7735_SetPixel(lcd, x0 + x, y0 - y, color);

        y += 1;
        if (err <= 0)
        {
            err += 2 * y + 1;
        }
        if (err > 0)
        {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

// Draw a rectangle
void draw_rectangle(ST7735_Object_t *lcd, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    for (uint16_t i = x; i < x + width; i++)
    {
        ST7735_SetPixel(lcd, i, y, color);               // Top edge
        ST7735_SetPixel(lcd, i, y + height - 1, color);  // Bottom edge
    }

    for (uint16_t i = y; i < y + height; i++)
    {
        ST7735_SetPixel(lcd, x, i, color);               // Left edge
        ST7735_SetPixel(lcd, x + width - 1, i, color);   // Right edge
    }
}

// GPIO and SPI initialization
void GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA clock

    // Configure CS, DC, RESET as output
    GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4 | GPIO_CRL_MODE2 | GPIO_CRL_CNF2 | GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
    GPIOA->CRL |= (GPIO_CRL_MODE4_1 | GPIO_CRL_MODE2_1 | GPIO_CRL_MODE3_1); // Output mode, 2 MHz
}

void SPI_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI1 clock

    // Configure SPI1 pins (PA5-SCK, PA7-MOSI)
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |= (GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1); // PA5 as alternate function push-pull
    GPIOA->CRL |= (GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1); // PA7 as alternate function push-pull

	
  SPI1->CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1
	
  SPI1->CR1 |= (1<<2);  // Master Mode
	
  SPI1->CR1 |= (3<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
	
  SPI1->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first
	
  SPI1->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management
	
  SPI1->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex
	
  SPI1->CR1 &= ~(1<<11);  // DFF=0, 8 bit data
	
  SPI1->CR2 = 0;
}

void GPIO_Write(GPIO_TypeDef *port, uint32_t pin, uint8_t state)
{
    if (state)
        port->BSRR = pin; // Set pin
    else
        port->BRR = pin;  // Reset pin
}

void SPI_Write(uint8_t data)
{
    while (!(SPI1->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
    SPI1->DR = data;
    while (SPI1->SR & SPI_SR_BSY); // Wait until SPI is not busy
}

// LCD IO functions
int32_t LCD_IO_Init(void)
{
    GPIO_Write(LCD_RESET_PORT, LCD_RESET_PIN, 0); // Reset LCD
    Delay_ms(50);
    GPIO_Write(LCD_RESET_PORT, LCD_RESET_PIN, 1);
    Delay_ms(120);
}

int32_t LCD_IO_WriteReg(uint8_t reg, uint8_t *data, uint32_t length)
{
    GPIO_Write(LCD_CS_PORT, LCD_CS_PIN, 0);
    GPIO_Write(LCD_DC_PORT, LCD_DC_PIN, 0); // Command mode
    SPI_Write(reg);
    GPIO_Write(LCD_DC_PORT, LCD_DC_PIN, 1); // Data mode
    for (uint32_t i = 0; i < length; i++)
    {
        SPI_Write(data[i]);
    }
    GPIO_Write(LCD_CS_PORT, LCD_CS_PIN, 1);
}

int32_t LCD_IO_SendData(uint8_t *data, uint32_t length)
{
    GPIO_Write(LCD_CS_PORT, LCD_CS_PIN, 0);
    GPIO_Write(LCD_DC_PORT, LCD_DC_PIN, 1); // Data mode
    for (uint32_t i = 0; i < length; i++)
    {
        SPI_Write(data[i]);
    }
    GPIO_Write(LCD_CS_PORT, LCD_CS_PIN, 1);
}

int32_t LCD_IO_GetTick(void)
{
    return SysTick->VAL; // Assuming SysTick is configured
}

void Delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 8000; i++); // Approximation
}
