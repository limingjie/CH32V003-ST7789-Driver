/// \brief ST7789 Driver for CH32V003
///
/// \author Li Mingjie
///  - Email:  limingjie@outlook.com
///  - GitHub: https://github.com/limingjie/
///
/// \date Sep 2023
///
/// \section References
///  - https://github.com/moononournation/Arduino_GFX
///  - https://github.com/cnlohr/ch32v003fun/tree/master/examples/spi_oled
///
/// \copyright Attribution-NonCommercial-ShareAlike 4.0 (CC BY-NC-SA 4.0)

#include "st7789.h"

#ifdef PLATFORMIO  // Use PlatformIO CH32V
    #include <debug.h>
#else  // Use ch32v003fun
    #include "ch32v003fun.h"
#endif

#include "font5x7.h"

// CH32V003 Pin Definitions
#ifndef ST7789_NO_CS
    #define PIN_CS 2  // PC2
#endif
#define PIN_RESET 3  // PC3
#define PIN_DC    4  // PC4
#define SPI_SCLK  5  // PC5
#define SPI_MOSI  6  // PC6

#define DATA_MODE()    (GPIOC->BSHR |= 1 << PIN_DC)  // DC High
#define COMMAND_MODE() (GPIOC->BCR |= 1 << PIN_DC)   // DC Low
#define RESET_HIGH()   (GPIOC->BSHR |= 1 << PIN_RESET)
#define RESET_LOW()    (GPIOC->BCR |= 1 << PIN_RESET)
#ifndef ST7789_NO_CS
    #define START_WRITE() (GPIOC->BCR |= 1 << PIN_CS)   // CS Low
    #define END_WRITE()   (GPIOC->BSHR |= 1 << PIN_CS)  // CS High
#else
    #define START_WRITE()
    #define END_WRITE()
#endif

// PlatformIO Compatibility
#ifdef PLATFORMIO
    #define CTLR1_SPE_Set      ((uint16_t)0x0040)
    #define GPIO_CNF_OUT_PP    0x00
    #define GPIO_CNF_OUT_PP_AF 0x08
#endif

// Delays
#define ST7789_RST_DELAY     120  // ms, wait before sending Sleep Out command
#define ST7789_SWRESET_DELAY 120  // ms, wait for software reset finish
#define ST7789_SLPOUT_DELAY  120  // ms, wait for sleep out finish

// System Function Command List - Write Commands Only
#define ST7789_NOP       0x00  // NOP
#define ST7789_SWRESET   0x01  // Software Reset
#define ST7789_SLPIN     0x10  // Sleep In
#define ST7789_SLPOUT    0x11  // Sleep Out
#define ST7789_PTLON     0x12  // Partial Display Mode On
#define ST7789_NORON     0x13  // Normal Display Mode On
#define ST7789_INVOFF    0x20  // Display Inversion Off
#define ST7789_INVON     0x21  // Display Inversion On
#define ST7789_DISPOFF   0x28  // Display Off
#define ST7789_DISPON    0x29  // Display On
#define ST7789_CASET     0x2A  // Column Address Set
#define ST7789_RASET     0x2B  // Row Address Set
#define ST7789_RAMWR     0x2C  // Memory Write
#define ST7789_PLTAR     0x30  // Partial Area
#define ST7789_MADCTL    0x36  // Memory Data Access Control
#define ST7789_COLMOD    0x3A  // Interface Pixel Format
#define ST7789_RAMCTRL   0xB0  // RAM Control
#define ST7789_PORCTRL   0xB2  // Porch Setting
#define ST7789_GCTRL     0xB7  // Gate Control
#define ST7789_VCOMS     0xBB  // VCOMS Setting
#define ST7789_LCMCTRL   0xC0  // LCM Control
#define ST7789_VDVVRHEN  0xC2  // VDV and VRH Command Enable
#define ST7789_VRHS      0xC3  // VRH Set
#define ST7789_VDVS      0xC4  // VDV Set
#define ST7789_FRCTRL2   0xC6  // Frame Rate Control in Normal Mode
#define ST7789_PWCTRL1   0xD0  // Power Control 1
#define ST7789_PVGAMCTRL 0xE0  // Positive Voltage Gamma Control
#define ST7789_NVGAMCTRL 0xE1  // Negative Voltage Gamma Control

// MADCTL Parameters
#define ST7789_MADCTL_MH  0x04  // Bit 2 - Refresh Left to Right
#define ST7789_MADCTL_RGB 0x00  // Bit 3 - RGB Order
#define ST7789_MADCTL_BGR 0x08  // Bit 3 - BGR Order
#define ST7789_MADCTL_ML  0x10  // Bit 4 - Scan Address Increase
#define ST7789_MADCTL_MV  0x20  // Bit 5 - X-Y Exchange
#define ST7789_MADCTL_MX  0x40  // Bit 6 - X-Mirror
#define ST7789_MADCTL_MY  0x80  // Bit 7 - Y-Mirror

// COLMOD Parameter
#define ST7789_COLMOD_16_BPP 0x55  // 01010101 - 16-bit/pixel

static uint16_t _cursor_x                  = 0;
static uint16_t _cursor_y                  = 0;      // Cursor position (x, y)
static uint16_t _color                     = WHITE;  // Color
static uint16_t _bg_color                  = BLACK;  // Background color
static uint8_t  _buffer[ST7789_WIDTH << 1] = {0};    // DMA buffer, long enough to fill a row.

/// \brief Initialize SPI
/// \details Configure SPI, DMA, and RESET/DC/CS lines. CPOL = 1 and CPHA = 1.
/// \todo Add a mode parameter for CPHA and CPOL configuration.
static void SPI_init(void)
{
    // Enable GPIO Port C and SPI peripheral
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1;

#ifndef ST7789_NO_CS
    // PC2 - CS
    GPIOC->CFGLR &= ~(0xf << (PIN_CS << 2));
    GPIOC->CFGLR |= (GPIO_CNF_OUT_PP | GPIO_Speed_50MHz) << (PIN_CS << 2);
#endif

    // PC3 - RESET
    GPIOC->CFGLR &= ~(0xf << (PIN_RESET << 2));
    GPIOC->CFGLR |= (GPIO_CNF_OUT_PP | GPIO_Speed_50MHz) << (PIN_RESET << 2);

    // PC4 - DC
    GPIOC->CFGLR &= ~(0xf << (PIN_DC << 2));
    GPIOC->CFGLR |= (GPIO_CNF_OUT_PP | GPIO_Speed_50MHz) << (PIN_DC << 2);

    // PC5 - SCLK
    GPIOC->CFGLR &= ~(0xf << (SPI_SCLK << 2));
    GPIOC->CFGLR |= (GPIO_CNF_OUT_PP_AF | GPIO_Speed_50MHz) << (SPI_SCLK << 2);

    // PC6 - MOSI
    GPIOC->CFGLR &= ~(0xf << (SPI_MOSI << 2));
    GPIOC->CFGLR |= (GPIO_CNF_OUT_PP_AF | GPIO_Speed_50MHz) << (SPI_MOSI << 2);

    // Configure SPI
    SPI1->CTLR1 = SPI_CPHA_2Edge             // Bit 0     - Clock PHAse
                  | SPI_CPOL_High            // Bit 1     - Clock POLarity - idles at the logical high voltage
                  | SPI_Mode_Master          // Bit 2     - Master device
                  | SPI_BaudRatePrescaler_2  // Bit 3-5   - F_HCLK / 2
                  | SPI_FirstBit_MSB         // Bit 7     - MSB transmitted first
                  | SPI_NSS_Soft             // Bit 9     - Software slave management
                  | SPI_DataSize_8b          // Bit 11    - 8-bit data
                  | SPI_Direction_1Line_Tx;  // Bit 14-15 - 1-line SPI, transmission only
    SPI1->CRCR = 7;                          // CRC
    SPI1->CTLR2 |= SPI_I2S_DMAReq_Tx;        // Configure SPI DMA Transfer
    SPI1->CTLR1 |= CTLR1_SPE_Set;            // Bit 6     - Enable SPI

    // Enable DMA peripheral
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

    // Config DMA for SPI TX
    DMA1_Channel3->CFGR = DMA_DIR_PeripheralDST          // Bit 4     - Read from memory
                          | DMA_Mode_Circular            // Bit 5     - Circulation mode
                          | DMA_PeripheralInc_Disable    // Bit 6     - Peripheral address no change
                          | DMA_MemoryInc_Enable         // Bit 7     - Increase memory address
                          | DMA_PeripheralDataSize_Byte  // Bit 8-9   - 8-bit data
                          | DMA_MemoryDataSize_Byte      // Bit 10-11 - 8-bit data
                          | DMA_Priority_VeryHigh        // Bit 12-13 - Very high priority
                          | DMA_M2M_Disable;             // Bit 14    - Disable memory to memory mode
    DMA1_Channel3->PADDR = (uint32_t)&SPI1->DATAR;
}

/// \brief Send Data Through SPI via DMA
/// \param buffer Memory address
/// \param size Memory size
/// \param repeat Repeat times
static void SPI_send_DMA(const uint8_t* buffer, uint16_t size, uint16_t repeat)
{
    DMA1_Channel3->MADDR = (uint32_t)buffer;
    DMA1_Channel3->CNTR  = size;
    DMA1_Channel3->CFGR |= DMA_CFGR1_EN;  // Turn on channel

    // Circulate the buffer
    while (repeat--)
    {
        // Clear flag, start sending?
        DMA1->INTFCR = DMA1_FLAG_TC3;

        // Waiting for channel 3 transmission complete
        while (!(DMA1->INTFR & DMA1_FLAG_TC3))
            ;
    }

    DMA1_Channel3->CFGR &= ~DMA_CFGR1_EN;  // Turn off channel
}

/// \brief Send Data Directly Through SPI
/// \param data 8-bit data
static void SPI_send(uint8_t data)
{
    // Send byte
    SPI1->DATAR = data;

    // Waiting for transmission complete
    while (!(SPI1->STATR & SPI_STATR_TXE))
        ;
}

/// \brief Send 8-Bit Command
/// \param cmd 8-bit command
static void write_command_8(uint8_t cmd)
{
    COMMAND_MODE();
    SPI_send(cmd);
}

/// \brief Send 8-Bit Data
/// \param cmd 8-bit data
static void write_data_8(uint8_t data)
{
    DATA_MODE();
    SPI_send(data);
}

/// \brief Send 16-Bit Data
/// \param cmd 16-bit data
static void write_data_16(uint16_t data)
{
    DATA_MODE();
    SPI_send(data >> 8);
    SPI_send(data);
}

/// \brief Initialize ST7789
/// \details Initialization sequence from Arduino_GFX
/// https://github.com/moononournation/Arduino_GFX/blob/master/src/display/Arduino_ST7789.h
void tft_init(void)
{
    SPI_init();

    // Reset timing
    // ____  TRW >= 10us  ________________
    //     \_____________/  TRT <= 120ms
    RESET_HIGH();
    Delay_Ms(1);
    RESET_LOW();
    Delay_Ms(1);  // TRW >= 10us
    RESET_HIGH();
    Delay_Ms(ST7789_RST_DELAY);  // TRT <= 120ms

    START_WRITE();

    // Software Reset
    // write_command_8(ST7789_SWRESET);
    // Delay_Ms(ST7789_SWRESET_DELAY);

    // Sleep out
    write_command_8(ST7789_SLPOUT);
    Delay_Ms(ST7789_SLPOUT_DELAY);

    // Set Interface Pixel Format - 16-bit/pixel
    write_command_8(ST7789_COLMOD);
    write_data_8(ST7789_COLMOD_16_BPP);

    // Set rotation
    write_command_8(ST7789_MADCTL);
    // write_data_8(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);  // 0 - Horizontal - X-Y Swap & Y-Mirror
    // write_data_8(ST7789_MADCTL_RGB);                                        // 1 - Vertical
    write_data_8(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);  // 2 - Horizontal - X-Y Swap & X-Mirror
    // write_data_8(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);  // 3 - Vertical   - X-Mirror & Y-Mirror

    // write_command_8(ST7789_RAMCTRL);
    // write_data_8(0x00);  // Power on default - RAM access from MCU interface and Display operation selection from MCU
    //                      // interface.
    // write_data_8(0xF0);  // Power on default - Data translate EPF = 0x11 and MSB first.

    // write_command_8(ST7789_PORCTRL);
    // write_data_8(0x0C);  // Power on default
    // write_data_8(0x0C);
    // write_data_8(0x00);
    // write_data_8(0x33);
    // write_data_8(0x33);

    // write_command_8(ST7789_GCTRL);
    // write_data_8(0x35);  // Power on default - VGHS = 13.26V, VGLS = -10.42V

    // write_command_8(ST7789_VCOMS);
    // write_data_8(0x20);  // Power on default - 0.9V

    // write_command_8(ST7789_LCMCTRL);
    // write_data_8(0x2C);  // Power on default

    // write_command_8(ST7789_VDVVRHEN);
    // write_data_8(0x01);  // Power on default
    // write_data_8(0xff);

    // write_command_8(ST7789_VRHS);
    // write_data_8(0x0B);  // Power off sequence - +/-4.1V

    // write_command_8(ST7789_VDVS);
    // write_data_8(0x20);  // Power on default - 0V

    // write_command_8(0xC6);
    // write_data_8(0x0F);  // Power on default - 60Hz

    // write_command_8(ST7789_PWCTRL1);
    // write_data_8(0xA4);  // Power on default
    // write_data_8(0xA1);

    // write_command_8(ST7789_PVGAMCTRL);
    // write_data_8(0b11110000);  // V63P3, V63P2, V63P1, V63P0,  V0P3,  V0P2,  V0P1,  V0P0
    // write_data_8(0b00001001);  //     0,     0,  V1P5,  V1P4,  V1P3,  V1P2,  V1P1,  V1P0
    // write_data_8(0b00010011);  //     0,     0,  V2P5,  V2P4,  V2P3,  V2P2,  V2P1,  V2P0
    // write_data_8(0b00010010);  //     0,     0,     0,  V4P4,  V4P3,  V4P2,  V4P1,  V4P0
    // write_data_8(0b00010010);  //     0,     0,     0,  V6P4,  V6P3,  V6P2,  V6P1,  V6P0
    // write_data_8(0b00101011);  //     0,     0,  J0P1,  J0P0, V13P3, V13P2, V13P1, V13P0
    // write_data_8(0b00111100);  //     0, V20P6, V20P5, V20P4, V20P3, V20P2, V20P1, V20P0
    // write_data_8(0b01000100);  //     0, V36P2, V36P1, V36P0,     0, V27P2, V27P1, V27P0
    // write_data_8(0b01001011);  //     0, V43P6, V43P5, V43P4, V43P3, V43P2, V43P1, V43P0
    // write_data_8(0b00011011);  //     0,     0,  J1P1,  J1P0, V50P3, V50P2, V50P1, V50P0
    // write_data_8(0b00011000);  //     0,     0,     0, V57P4, V57P3, V57P2, V57P1, V57P0
    // write_data_8(0b00010111);  //     0,     0,     0, V59P4, V59P3, V59P2, V59P1, V59P0
    // write_data_8(0b00011101);  //     0,     0, V61P5, V61P4, V61P3, V61P2, V61P1, V61P0
    // write_data_8(0b00100001);  //     0,     0, V62P5, V62P4, V62P3, V62P2, V62P1, V62P0

    // write_command_8(ST7789_NVGAMCTRL);
    // write_data_8(0b11110000);  // V63P3, V63P2, V63P1, V63P0,  V0P3,  V0P2,  V0P1,  V0P0
    // write_data_8(0b00001001);  //     0,     0,  V1P5,  V1P4,  V1P3,  V1P2,  V1P1,  V1P0
    // write_data_8(0b00010011);  //     0,     0,  V2P5,  V2P4,  V2P3,  V2P2,  V2P1,  V2P0
    // write_data_8(0b00001100);  //     0,     0,     0,  V4N4,  V4N3,  V4N2,  V4N1,  V4N0
    // write_data_8(0b00001101);  //     0,     0,     0,  V6N4,  V6N3,  V6N2,  V6N1,  V6N0
    // write_data_8(0b00100111);  //     0,     0,  J0N1,  J0N0, V13N3, V13N2, V13N1, V13N0
    // write_data_8(0b00111011);  //     0, V20N6, V20N5, V20N4, V20N3, V20N2, V20N1, V20N0
    // write_data_8(0b01000100);  //     0, V36N2, V36N1, V36N0,     0, V27N2, V27N1, V27N0
    // write_data_8(0b01001101);  //     0, V43N6, V43N5, V43N4, V43N3, V43N2, V43N1, V43N0
    // write_data_8(0b00001011);  //     0,     0,  J1N1,  J1N0, V50N3, V50N2, V50N1, V50N0
    // write_data_8(0b00010111);  //     0,     0,     0, V57N4, V57N3, V57N2, V57N1, V57N0
    // write_data_8(0b00010111);  //     0,     0,     0, V59N4, V59N3, V59N2, V59N1, V59N0
    // write_data_8(0b00011101);  //     0,     0, V61N5, V61N4, V61N3, V61N2, V61N1, V61N0
    // write_data_8(0b00100001);  //     0,     0, V62N5, V62N4, V62N3, V62N2, V62N1, V62N0

    // Invert display
    write_command_8(ST7789_INVON);
    // write_command_8(ST7789_INVOFF); // Power on default

    // Normal display on
    // write_command_8(ST7789_NORON); // Power on default

    // Display On
    write_command_8(ST7789_DISPON);
    END_WRITE();
}

/// \brief Set Cursor Position for Print Functions
/// \param x X coordinate, from left to right.
/// \param y Y coordinate, from top to bottom.
/// \details Calculate offset and set to `_cursor_x` and `_cursor_y` variables
void tft_set_cursor(uint16_t x, uint16_t y)
{
    _cursor_x = x + ST7789_X_OFFSET;
    _cursor_y = y + ST7789_Y_OFFSET;
}

/// \brief Set Text Color
/// \param color Text color
/// \details Set to `_color` variable
void tft_set_color(uint16_t color)
{
    _color = color;
}

/// \brief Set Text Background Color
/// \param color Text background color
/// \details Set to `_bg_color` variable
void tft_set_background_color(uint16_t color)
{
    _bg_color = color;
}

/// \brief Set Memory Write Window
/// \param x0 Start column
/// \param y0 Start row
/// \param x1 End column
/// \param y1 End row
static void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    write_command_8(ST7789_CASET);
    write_data_16(x0);
    write_data_16(x1);
    write_command_8(ST7789_RASET);
    write_data_16(y0);
    write_data_16(y1);
    write_command_8(ST7789_RAMWR);
}

/// \brief Print a Character
/// \param c Character to print
/// \details DMA accelerated.
void tft_print_char(char c)
{
    const unsigned char* start = &font[c + (c << 2)];

    uint16_t sz = 0;
    for (uint8_t i = 0; i < FONT_HEIGHT; i++)
    {
        for (uint8_t j = 0; j < FONT_WIDTH; j++)
        {
            if ((*(start + j)) & (0x01 << i))
            {
                _buffer[sz++] = _color >> 8;
                _buffer[sz++] = _color;
            }
            else
            {
                _buffer[sz++] = _bg_color >> 8;
                _buffer[sz++] = _bg_color;
            }
        }
    }

    START_WRITE();
    tft_set_window(_cursor_x, _cursor_y, _cursor_x + FONT_WIDTH - 1, _cursor_y + FONT_HEIGHT - 1);
    DATA_MODE();
    SPI_send_DMA(_buffer, sz, 1);
    END_WRITE();
}

/// \brief Print a String
/// \param str String to print
void tft_print(const char* str)
{
    while (*str)
    {
        tft_print_char(*str++);
        _cursor_x += FONT_WIDTH + FONT_SPACING_HOR;
    }
}

/// \brief Print an Integer
/// \param num Number to print
/// \param width Expected width of the number.
/// Align left if it is less than the width of the number.
/// Align right if it is greater than the width of the number.
void tft_print_number(int32_t num, uint16_t width)
{
    static char str[12];
    uint8_t     position  = 11;
    uint8_t     negative  = 0;
    uint16_t    num_width = 0;

    // Handle negative number
    if (num < 0)
    {
        negative = 1;
        num      = -num;
    }

    str[position] = '\0';  // End of the string.
    while (num)
    {
        str[--position] = num % 10 + '0';
        num /= 10;
    }

    if (position == 11)
    {
        str[--position] = '0';
    }

    if (negative)
    {
        str[--position] = '-';
    }

    // Calculate alignment
    num_width = (11 - position) * (FONT_WIDTH + FONT_SPACING_HOR) - 1;
    if (width > num_width)
    {
        _cursor_x += width - num_width;
    }

    tft_print(&str[position]);
}

/// \brief Draw a Pixel
/// \param x X
/// \param y Y
/// \param color Pixel color
/// \details SPI direct write
void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    x += ST7789_X_OFFSET;
    y += ST7789_Y_OFFSET;
    START_WRITE();
    tft_set_window(x, y, x, y);
    write_data_16(color);
    END_WRITE();
}

/// \brief Fill a Rectangle Area
/// \param x Start X coordinate
/// \param y Start Y coordinate
/// \param width Width
/// \param height Height
/// \param color Fill Color
/// \details DMA accelerated.
void tft_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    x += ST7789_X_OFFSET;
    y += ST7789_Y_OFFSET;

    uint16_t sz = 0;
    for (uint16_t x = 0; x < width; x++)
    {
        _buffer[sz++] = color >> 8;
        _buffer[sz++] = color;
    }

    START_WRITE();
    tft_set_window(x, y, x + width - 1, y + height - 1);
    DATA_MODE();
    SPI_send_DMA(_buffer, sz, height);
    END_WRITE();
}

/// \brief Draw a Bitmap
/// \param x Start X coordinate
/// \param y Start Y coordinate
/// \param width Width
/// \param height Height
/// \param bitmap Bitmap
void tft_draw_bitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t* bitmap)
{
    x += ST7789_X_OFFSET;
    y += ST7789_Y_OFFSET;
    START_WRITE();
    tft_set_window(x, y, x + width - 1, y + height - 1);
    DATA_MODE();
    SPI_send_DMA(bitmap, width * height << 1, 1);
    END_WRITE();
}

/// \brief Draw a Vertical Line Fast
/// \param x0 Start X coordinate
/// \param y0 Start Y coordinate
/// \param x1 End X coordinate
/// \param y1 End Y coordinate
/// \param color Line color
/// \details DMA accelerated
static void _tft_draw_fast_v_line(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    x += ST7789_X_OFFSET;
    y += ST7789_Y_OFFSET;

    _buffer[0] = color >> 8;
    _buffer[1] = color;

    START_WRITE();
    tft_set_window(x, y, x, y + h - 1);
    DATA_MODE();
    SPI_send_DMA(_buffer, 2, h);
    END_WRITE();
}

/// \brief Draw a Horizontal Line Fast
/// \param x0 Start X coordinate
/// \param y0 Start Y coordinate
/// \param x1 End X coordinate
/// \param y1 End Y coordinate
/// \param color Line color
/// \details DMA accelerated
static void _tft_draw_fast_h_line(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    x += ST7789_X_OFFSET;
    y += ST7789_Y_OFFSET;

    _buffer[0] = color >> 8;
    _buffer[1] = color;

    START_WRITE();
    tft_set_window(x, y, x + w - 1, y);
    DATA_MODE();
    SPI_send_DMA(_buffer, 2, w);
    END_WRITE();
}

// Draw line helpers
#define _diff(a, b) ((a > b) ? (a - b) : (b - a))
#define _swap_int16_t(a, b) \
    {                       \
        int16_t t = a;      \
        a         = b;      \
        b         = t;      \
    }

/// \brief Bresenham's line algorithm from Arduino GFX
/// https://github.com/moononournation/Arduino_GFX/blob/master/src/Arduino_GFX.cpp
/// \param x0 Start X coordinate
/// \param y0 Start Y coordinate
/// \param x1 End X coordinate
/// \param y1 End Y coordinate
/// \param color Line color
/// \details SPI direct write
static void _tft_draw_line_bresenham(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    uint8_t steep = _diff(y1, y0) > _diff(x1, x0);
    if (steep)
    {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1)
    {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx   = x1 - x0;
    int16_t dy   = _diff(y1, y0);
    int16_t err  = dx >> 1;
    int16_t step = (y0 < y1) ? 1 : -1;

    for (; x0 <= x1; x0++)
    {
        if (steep)
        {
            tft_draw_pixel(y0, x0, color);
        }
        else
        {
            tft_draw_pixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0)
        {
            err += dx;
            y0 += step;
        }
    }
}

/// \brief Draw a Rectangle
/// \param x0 Start X coordinate
/// \param y0 Start Y coordinate
/// \param x1 End X coordinate
/// \param y1 End Y coordinate
/// \param color Line color
void tft_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    _tft_draw_fast_h_line(x, y, width, color);
    _tft_draw_fast_h_line(x, y + height - 1, width, color);
    _tft_draw_fast_v_line(x, y, height, color);
    _tft_draw_fast_v_line(x + width - 1, y, height, color);
}

/// \brief Draw Line Function from Arduino GFX
/// https://github.com/moononournation/Arduino_GFX/blob/master/src/Arduino_GFX.cpp
/// \param x0 Start X coordinate
/// \param y0 Start Y coordinate
/// \param x1 End X coordinate
/// \param y1 End Y coordinate
/// \param color Line color
void tft_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    if (x0 == x1)
    {
        if (y0 > y1)
        {
            _swap_int16_t(y0, y1);
        }
        _tft_draw_fast_v_line(x0, y0, y1 - y0 + 1, color);
    }
    else if (y0 == y1)
    {
        if (x0 > x1)
        {
            _swap_int16_t(x0, x1);
        }
        _tft_draw_fast_h_line(x0, y0, x1 - x0 + 1, color);
    }
    else
    {
        _tft_draw_line_bresenham(x0, y0, x1, y1, color);
    }
}
