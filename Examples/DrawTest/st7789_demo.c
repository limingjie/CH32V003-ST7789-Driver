/// \brief ST7789 Driver for CH32V003 - Demo
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
///  - Attribution - You must give appropriate credit, provide a link to the
///    license, and indicate if changes were made. You may do so in any
///    reasonable manner, but not in any way that suggests the licensor endorses
///    you or your use.
///  - NonCommercial - You may not use the material for commercial purposes.
///  - ShareAlike - If you remix, transform, or build upon the material, you
///    must distribute your contributions under the same license as the original.

#ifdef PLATFORMIO  // Use PlatformIO CH32V
    #include <debug.h>
#else  // Use ch32v003fun
    #include "ch32v003fun.h"
#endif

#include "st7789.h"

#include <stdint.h>

uint8_t rand8(void);

void popup(const char *msg, uint32_t delay)
{
    for (int i = 1; i < 11; i++)
    {
        tft_fill_rect((ST7789_WIDTH >> 1) - (i << 2), (ST7789_HEIGHT >> 1) - (i << 1), i << 3, i << 2, BLACK);
        Delay_Ms(10);
    }
    tft_set_cursor((ST7789_WIDTH >> 1) - 27, (ST7789_HEIGHT >> 1) - 3);

    tft_print(msg);

    Delay_Ms(delay);
}

int main(void)
{
#ifdef PLATFORMIO  // Use PlatformIO CH32V
    Delay_Init();
#else  // Use ch32v003fun
    SystemInit();
    Delay_Ms(100);
#endif

    tft_init();

    uint32_t frame = 0;

    uint16_t colors[] = {
        BLACK, NAVY, DARKGREEN, DARKCYAN, MAROON, PURPLE, OLIVE,  LIGHTGREY,   DARKGREY, BLUE,
        GREEN, CYAN, RED,       MAGENTA,  YELLOW, WHITE,  ORANGE, GREENYELLOW, PINK,
    };

    tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, RED);

    while (1)
    {
        tft_set_color(RED);
        tft_set_background_color(BLACK);

        popup("Draw Point", 1000);
        tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BLACK);

        frame = 30000;
        while (frame-- > 0)
        {
            tft_draw_pixel(rand8() % ST7789_WIDTH, rand8() % ST7789_HEIGHT, colors[rand8() % 19]);
        }

        popup("Scan Line", 1000);
        tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BLACK);

        frame = 50;
        while (frame-- > 0)
        {
            for (uint8_t i = 0; i < ST7789_WIDTH; i++)
            {
                tft_draw_line(i, 0, i, ST7789_HEIGHT, colors[rand8() % 19]);
            }
        }
        frame = 50;
        while (frame-- > 0)
        {
            for (uint8_t i = 0; i < ST7789_HEIGHT; i++)
            {
                tft_draw_line(0, i, ST7789_WIDTH, i, colors[rand8() % 19]);
            }
        }

        popup("Draw Line", 1000);
        tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BLACK);

        frame = 2000;
        while (frame-- > 0)
        {
            tft_draw_line(rand8() % ST7789_WIDTH, rand8() % ST7789_HEIGHT, rand8() % ST7789_WIDTH,
                          rand8() % ST7789_HEIGHT, colors[rand8() % 19]);
        }

        popup("Scan Rect", 1000);
        tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BLACK);

        frame = 100;
        while (frame-- > 0)
        {
            for (uint8_t i = 0; i < ST7789_HEIGHT >> 1; i++)
            {
                tft_draw_rect(i, i, ST7789_WIDTH - (i << 1), ST7789_HEIGHT - (i << 1), colors[rand8() % 19]);
            }
        }

        popup("Draw Rect", 1000);
        tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BLACK);

        frame = 5000;
        while (frame-- > 0)
        {
            tft_draw_rect(rand8() % ST7789_WIDTH, rand8() % ST7789_HEIGHT, 20, 20, colors[rand8() % 19]);
        }

        popup("Fill Rect", 1000);
        tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BLACK);

        frame = 5000;
        while (frame-- > 0)
        {
            tft_fill_rect(rand8() % ST7789_WIDTH, rand8() % ST7789_HEIGHT, 20, 20, colors[rand8() % 19]);
        }

        popup("Move Text", 1000);
        tft_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BLACK);

        frame      = 500;
        uint8_t  x = 0, y = 0, step_x = 1, step_y = 1;
        uint16_t bg = ORANGE;
        while (frame-- > 0)
        {
            if (frame % 20 == 0)
            {
                bg = colors[rand8() % 18 + 1];
            }
            tft_fill_rect(x, y, 88, 17, bg);
            tft_set_color(colors[rand8() % 19]);
            tft_set_background_color(bg);
            tft_set_cursor(x + 5, y + 5);
            tft_print("Hello, World!");
            Delay_Ms(25);

            x += step_x;
            if (x >= ST7789_WIDTH - 88)
            {
                step_x = -step_x;
            }
            y += step_y;
            if (y >= ST7789_HEIGHT - 17)
            {
                step_y = -step_y;
            }
        }
    }
}

/* White Noise Generator State */
#define NOISE_BITS      8
#define NOISE_MASK      ((1 << NOISE_BITS) - 1)
#define NOISE_POLY_TAP0 31
#define NOISE_POLY_TAP1 21
#define NOISE_POLY_TAP2 1
#define NOISE_POLY_TAP3 0
uint32_t lfsr = 1;

/*
 * random byte generator
 */
uint8_t rand8(void)
{
    uint8_t  bit;
    uint32_t new_data;

    for (bit = 0; bit < NOISE_BITS; bit++)
    {
        new_data = ((lfsr >> NOISE_POLY_TAP0) ^ (lfsr >> NOISE_POLY_TAP1) ^ (lfsr >> NOISE_POLY_TAP2) ^
                    (lfsr >> NOISE_POLY_TAP3));
        lfsr     = (lfsr << 1) | (new_data & 1);
    }

    return lfsr & NOISE_MASK;
}
