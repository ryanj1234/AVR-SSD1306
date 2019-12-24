#ifndef __SSD1306_H__
#define __SSD1306_H__

#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64

void ssd1306_init(void);
void ssd1306_clear_screen(void);
void ssd1306_draw_pixel(uint8_t x, uint8_t y);
void ssd1306_clear_pixel(uint8_t x, uint8_t y);
void ssd1306_display(void);

#endif