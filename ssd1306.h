#ifndef __SSD1306_H__
#define __SSD1306_H__

#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64

void ssd1306_init(void);
void ssd1306_clear_screen(void);
void ssd1306_set_cursor(uint8_t x_pix, uint8_t line);
void ssd1306_putc(char c);
void ssd1306_puts(const char* s);
void ssd1306_disp_off(void);
void ssd1306_disp_on(void);

void ssd1306_draw_pixel(uint8_t x, uint8_t y);
void ssd1306_clear_pixel(uint8_t x, uint8_t y);
void ssd1306_display(void);

#define SSD1306_USE_BIG_CHARS   1
#define SSD1306_GRAPHICS_MODE   1

#endif