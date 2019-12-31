#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <avr/io.h>

#define SSD1306_DISPLAY_WIDTH   128
#define SSD1306_DISPLAY_HEIGHT  64

#define SSD1306_USE_BIG_CHARS   0
#define SSD1306_GRAPHICS_MODE   0

void ssd1306_init(void);
void ssd1306_clear_screen(void);
void ssd1306_set_cursor(uint8_t x_pix, uint8_t line);
void ssd1306_putc(char c);
void ssd1306_puts(const char* s);
void ssd1306_disp_off(void);
void ssd1306_disp_on(void);
void ssd1306_scroll_left(uint8_t start, uint8_t stop, uint8_t interval);
void ssd1306_scroll_right(uint8_t start, uint8_t stop, uint8_t interval);
/**
 * Activate scroll
 */
extern void ssd1306_activate_scroll(void);

/**
 * Deactivate scroll
 */
extern void ssd1306_deactivate_scroll(void);


#if SSD1306_GRAPHICS_MODE
void ssd1306_draw_pixel(uint8_t x, uint8_t y);
void ssd1306_clear_pixel(uint8_t x, uint8_t y);
void ssd1306_display(void);
#endif

#endif