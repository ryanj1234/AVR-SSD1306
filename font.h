/*
 *  font.h
 *  i2c
 *
 *  Created by Michael Köhler on 13.09.18.
 *  Copyright 2018 Skie-Systems. All rights reserved.
 *
 */
#ifndef _font_h_
#define _font_h_
#include <avr/pgmspace.h>

extern const char ssd1306oled_font[][6] PROGMEM;
// extern const char special_char[][2] PROGMEM;
void char2pixels(const char c, uint8_t *dat);

#define FONT        ssd1306oled_font
#define FONT_WIDTH  6

#endif