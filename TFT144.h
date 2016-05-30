/*
 * TFT144.h
 *
 *  Created on: May 27, 2016
 *      Author: Ehsan
 */

#ifndef TFT144_H_
#define TFT144_H_

#include <util/delay.h>
#include "tft144_config.h"

#define TFTWIDTH 128
#define TFTHEIGHT 128

// abbreviation for times i get lazy :D
#define u8			uint8_t
#define u16			uint16_t
#define u32			uint32_t

// ILI9163 commands
#define NOP 0x00
#define SOFT_RESET 0x01
#define ENTER_SLEEP_MODE 0x10
#define EXIT_SLEEP_MODE 0x11
#define ENTER_PARTIAL_MODE 0x12
#define ENTER_NORMAL_MODE 0x13
#define EXIT_INVERT_MODE 0x20
#define ENTER_INVERT_MODE 0x21
#define SET_GAMMA_CURVE 0x26
#define SET_DISPLAY_OFF 0x28
#define SET_DISPLAY_ON 0x29
#define SET_COLUMN_ADDRESS 0x2A
#define SET_PAGE_ADDRESS 0x2B
#define WRITE_MEMORY_START 0x2C
#define SET_PARTIAL_AREA 0x30
#define SET_SCROLL_AREA 0x33
#define SET_ADDRESS_MODE 0x36
#define SET_SCROLL_START 0X37
#define EXIT_IDLE_MODE 0x38
#define ENTER_IDLE_MODE 0x39
#define SET_PIXEL_FORMAT 0x3A
#define WRITE_MEMORY_CONTINUE 0x3C
#define READ_MEMORY_CONTINUE 0x3E
#define FRAME_RATE_CONTROL1 0xB1
#define FRAME_RATE_CONTROL2 0xB2
#define FRAME_RATE_CONTROL3 0xB3
#define DISPLAY_INVERSION 0xB4
#define POWER_CONTROL1 0xC0
#define POWER_CONTROL2 0xC1
#define POWER_CONTROL3 0xC2
#define POWER_CONTROL4 0xC3
#define POWER_CONTROL5 0xC4
#define VCOM_CONTROL1 0xC5
#define VCOM_CONTROL2 0xC6
#define VCOM_OFFSET_CONTROL 0xC7
#define POSITIVE_GAMMA_CORRECT 0xE0
#define NEGATIVE_GAMMA_CORRECT 0xE1
#define GAM_R_SEL 0xF2

// sleep in seconds
#define sleep(n)	_delay_ms(n*1000)

// preprocessor color calculator
#define _RGB565(r,g,b)	((((uint16_t)r)&0xf8)<<8)|((((uint16_t)g)&0xfc)<<3)|((((uint16_t)b)&0xf8)>>3)
#define _BGR565(r,g,b)	((((uint16_t)b)&0xf8)<<8)|((((uint16_t)g)&0xfc)<<3)|((((uint16_t)r)&0xf8)>>3)

typedef uint16_t colour;

// default colors definition
#define BLUE 			_BGR565(0,0,255)
#define GREEN 			_BGR565(0,255,0)
#define RED 			_BGR565(255,0,0)
#define PINK 			_BGR565(255,120,120)
#define LIGHTBLUE 		_BGR565(120,120,255)
#define LIGHTGREEN 		_BGR565(120,255,120)
#define BLACK 			_BGR565(0,0,0)
#define WHITE 			_BGR565(255,255,255)
#define GREY 			_BGR565(120,120,120)
#define LIGHTGREY 		_BGR565(200,200,200)
#define YELLOW 			_BGR565(255,255,0)
#define MAGENTA 		_BGR565(255,0,255)
#define CYAN			_BGR565(0,255,255)

uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b);

void tft144_init();
void tft144_ledon(uint8_t onoff);
uint8_t tft144_textX(uint8_t x, uint8_t font);
uint8_t tft144_textY(uint8_t y, uint8_t font);
void tft144_reset();
void tft144_write_command(const uint8_t address);
void tft144_write_data(const uint8_t data[], const uint8_t byte, uint8_t size);
void tft144_init_lcd(uint8_t orientation);
void tft144_clear_display(colour color);
void tft144_set_frame(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2);
void tft144_draw_dot(uint8_t x, uint8_t y, colour color);
void tft144_draw_dot_alpha(uint8_t x, uint8_t y, colour color, float alpha);
void tft144_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, colour color);
void tft144_draw_bezier(uint8_t x[4], uint8_t y[4], colour color);
void tft144_draw_rectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, colour color);
void tft144_draw_filled_rectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, colour color);
void tft144_draw_circle(uint8_t x0, uint8_t y0, uint8_t radio, colour color);
uint8_t tft144_putchar(char c, uint8_t x, uint8_t y, colour fgcolor, colour bgcolor, uint8_t font, uint8_t w, uint8_t h, uint8_t offset);
void tft144_putstring(char *str, uint8_t *x, uint8_t *y, colour fgcolor, colour bgcolor, uint8_t font);
void tft144_draw_bmp(uint8_t *filename, uint8_t x0, uint8_t y0);
void tft144_invert_screen();
void tft144_normal_screen();


#endif /* TFT144_H_ */
