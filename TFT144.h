/*
 * CTFT144.h
 *
 *  Created on: May 27, 2016
 *      Author: ehsan
 */

#ifndef TFT144_H_
#define TFT144_H_

#include <util/delay.h>
#include "tft144_config.h"

#define TFTWIDTH 128
#define TFTHEIGHT 128

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

#define sleep(n)	_delay_ms(n*1000)

#define _RGB565(b,g,r)	((((uint16_t)r)&0xf8)<<8)|((((uint16_t)g)&0xfc)<<3)|((((uint16_t)b)&0xf8)>>3)

typedef uint16_t colour;


inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b);

void tft144_init();
void tft144_ledon(uint8_t onoff);
uint8_t tft144_textX(uint8_t x, uint8_t font);
uint8_t tft144_textY(uint8_t y, uint8_t font);
void tft144_reset();
void tft144_write_command(const uint8_t address);
void tft144_write_data(const uint8_t data[], const uint8_t byte, uint8_t size);
void tft144_init_lcd(uint8_t orientation);
void tft144_clear_display(colour color);
void tft144_set_frame(u8 x1, u8 x2, u8 y1, u8 y2);
void tft144_draw_dot(u8 x, u8 y, colour color);
void tft144_draw_line(u8 x0, u8 y0, u8 x1, u8 y1, colour color);
void tft144_draw_rectangle(u8 x0, u8 y0, u8 x1, u8 y1, colour color);
void tft144_draw_filled_rectangle(u8 x0, u8 y0, u8 x1, u8 y1, colour color);
void tft144_draw_circle(u8 x0, u8 y0, u8 radio, colour color);
void tft144_putchar(u8 character, u8 x, u8 y, colour fgcolor, colour bgcolor, u8 font);
void tft144_putstring(u8 *str, u8 x, u8 y, colour fgcolor, colour bgcolor, u8 font);
void tft144_draw_bmp(u8 *filename, u8 x0, u8 y0);
void tft144_invert_screen();
void tft144_normal_screen();


#endif /* TFT144_H_ */
