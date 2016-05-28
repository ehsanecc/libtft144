/*
 * 
 *
 *  Created on: May 27, 2016
 *      Author: Ehsan Varasteh
 *
 *   pure AVR ( no arduino or any starter board ) tft144 library
 *   use for driving ILI9163 lcds, converted code from Python
 *   you can see licenses for original code below
 *   some tweaks will add to make it more optimise for AVR use
 **************************************************************************
 *   Raspberry Pi Serial-SPI version
 *        eg http://www.ebay.com.au/itm/141239781210     - under $4!
 *   Both "red" and "black" boards supported as from V1.6 April 2015
 *   Board has inbuilt 5V-3V (2.9?) regulator (which does NOT break out the 3V!!)
 *   As far as I can discern, logic level is still 3.3V limit, despite supply is 5V.
 *   Currently the code here is designed simply for case of 128x128 pixels.
 *   Brian Lavery (C) Oct 2014    brian (at) blavery (dot) com
 *   Added: SPI access, BMP file load, double size fonts, python class
 *   Works on: Rasp Pi GPIO,    or "virtual GPIO" 3.3V   (identical library for both)
 *
 *                   THIS BOARD WORKS A TREAT !!!!                         BL
 *
 *************************************************************************
 *
 *   (1) Based on ILI9163 128x128 LCD library   - parallel I/O AVR C code
 *      Copyright (C) 2012 Simon Inns    Email: simon.inns@gmail.com
 *      http://www.waitingforfriday.com/index.php/Reverse_Engineering_a_1.5_inch_Photoframe
 *   (2) ... then based on Antares python/parallel Raspberry Pi code:
 *      http://www.raspberrypi.org/forums/viewtopic.php?t=58291&p=450201
 *   (3) ... making this version lib_tft144 python SPI interface for RPI or Virtual GPIO
 *      (It's looking a bit different now from Inns' original!)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *************************************************************************/

#include "TFT144.h"
#include "tft144_fonts.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// these colors calculated once for all
const colour
	BLUE=_RGB565(0,0,255),
	GREEN=_RGB565(0,255,0),
	RED=_RGB565(255,0,0),
	PINK=_RGB565(255,120,120),
	LIGHTBLUE=_RGB565(120,120,255),
	LIGHTGREEN=_RGB565(120,255,120),
	BLACK=_RGB565(0,0,0),
	WHITE=_RGB565(255,255,255),
	GREY=_RGB565(120,120,120),
	LIGHTGREY=_RGB565(200,200,200),
	YELLOW=_RGB565(255,255,0),
	MAGENTA=_RGB565(255,0,255),
	CYAN=_RGB565(0,255,255);

/* global vars */
uint8_t fontW, fontH;

/* rgb565 */
inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
	return ((((uint16_t)b)&0xf8)<<8)|((((uint16_t)g)&0xfc)<<3)|((((uint16_t)r)&0xf8)>>3);
}

/* quick AVR SPI read&write method */
void spi_write(const uint8_t data) {
	TFT_CTRL_PORT &= ~(1<<TFT_CE); // active
	SPDR = data;
	while(!(SPSR&(1<<SPIF)));
	TFT_CTRL_PORT |= (1<<TFT_CE); // de-active
}

void tft144_init() {
	// init SPI
	SPCR = (1<<SPE)|(1<<MSTR); // Master Mode, SPI Enabled
	SPSR = (1<<SPI2X); // fsck/2 (4MHZ)

	TFT_CTRL_DDR |= (1<<TFT_DC)|(1<<TFT_RST)|(1<<TFT_LED)|(1<<TFT_CE); // DC(A0),RST,LED
	TFT_CTRL_PORT |= (1<<TFT_DC)|(1<<TFT_RST);
	tft144_ledon(1);
	sleep(0.5);

	tft144_init_lcd(ORIENTATION);
}

void tft144_ledon(uint8_t onoff) {
	if(onoff)
		TFT_CTRL_PORT |= (1<<TFT_LED);
	else
		TFT_CTRL_PORT &= ~(1<<TFT_LED);
}

uint8_t tft144_textX(uint8_t x, uint8_t font) {
	return fontDim[font][0];
}

uint8_t tft144_textY(uint8_t y, uint8_t font) {
	return fontDim[font][1];
}

void tft144_reset() {
	TFT_CTRL_PORT &= ~(1<<TFT_RST); // low
	sleep(0.2);
	TFT_CTRL_PORT |= (1<<TFT_RST); // high
	sleep(0.2);
}

void tft144_write_command(const uint8_t address) {
	TFT_CTRL_PORT &= ~(1<<TFT_DC); //low
	spi_write(address);
}

void tft144_write_data(const uint8_t data[], const uint8_t byte, uint8_t size) {
	int n;

	TFT_CTRL_PORT |= (1<<TFT_DC); // high
	if(data != NULL && size > 1)
		for(n=0;n<size;n++)
			spi_write(data[n]);

	else spi_write(byte);
}

void tft144_init_lcd(uint8_t orientation) {
	const uint8_t
		_PGC[15]={0x3f, 0x25, 0x1c, 0x1e, 0x20, 0x12, 0x2a, 0x90, 0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00},
		_NGC[15]={0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xa7, 0x3d, 0x18, 0x25, 0x2a, 0x2b, 0x2b, 0x3a},
		_FRC1[2]={0x08, 0x08},
		_PC1[2]= {0x0a, 0x02},
		_VC1[2]= {0x50, 0x5b};

	tft144_reset();
	tft144_write_command(EXIT_SLEEP_MODE);
	sleep(0.05);
	tft144_write_command(SET_PIXEL_FORMAT);
	tft144_write_data(NULL,0x05,1);
	tft144_write_command(SET_GAMMA_CURVE);
	tft144_write_data(NULL,0x04,1);
	tft144_write_command(GAM_R_SEL);
	tft144_write_data(NULL,0x01,1);

	tft144_write_command(POSITIVE_GAMMA_CORRECT);
	tft144_write_data(_PGC,0,15);

	tft144_write_command(NEGATIVE_GAMMA_CORRECT);
	tft144_write_data(_NGC,0,15);
	tft144_write_command(FRAME_RATE_CONTROL1);
	tft144_write_data(_FRC1,0,2);

	tft144_write_command(DISPLAY_INVERSION);
	tft144_write_data(NULL,0x01,1);

	tft144_write_command(POWER_CONTROL1);
	tft144_write_data(_PC1,0,2);

	tft144_write_command(POWER_CONTROL2);
	tft144_write_data(NULL,0x02,1);

	tft144_write_command(VCOM_CONTROL1);
	tft144_write_data(_VC1,0,2);

	tft144_write_command(VCOM_OFFSET_CONTROL);
	tft144_write_data(NULL,0x40,1);
	tft144_set_frame(0,TFTWIDTH-1,0,TFTHEIGHT-1);

	tft144_write_command(SET_ADDRESS_MODE);
	tft144_write_data(NULL,orientation,1);

	tft144_clear_display(BLACK);
	tft144_write_command(SET_DISPLAY_ON);
}

void tft144_clear_display(colour color) {
	uint8_t color_hi = color>>8, color_lo=color&0xff;
	uint16_t n;

	tft144_set_frame(0,TFTWIDTH-1,0,TFTHEIGHT-1);
	tft144_write_command(WRITE_MEMORY_START);
	TFT_CTRL_PORT |= (1<<TFT_DC); // high(data)
	for(n=0;n<16384;n++) {
		spi_write(color_hi);
		spi_write(color_lo);
	}
}

void tft144_set_frame(u8 x1, u8 x2, u8 y1, u8 y2) {
	if(REDBOARD) {
		if(ORIENTATION == ORIENTATION0) {
			y1 += 32;
			y2 += 32;
		}
		if(ORIENTATION == ORIENTATION90) {
			x1 += 32;
			x2 += 32;
		}
	}
	tft144_write_command(SET_COLUMN_ADDRESS);
	tft144_write_data(NULL, 0x00, 1);
	tft144_write_data(NULL, x1, 1);
	tft144_write_data(NULL, 0x00, 1);
	tft144_write_data(NULL, x2, 1);

	tft144_write_command(SET_PAGE_ADDRESS);
	tft144_write_data(NULL, 0x00, 1);
	tft144_write_data(NULL, y1, 1);
	tft144_write_data(NULL, 0x00, 1);
	tft144_write_data(NULL, y2, 1);
}

void tft144_draw_dot(u8 x, u8 y, colour color) {
	uint8_t color_hi = color>>8, color_lo=color&0xff;

	tft144_set_frame(x, x+1, y, y+1);
	tft144_write_command(WRITE_MEMORY_START);
	TFT_CTRL_PORT |= (1<<TFT_DC); // high(data)
	spi_write(color_hi);
	spi_write(color_lo);
}

void tft144_draw_line(u8 x0, u8 y0, u8 x1, u8 y1, colour color) {
	uint8_t dx, dy, stepy, stepx, fraction;

	dy = y1 - y0;
	dx = x1 - x0;
	if (dy < 0) {
		dy = -dy;
		stepy = -1;
	} else
		stepy = 1;
	if (dx < 0) {
		dx = -dx;
		stepx = -1;
	} else
		stepx = 1;
	dx <<= 1;
	dy <<= 1;
	tft144_draw_dot(x0, y0, color);
	if (dx > dy) {
		fraction = dy - (dx >> 1);
		while (x0 != x1) {
			if (fraction >= 0) {
				y0 += stepy;
				fraction -= dx;
			}
			x0 += stepx;
			fraction += dy;
			tft144_draw_dot(x0, y0, color);
		}
	} else {
		fraction = dx - (dy >> 1);
		while (y0 != y1) {
			if (fraction >= 0) {
				x0 += stepx;
				fraction -= dy;
			}
			y0 += stepy;
			fraction += dx;
			tft144_draw_dot(x0, y0, color);
		}
	}
}

void tft144_draw_rectangle(u8 x0, u8 y0, u8 x1, u8 y1, colour color) {
	tft144_draw_line(x0, y0, x0, y1, color);
	tft144_draw_line(x0, y1, x1, y1, color);
	tft144_draw_line(x1, y0, x1, y1, color);
	tft144_draw_line(x0, y0, x1, y0, color);
}

void tft144_draw_filled_rectangle(u8 x0, u8 y0, u8 x1, u8 y1, colour color) {
	uint8_t color_hi = color >> 8, color_lo = color & 0xff, a, b;

	tft144_set_frame(x0, x1, y0, y1);

	tft144_write_command(WRITE_MEMORY_START);
	TFT_CTRL_PORT |= (1 << TFT_DC); // high(data)
	for (a = 0; a < (1 + x1 - x0); a++)
		for (b = 0; b < (y1 - y0); b++) {
			spi_write(color_hi);
			spi_write(color_lo);
		}
}

void tft144_draw_circle(u8 x0, u8 y0, u8 radio, colour color) {
	uint8_t error, errorx, errory, y, x;

	error = 1 - radio;
	errorx = 1;
	errory = -2 * radio;
	y = radio;
	x = 0;
	tft144_draw_dot(x0, y0 + radio, color);
	tft144_draw_dot(x0, y0 - radio, color);
	tft144_draw_dot(x0 + radio, y0, color);
	tft144_draw_dot(x0 - radio, y0, color);
	while (x < y) {
		if (error >= 0) {
			y -= 1;
			errory += 2;
			error += errory;
		}
		x += 1;
		errorx += 2;
		error += errorx;
		tft144_draw_dot(x0 + x, y0 + y, color);
		tft144_draw_dot(x0 - x, y0 + y, color);
		tft144_draw_dot(x0 + x, y0 - y, color);
		tft144_draw_dot(x0 - x, y0 - y, color);
		tft144_draw_dot(x0 + y, y0 + x, color);
		tft144_draw_dot(x0 - y, y0 + x, color);
		tft144_draw_dot(x0 + y, y0 - x, color);
		tft144_draw_dot(x0 - y, y0 - x, color);
	}
}

void tft144_putchar(u8 character, u8 x, u8 y, colour fgcolor, colour bgcolor, u8 font) {
	uint8_t fgcolor_hi = fgcolor >> 8, fgcolor_lo = fgcolor & 0xff, bgcolor_hi =
			bgcolor >> 8, bgcolor_lo = bgcolor & 0xff, fontScale, xx[4], *cbuf,
			column, row, flor[2], topleft, pixOn, rpt;

	fontW = fontDim[font][0];
	fontH = fontDim[font][1];
	fontScale = fontDim[font][2];
	if (!(font == 3 || font == 4)) {   // restricted char set 32-126 for most
		if (character < 32 || character > 126) // only strictly ascii chars
			character = 0;
		else
			character -= 32;
		tft144_set_frame(x, (x + fontW - 1), y, (y + fontH - 1));

		if (fontScale == 2) {
			//xx = [0, 2, 2 * fontW, 2 + (2 * fontW) ]   // DOUBLE: every pixel becomes a 2x2 pixel
			xx[0] = 0;
			xx[1] = 2;
			xx[2] = 2 * fontW;
			xx[3] = 2 + (2 * fontW);
		}

		tft144_write_command(WRITE_MEMORY_START);
		cbuf = (uint8_t*) malloc(fontW * fontH * 2);
		flor[0] = floor(fontH / fontScale);
		flor[1] = floor(fontW / fontScale);
		for (column = 0; column < flor[0]; column++) {
			for (row = 0; row < flor[1]; row++) {
				topleft = ((column * 2 * fontScale)
						+ (row * 2 * fontW * fontScale));
				if (font <= 2)
					//pixOn = (font4x6[character][row]) & (1 << column);
					pixOn = (pgm_read_byte(font4x6+(character*6)+row)) & (1 << column);
				else if (font >= 7)
					//pixOn = (font8x16[character][row]) & (1 << column);
					pixOn = (pgm_read_byte(font8x16+(character*16)+row)) & (1 << column);
				else if (font >= 5)
					//pixOn = (font8x12[character][row]) & (1 << column);
					pixOn = (pgm_read_byte(font8x12+(character*13)+row)) & (1 << column);
				else
					//pixOn = (font6x8[character][column]) & (1 << row);
					pixOn = pgm_read_byte(font6x8+(character*6)+column) & (1<<row);
				if (pixOn) {
					for (rpt = 0; rpt < 4; rpt++) { // one pixel or a 2x2 "doubled" pixel
						cbuf[xx[rpt] + topleft] = fgcolor_hi;
						cbuf[xx[rpt] + 1 + topleft] = fgcolor_lo;
					}
				} else {
					for (rpt = 0; rpt < 4; rpt++) {
						cbuf[xx[rpt] + topleft] = bgcolor_hi;
						cbuf[xx[rpt] + 1 + topleft] = bgcolor_lo;
					}
				}
			}
		}
		tft144_write_data(cbuf, 0, fontW * fontH * 2);
	}

	free(cbuf);
}

void tft144_putstring(u8 *str, u8 originx, u8 y, colour fgcolor, colour bgcolor, u8 font) {
	uint8_t x, n;

	x = originx;
	fontW = fontDim[font][0];
	fontH = fontDim[font][1];
	for (n = 0; n < strlen(str); n++) {
		if ((x + fontW) > TFTWIDTH) {
			x = originx;
			y += (fontH);
		}
		if ((y + fontH) > TFTHEIGHT)
			break;
		tft144_putchar(str[n], x, y, fgcolor, bgcolor, font);
		x += (fontW);
	}
}
void tft144_draw_bmp(u8 *filename, u8 x0, u8 y0) {
	// ... os function
}

void tft144_invert_screen() {
	tft144_write_command(ENTER_INVERT_MODE);
}

void tft144_normal_screen() {
	tft144_write_command(EXIT_INVERT_MODE);
}

/********* EXTERNAL */
void aputchar(uint8_t x,uint8_t y,uint8_t c,colour charColor,colour bkColor)
{
  uint16_t i=0, j=0;
  
  uint8_t tmp_char=0;

  for (i=0;i<16;i++) {
    //tmp_char=ascii_8x16[((c-0x20)*16)+i]; // row sweep
	tmp_char=pgm_read_byte(ascii_8x16+((c-0x20)*16)+i);
    for (j=0;j<8;j++) {
      if ((tmp_char >> 7-j)&0x01)
        tft144_draw_dot(x+j,y+i,charColor);
      //else
       // tft144_draw_dot(x+j,y+i,bkColor);
    }
  }
}

/* added support to some basical text screens */
void lcd_putstring(uint8_t *x, uint8_t *y, char *str, colour Color, colour bColor) {
  //uint16_t ox=*x;
  while(*str) {
	if(*x > 120) { *y+=16; *x=0; } // (end of screen)
    if(*str == '\n') { *y+=16; *x=0; } // newline
    else if(*str == '\r') *x=0; // return
    else if(*str == '\b') *x-=8; // backspace
    else if(*str == '\v') *x = *y = 0; // start of screen
    else aputchar(*x,*y,*str,(Color),(bColor)), *x+=8;
    str++;
  }
}

