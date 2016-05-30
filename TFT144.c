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
#include <ctype.h>

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

uint8_t tft144_textW(uint8_t x, uint8_t font) {
	return 0;
}

uint8_t tft144_textH(uint8_t y, uint8_t font) {
	return 0;
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

void tft144_set_frame(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2) {
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

void tft144_draw_dot(uint8_t x, uint8_t y, colour color) {
	uint8_t color_hi = color>>8, color_lo=color&0xff;

	tft144_set_frame(x, x+1, y, y+1);
	tft144_write_command(WRITE_MEMORY_START);
	TFT_CTRL_PORT |= (1<<TFT_DC); // high(data)
	spi_write(color_hi);
	spi_write(color_lo);
}

/* this function needs to read pixel and mix with current color with factor of alpha
 * but as read from LCD is not implemented so it's not possible!
 */
void tft144_draw_dot_alpha(uint8_t x, uint8_t y, colour color, float alpha) {
	uint8_t r,g,b;

	// 00000000
	// BBBBBGGGGGGRRRRR
	// extract rgb
	r=(color&0x1f)<<3;
	g=(color&(0x3f<<5))>>3;
	b=(color&(0x1f<<11))>>8;

	// multiply by brightness ( 0 < brightness < 1 )
	r=(uint8_t)(((float)r)*alpha);
	g=(uint8_t)(((float)g)*alpha);
	b=(uint8_t)(((float)b)*alpha);

	tft144_draw_dot(x,y,RGB565(r,g,b));
}

/* Xialin Wu's alias line algorithm
 *
 * This function needs READ from LCD for alpha channeling... but unfortunately
 * i didn't implement read operation for LCD yet, because i don't know how! :(
 */

#define _ipart(x)		((int)(x))
#define _round(x)		(_ipart(x)+0.5f)
#define _fpart(x)		((x)<0?1-((x)-floor(x)):(x)-floor(x))
#define _rfpart(x)		(1-_fpart(x))
#define _swap(a,b,c)	{c=a;a=b;b=c;}

void tft144_draw_line_aliased(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, colour color) {
	int16_t steep = abs(y1 - y0) > abs(x1 - x0), c, x, xpxl1, ypxl1, xpxl2,
			ypxl2, xend, yend, xgap;
	float dx, dy, gradient, intery;

	if (steep) {
		_swap(x0, y0, c);
		_swap(x1, y1, c);
	}
	if (x0 > x1) {
		_swap(x0, x1, c);
		_swap(y0, y1, c);
	}

	dx = x1 - x0;
	dy = y1 - y0;
	gradient = dy / dx;

	// handle first endpoint
	xend = _round(x0);
	yend = y0 + gradient * (xend - x0);
	xgap = _rfpart(x0 + 0.5f);
	xpxl1 = xend; // this will be used in the main loop
	ypxl1 = _ipart(yend);
	if (steep) {
		tft144_draw_dot_alpha(ypxl1, xpxl1, color, _rfpart(yend) * xgap);
		tft144_draw_dot_alpha(ypxl1 + 1, xpxl1, color, _fpart(yend) * xgap);
	} else {
		tft144_draw_dot_alpha(xpxl1, ypxl1, color, _rfpart(yend) * xgap);
		tft144_draw_dot_alpha(xpxl1, ypxl1 + 1, color, _fpart(yend) * xgap);
	}
	intery = yend + gradient; // first y-intersection for the main loop

	// handle second endpoint
	xend = _round(x1);
	yend = y1 + gradient * (xend - x1);
	xgap = _rfpart(x1 + 0.5f);
	xpxl2 = xend;
	//this will be used in the main loop
	ypxl2 = _ipart(yend);
	if (steep) {
		tft144_draw_dot_alpha(ypxl2, xpxl2, color, _rfpart(yend) * xgap);
		tft144_draw_dot_alpha(ypxl2 + 1, xpxl2, color, _fpart(yend) * xgap);
	} else {
		tft144_draw_dot_alpha(xpxl2, ypxl2, color, _rfpart(yend) * xgap);
		tft144_draw_dot_alpha(xpxl2, ypxl2 + 1, color, _fpart(yend) * xgap);
	}

	// main loop
	for (x = xpxl1 + 1; x < (xpxl2 - 1); x++) {
		if (steep) {
			tft144_draw_dot_alpha(_ipart(intery), x, color, _rfpart(intery));
			tft144_draw_dot_alpha(_ipart(intery) + 1, x, color, _fpart(intery));
		} else {
			tft144_draw_dot_alpha(x, _ipart(intery), color, _rfpart(intery));
			tft144_draw_dot_alpha(x, _ipart(intery) + 1, color, _fpart(intery));
		}
		intery = intery + gradient;
	}
}

void tft144_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, colour color) {
	int16_t dx, dy, stepy, stepx, fraction;

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

/* four-point bezier curve drawing */
void tft144_draw_bezier(uint8_t x[4], uint8_t y[4], colour color) {
	int i;
	float t, xt, yt;

	for (t = 0.0f; t < 1.0f; t += 0.001f) {
		xt = pow(1 - t, 3) * x[0] + 3 * t * pow(1 - t, 2) * x[1]
				+ 3 * pow(t, 2) * (1 - t) * x[2] + pow(t, 3) * x[3];

		yt = pow(1 - t, 3) * y[0] + 3 * t * pow(1 - t, 2) * y[1]
				+ 3 * pow(t, 2) * (1 - t) * y[2] + pow(t, 3) * y[3];

		tft144_draw_dot(xt, yt, color);
	}
}

void tft144_draw_rectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, colour color) {
	tft144_draw_line(x0, y0, x0, y1, color);
	tft144_draw_line(x0, y1, x1, y1, color);
	tft144_draw_line(x1, y0, x1, y1, color);
	tft144_draw_line(x0, y0, x1, y0, color);
}

void tft144_draw_filled_rectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, colour color) {
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

void tft144_draw_circle(uint8_t x0, uint8_t y0, uint8_t radio, colour color) {
	int16_t error, errorx, errory, y, x;

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

/* this function seems a little bit slow(for it's random dot filling technique) but i found its really effective
   when you have no background color! and write your character on your picture!
   it returns the width printed (useful when your font is too heavy with spaces!) */
uint8_t tft144_putchar(char c, uint8_t x, uint8_t y, colour fgcolor, colour bgcolor, uint8_t font, uint8_t w, uint8_t h, uint8_t offset) {
	uint8_t i, j, o, k, fcs, tmp_char, cw=0;

	k = (((h-1)/8)+1);
	fcs = k*w; // font character size
	for(i=0;i<fcs;i+=k)
		for(o=0;o<k;o++) {
			tmp_char=pgm_read_byte(pgm_read_word(&fontlib[font])+6+((((c-offset)*fcs)+i+o)*2));
			for(j=0;j<8;j++)
				if((tmp_char >> j)&1) {
					tft144_draw_dot(x+(i/k), y+j+(o*8),fgcolor);
					if((i/k)>cw) cw=(i/k);
				}
		}

	return cw;
}

/* x & y are pointers to global(or local) variables that caller gives.
   the idea behind this is to give the caller a screen, that can write multiple times with just
   one x & y initialization.
   you can see in this function x & y updates after every character process. */
void tft144_putstring(char *str, uint8_t *x, uint8_t *y, colour fgcolor, colour bgcolor, uint8_t font) {
	uint8_t cw,w,h,offset;

	w=pgm_read_byte(pgm_read_word(&fontlib[font]));
	h=pgm_read_byte(pgm_read_word(&fontlib[font])+2);
	offset=pgm_read_byte(pgm_read_word(&fontlib[font])+4);
	while (*str) {
		if (*x > (TFTWIDTH - w)) {
			*y += h;
			*x = 0;
		} // (end of screen)
		if (*str == '\n') {
			*y += h;
			*x = 0;
		} // newline
		else if (*str == '\r')
			*x = 0; // return
		else if (*str == '\b')
			*x -= w; // backspace
		else if (*str == '\v')
			*x = *y = 0; // start of screen
		else if ((*str >= offset) && (isprint(*str))) {
			cw = tft144_putchar(*str, *x, *y, fgcolor, bgcolor, font, w, h,
					offset);
			*x += (cw ? cw + 2 : w / 2);
		}

		str++;
	}
}

void tft144_draw_bmp(uint8_t *filename, uint8_t x0, uint8_t y0) {
	// ... not implemented function
}

void tft144_invert_screen() {
	tft144_write_command(ENTER_INVERT_MODE);
}

void tft144_normal_screen() {
	tft144_write_command(EXIT_INVERT_MODE);
}

