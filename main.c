#include "tft144.h"
//#include <avr/io.h>
//#include <util/delay.h>

int main() {
	int a,b;
	
	PORTB = (1<<4);
	DDRB = (1<<7)|(1<<5)|(1<<4)|1; // (MOSI,SCK,SS) should be output
	
	tft144_reset();
	tft144_init();
	a=0, b=0;
	for(a=0;a<128;a++)
		for(b=0;b<128;b++)
			tft144_draw_dot(a,b,RGB565(a*a,b*b,a*b));
	tft144_clear_display(_RGB565(255,255,255));
	a=0; b=64;
	lcd_putstring(&a, &b, "ehsan.ecc@gmail.com\n09120187769\n09364075229", _RGB565(0,0,0), _RGB565(0,0,0));
	while(1) ;
}