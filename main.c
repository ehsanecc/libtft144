/* **************************************************
 * this is just an example to show how library works
 * this project is on atmega32 
 * pin connections(LCD->AVR):
 * VCC->+3.3
 * GND->GND
 * CS->PB0
 * RST->PB1
 * A0->PB2
 * SDA->MOSI
 * SCK->SCK
 * LED->VCC
 ************************************************** */

#include "tft144.h"

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
	lcd_putstring(&a, &b, "just sample text!", _RGB565(0,0,0), _RGB565(0,0,0));
	while(1) ;
}
