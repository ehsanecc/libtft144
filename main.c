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
#include <stdio.h>
#include <avr/pgmspace.h>

void uart_putchar(char c);
char uart_getchar();

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL,uart_getchar, _FDEV_SETUP_READ);

void uart_putchar(char c) {
	loop_until_bit_is_set(UCSRA,UDRE);
    UDR = c;
    loop_until_bit_is_set(UCSRA, TXC); /* Wait until transmission ready. */
}

char uart_getchar() {
	loop_until_bit_is_set(UCSRA,RXC);
	return UDR;
}

void USART_Init() {
	UCSRA=0x00;
	UCSRB=0x18;
	UCSRC=0x86;
	UBRRH=0x00;
	UBRRL=0x05;
}

void init() {
	PORTB = (1 << 4);
	DDRB = (1 << 7) | (1 << 5) | (1 << 4) | 1; // (MOSI,SCK,SS) should be output

	USART_Init(115200);
	stdout = &uart_output;
	stdin = &uart_input;
	printf("initializing system...\r\n");
	tft144_reset();
	tft144_init();
}

int main() {
	u8 a,b,str[2]={0,0};
	float v=0;
	uint8_t x[4]={0,0,60,100},y[4]={0,40,30,90};

	init();
	for(a=0;a<128;a++)
		for(b=0;b<128;b++)
			tft144_draw_dot(a,b,RGB565(a*a,b*b,a*b));
	_delay_ms(1000);
	tft144_clear_display(BLACK);
	a=0; b=0;
	tft144_putstring("abcdefghijklmnopqrstuvwxyz",&a,&b,WHITE,BLACK,0);
	tft144_putstring("abcdefghijklmnopqrstuvwxyz",&a,&b,WHITE,BLACK,1);
	tft144_putstring("abcdefghijklmnopqrstuvwxyz",&a,&b,WHITE,BLACK,2);
	_delay_ms(5000);
	tft144_clear_display(BLACK);
	a=0;b=0;
	tft144_putstring("UART Sniffer\nBaud 115200\n",&a,&b,WHITE,BLACK,1);
	tft144_draw_line(0,16,127,16,RED);
	while(1) {
		/***********************************************************************
		 * Example of reading pixels from UART and write them to 128x128 LCD :
		tft144_set_frame(0, 127, 0, 127);
		tft144_write_command(WRITE_MEMORY_START);
		TFT_CTRL_PORT |= (1<<TFT_DC); // high(data)
		for(af=0;af<16384;af++) {
			loop_until_bit_is_set(UCSRA,RXC);
			bb = UDR;
			loop_until_bit_is_set(UCSRA,RXC);
			spi_write(UDR);
			spi_write(bb);
		}
		************************************************************************/
		str[0] = getchar();
		tft144_putstring((char*)str,&a,&b,GREEN,BLACK,2);
		if(b >= 128) {
			tft144_clear_display(BLACK);
			a=0;b=0;
			tft144_putstring("UART Sniffer\nBaud 115200\n",&a,&b,WHITE,BLACK,1);
			tft144_draw_line(0,16,127,16,RED);
		}
	}
}

