/*
 * tft144_config.h
 *
 *  Created on: May 27, 2016
 *      Author: Ehsan
 *
 * Should change definitions for use with other boards. mine just fine!
 */

#ifndef TFT144_CONFIG_H_
#define TFT144_CONFIG_H_

#include <avr/io.h>

// constants
#define ORIENTATION0	0
#define ORIENTATION90 	96
#define ORIENTATION270	160
#define ORIENTATION180	192

// CONFIG:
#define REDBOARD		0 // is redboard?

#define TFT_CTRL_PORT		PORTB
#define TFT_CTRL_PIN		PINB
#define TFT_CTRL_DDR		DDRB
#define TFT_CE				0 // It's SPIs SS pin ( active low )
#define TFT_DC				2 // A0
#define TFT_RST				1
#define TFT_LED				3
#define ORIENTATION			ORIENTATION0

#endif /* TFT144_CONFIG_H_ */
