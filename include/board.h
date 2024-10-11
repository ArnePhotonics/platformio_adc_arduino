#ifndef BOARD_H_
#define BOARD_H_

#include <avr/io.h>
#include <avr/interrupt.h>

// ADC

#define PIN_TCM (1 << 0) // out
#define PIN_TCP (1 << 1) // out

#define PIN_LED_L (1 << 6) // out
#define SET_LED_L() (PORTA |= PIN_LED_L)

#define CLEAR_LED_L() (PORTA &= ~PIN_LED_L)
#endif /*BOARD_H_*/
