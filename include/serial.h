#ifndef SERIAL_H_
#define SERIAL_H_
#include <stdint.h>

void delay_ms(uint32_t sleep_ms);
void xSerialPutChar(uint8_t data);
void xSerialToRPC(void);
void SET_LED(int ledstatus);

#endif /*SERIAL_H_*/
