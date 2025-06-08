#ifndef USART_H
#define USART_H

#include <Arduino.h>

#define BAUD_RATE 9600 // Baud rate. The usart_init routine uses this.

#define RX_BUFFER_SIZE 64

void usart_clear(void);
void usart_init(void);
void usart_printf(const char *ptr);
void usart_putc(const char c);
void usart_prints(const char *ptr);
unsigned char usart_getc(void);
unsigned char uart_buffer_empty(void);
void usart_send_int(int value);

#endif /* USART_H */