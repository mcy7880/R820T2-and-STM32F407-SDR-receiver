/*
 * usart.h - serial i/o routines for STM32F030F4
 * 05-30-15 E. Brombaugh
 */

#ifndef __usart__
#define __usart__

#include "main.h"

#define RX_BUFLEN 256
#define TX_BUFLEN 650

void usart_init(UART_HandleTypeDef *huartx_handle);
void usart_flush_RX_buffer();
int usart_getc(void);
void usart_putc(void* p, char c);

#endif
