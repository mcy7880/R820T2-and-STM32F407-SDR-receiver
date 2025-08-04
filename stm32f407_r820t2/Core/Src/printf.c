//02-08-2025 - Maciej Fajfer

#include "printf.h"
#include "usart.h"

char UART_printf_tmp_buf[100];

void UART_printf(const char *format, ...)
{
   va_list args;
   va_start(args, format);
   vsprintf(UART_printf_tmp_buf, format, args);
   va_end(args);

   char *tmp_pointer = UART_printf_tmp_buf;
   while(*tmp_pointer != '\0') usart_putc(0, *tmp_pointer++);
}
