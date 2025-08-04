/*
 * usart.c - serial i/o routines for STM32F030F4
 * 09-04-16 E. Brombaugh
 * 02-08-2025 - Maciej Fajfer - some minor modifications
 */

#include <stdio.h>
#include "usart.h"
#include "main.h"

/* rx buffer */
uint8_t RX_buffer[RX_BUFLEN];
uint8_t *RX_wptr, *RX_rptr;

/* tx buffer */
uint8_t TX_buffer[TX_BUFLEN];
uint8_t *TX_wptr, *TX_rptr;

UART_HandleTypeDef *huartx;
extern uint8_t txchar;

/* USART setup */
void usart_init(UART_HandleTypeDef *huartx_pointer)
{
	huartx = huartx_pointer;

	/* init RX buffer write/read pointers*/
	RX_wptr = &RX_buffer[0];
	RX_rptr = &RX_buffer[0];

	/* init TX buffer */
	TX_wptr = &TX_buffer[0];
	TX_rptr = &TX_buffer[0];
}

//USART flush RX buffer
void usart_flush_RX_buffer()
{
	/* reinit RX buffer write/read pointers*/
	RX_wptr = &RX_buffer[0];
	RX_rptr = &RX_buffer[0];
}

int usart_getc(void)
{
	/* interrupt version */
	int retval;
	
	/* check if there's data in the buffer */
	if(RX_rptr != RX_wptr)
	{
		/* get the data */
		retval = *RX_rptr++;
		
		/* wrap the pointer */
		if((RX_rptr - &RX_buffer[0])>=RX_BUFLEN)
			RX_rptr = &RX_buffer[0];
	}
	else
		retval = EOF;

	return retval;
}

/*
 * output for tiny printf
 */
void usart_putc(void* p, char c)
{
	/* interrupt version */
	/* check if there's room in the buffer */
	if((TX_wptr != TX_rptr-1) &&
	   (TX_wptr - TX_rptr != (TX_BUFLEN-1)))
	{
		/* Yes - Queue the new char & enable IRQ */

		if (HAL_UART_GetState(huartx) == HAL_UART_STATE_BUSY_TX_RX)
		{
			*TX_wptr++ = c; //Queue the new char if there's transmission in progress
		}
		else
		{
			txchar = c;
			HAL_UART_Transmit_IT(huartx, &txchar, 1); //if not just send it
		}

		/* Wrap pointer */
		if((TX_wptr - &TX_buffer[0])>=TX_BUFLEN)
			TX_wptr = &TX_buffer[0];
	}
}

