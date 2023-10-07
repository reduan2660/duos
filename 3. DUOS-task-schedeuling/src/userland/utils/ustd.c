/*
 * Copyright (c) 2022
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <ustd.h>
#include <stdarg.h>

#include <kstdio.h>
#include <usart.h>

static uint8_t __outbuf[50];

void printf(char *format, ...)
{
	kprintf("in - printf\n");

	// CAll SVC for sys_write | SYS_write => 55
	__asm volatile(
		"mov r3, %[x]\n"
		"svc #55\n"
		:
		: [x] "r"(format));

	kprintf("printf - out\n");
}

/*

uint8_t * convert(uint32_t x, uint8_t base)
{
	static uint8_t baseval [] = "0123456789ABCDEF";
	uint8_t *ptr;
	ptr = &__outbuf[49];
	*ptr = '\0';
	do
	{
		*--ptr = baseval[x%base];
		x /= base;
	} while (x != 0);
	return (ptr);
}


void printf(char *format, ...){
	// write your code here
	char *tr;
	uint32_t i, ind=0;
	uint8_t *str;
	va_list list;
	double dval;
	// uint32_t *intval;
	va_start(list, format);

	char str[100];

	for (tr = format; *tr != '\0'; tr++)
	{
		while (*tr != '%' && *tr != '\0')
		{
			str[ind] = *tr; ind++; // UART_SendChar(USART2, *tr);
			tr++;
		}
		if (*tr == '\0')
			break;
		tr++;
		switch (*tr)
		{
		case 'c':
			i = va_arg(list, int);
			str[ind] = i; ind++; // UART_SendChar(USART2, i);
			break;
		case 'd':
			i = va_arg(list, int);
			if (i < 0)
			{
				str[ind] = '-'; ind++; // UART_SendChar(USART2, '-');
				i = -i;
			}
			uint8_t *s = (uint8_t *)convert(i, 10);
			while(*s) str[ind] = *s++; ind++; // while (*s) UART_SendChar(usart,*s++); //_USART_WRITE(USART2, (uint8_t *)convert(i, 10));
			break;
		case 'o':
			i = va_arg(list, int);
			if (i < 0)
			{
				str[ind] = '-'; ind++; // UART_SendChar(USART2, '-');
				i = -i;
			}
			uint8_t *s = (uint8_t *)convert(i, 8);
			while(*s) str[ind] = *s++; ind++; // while (*s) UART_SendChar(usart,*s++); //_USART_WRITE(USART2, (uint8_t *)convert(i, 8));
			break;
		case 'x':
			i = va_arg(list, int);
			if (i < 0)
			{
				str[ind] = '-'; ind++; // UART_SendChar(USART2, '-');
				i = -i;
			}
			uint8_t *s = (uint8_t *)convert(i, 16);
			while(*s) str[ind] = *s++; ind++; // while (*s) UART_SendChar(usart,*s++); // _USART_WRITE(USART2, (uint8_t *)convert(i, 16));

			break;
		case 's':
			str = va_arg(list, uint8_t *);
			uint8_t *s = str;
			while(*s) str[ind] = *s++; ind++; // while (*s) UART_SendChar(usart,*s++); //_USART_WRITE(USART2, str);
			break;
		case 'f':
			dval = va_arg(list, double);
			uint8_t *s = (uint8_t *)float2str(dval);
			while(*s) str[ind] = *s++; ind++; // while (*s) UART_SendChar(usart,*s++); //_USART_WRITE(USART2, (uint8_t *)float2str(dval));
			break;
		default:
			break;
		}
	}

	va_end(list);
	str[ind] = '\0';
}

*/