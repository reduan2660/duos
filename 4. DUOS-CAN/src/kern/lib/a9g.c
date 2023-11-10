#include <a9g.h>
#include <stm32_peps.h>
#include <usart.h>
#include <kstring.h>
#include <kstdio.h>
#include <string.h>

/**
 * first argument define the type of string to kprintf and kscanf,
 * %c for charater
 * %s for string,
 * %d for integer
 * %x hexadecimal
 * %o octal number
 * %f for floating point number
 */
// Simplified version of printf
void a9g_send_at(char *format, ...)
{
	kprintf("Sending AT command: %s", format);
	char *tr;
	uint32_t i;
	uint8_t *str;
	va_list list;
	double dval;
	// uint32_t *intval;
	va_start(list, format);

	for (tr = format; *tr != '\0'; tr++)
	{
		while (*tr != '%' && *tr != '\0')
		{
			UART_SendChar(USART4, *tr);
			tr++;
		}
		if (*tr == '\0')
			break;
		tr++;
		switch (*tr)
		{
		case 'c':
			i = va_arg(list, int);
			UART_SendChar(UART4, i);
			break;
		case 'd':
			i = va_arg(list, int);
			if (i < 0)
			{
				UART_SendChar(UART4, '-');
				i = -i;
			}
			_USART_WRITE(UART4, (uint8_t *)convert(i, 10));
			break;
		case 'o':
			i = va_arg(list, int);
			if (i < 0)
			{
				UART_SendChar(UART4, '-');
				i = -i;
			}
			_USART_WRITE(UART4, (uint8_t *)convert(i, 8));
			break;
		case 'x':
			i = va_arg(list, int);
			if (i < 0)
			{
				UART_SendChar(UART4, '-');
				i = -i;
			}
			_USART_WRITE(UART4, (uint8_t *)convert(i, 16));
			break;
		case 's':
			str = va_arg(list, uint8_t *);
			_USART_WRITE(UART4, str);
			break;
		case 'f':
			dval = va_arg(list, double);
			_USART_WRITE(UART4, (uint8_t *)float2str(dval));
			break;
		default:
			break;
		}
	}

	va_end(list);
}

static ring_buffer rx_buffer1 = {{0}, 0, 0};
static ring_buffer *_rx_buffer1;

void a9g_init(void)
{
	kprintf("Connecting to A9g\n");
	pseudo_delay(10000000);
	a9g_buffer_init();
	a9g_flush();
	a9g_send_at("AT+RST=1\r\n");
	// pseudo_delay(10000000);
	while (!(a9G_wait_for("OK\r\n")))
		;
	kprintf("Connected.\n");

	// kprintf("Turnign on GPS");
	// a9g_send_at("AT+GPS=1\r\n");
	// while (!(a9G_wait_for("OK\r\n")))
	// 	;
	// pseudo_delay(10000000);

	kprintf("\nMQTT Initilization\n");
	a9g_send_at("AT+CREG=1\r\n");
	while (!(a9G_wait_for("OK\r\n")))
		;
	// pseudo_delay(10000000);

	a9g_send_at("AT+CGATT=1\r\n");
	while (!(a9G_wait_for("OK\r\n")))
		;
	// pseudo_delay(10000000);

	a9g_send_at("AT+CGDCONT=1,\"IP\",\"blweb\"\r\n");
	while (!(a9G_wait_for("OK\r\n")))
		;
	// pseudo_delay(10000000);

	a9g_send_at("AT+CGACT=1,1\r\n");
	while (!(a9G_wait_for("OK\r\n")))
		;
	// pseudo_delay(10000000);

	a9g_send_at("AT+MQTTCONN=\"103.221.252.141\",1883,\"ecu\",120,0,\"test\",\"test\"\r\n");
	while (!(a9G_wait_for("OK\r\n")))
		;
	// pseudo_delay(10000000);

	kprintf("\nMQTT Connected\n");

	a9g_send_at("AT+MQTTCONN=\"103.221.252.141\",1883,\"ecu/1/req\",120,0,\"test\",\"test\"\r\n");
	while (!(a9G_wait_for("OK\r\n")))
		;
	kprintf("Subbed to ecu/1/req\r\n");
}

void a9g_buffer_init(void)
{
	_rx_buffer1 = &rx_buffer1;
}

void store_char(unsigned char c, ring_buffer *buffer)
{
	// kprintf("Storing char: %c\n", c);
	unsigned int i = (unsigned int)(buffer->head + 1) % A9G_BUFFER_SIZE;
	if (i != buffer->tail)
	{
		buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}

int a9g_data_available(void)
{
	int a = (uint16_t)(A9G_BUFFER_SIZE + _rx_buffer1->head - _rx_buffer1->tail) % A9G_BUFFER_SIZE;
	return a;
	// return (uint16_t)(A9G_BUFFER_SIZE + _rx_buffer1->head - _rx_buffer1->tail) % A9G_BUFFER_SIZE;
}

int a9g_peek(void)
{

	if (_rx_buffer1->head == _rx_buffer1->tail)
	{
		return -1;
	}
	else
	{
		return _rx_buffer1->buffer[_rx_buffer1->tail];
	}
}

int a9g_read(void)
{

	if (_rx_buffer1->head == _rx_buffer1->tail)
	{
		return -1;
	}
	else
	{
		unsigned char c = _rx_buffer1->buffer[_rx_buffer1->tail];
		_rx_buffer1->tail = (unsigned int)(_rx_buffer1->tail + 1) % A9G_BUFFER_SIZE;
		return c;
	}
}

void a9g_flush(void)
{
	// memset(_rx_buffer1->buffer, '\0', A9G_BUFFER_SIZE);

	for (int i = 0; i < A9G_BUFFER_SIZE; i++)
	{
		_rx_buffer1->buffer[i] = '\0';
	}

	_rx_buffer1->head = 0;
	_rx_buffer1->tail = 0;
}

int a9G_wait_for(char *string)
{
	int so_far = 0;
	int len = (int)__strlen(string);

again_device:

	while (!a9g_data_available())
		;

	if (a9g_peek() != string[so_far])
	{
		_rx_buffer1->tail = (unsigned int)(_rx_buffer1->tail + 1) % A9G_BUFFER_SIZE;
		goto again_device;
	}
	while (a9g_peek() == string[so_far])
	{
		so_far++;
		a9g_read();
		if (so_far == len)
			return 1;
		while (!a9g_data_available())
			;
	}

	if (so_far != len)
	{
		so_far = 0;
		goto again_device;
	}

	if (so_far == len)
		return 1;
	else
		return -1;
}

void UART4_Handler(void)
{

	if (USART4->SR & USART_SR_RXNE)
	{
		char c = UART_GetChar(USART4);
		store_char(c, _rx_buffer1);
	}
}