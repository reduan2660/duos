#ifndef __A9G_H
#define __A9G_H
#include <stdint.h>
#include <stdarg.h>

#define A9G_BUFFER_SIZE 512
typedef struct
{
    unsigned char buffer[A9G_BUFFER_SIZE];
    volatile unsigned int head;
    volatile unsigned int tail;
} ring_buffer;

void a9g_send_at(char *, ...);
void store_char(unsigned char c, ring_buffer *buffer);
void a9g_init(void);
int a9g_data_available(void);
int a9g_peek(void);
int a9g_read(void);
void a9g_buffer_init(void);
void a9g_flush(void);
int a9G_wait_for(char *string);

#endif /* A9G */
