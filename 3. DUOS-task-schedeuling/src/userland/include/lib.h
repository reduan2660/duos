#ifndef __LIB_H
#define __LIB_H

#include <bits/types.h>
#include <syscall_def.h>

typedef __uint8_t File_descriptor;

void write(File_descriptor fd, char *s);

#endif