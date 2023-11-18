#include "../include/lib.h"

void write(File_descriptor fd, char *s)
{
    // stack the arguments
    __uint32_t args[3];

    // first argument is syscall number
    args[0] = SYS_write;
    args[1] = (__uint32_t)fd;
    args[2] = (__uint32_t)s;
    
    // call the syscall
    syscall(SYS_write);
}