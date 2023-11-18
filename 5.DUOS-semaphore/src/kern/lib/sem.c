#include <sem.h>

void sem_dec(semaphore *_semaphore)
{
    asm volatile(
        ".macro WAIT_FOR_UPDATE         \n"
        "   WFI                         \n"
        ".endm                          \n");

    asm volatile(
        "1: LDREX   r1, [r0]            \n"
        "   CMP	    r1, #0              \n" // ; Test if semaphore holds the value 0
        "   BEQ     2f                  \n" // ; If it does, block before retrying
        "   SUB     r1, #1              \n" // ; If not, decrement temporary copy
        "   STREX   r2, r1, [r0]        \n" // ; Attempt Store-Exclusive
        "   CMP     r2, #0              \n" // ; Check if Store-Exclusive succeeded
        "   BNE     1b                  \n" // ; If Store-Exclusive failed, retry from start
        "   DMB                         \n" // ; Required before accessing protected resource
        "   B      3f                   \n"
        "2:                             \n" // ; Take appropriate action while waiting for semaphore to be incremented
        "   WAIT_FOR_UPDATE             \n"
        "   B       1b                  \n"
        "3:                             \n"
        : [r0] "=r"(_semaphore) :);
    return;
}

void sem_inc(semaphore *_semaphore)
{
    asm volatile(
        ".macro SIGNAL_UPDATE           \n"
        "    SEV                        \n"
        ".endm                          \n");
    asm volatile(
        "1:  LDREX   r1, [r0]           \n"
        "    ADD     r1, #1             \n" // ; Increment temporary copy
        "    STREX   r2, r1, [r0]       \n" // ; Attempt Store-Exclusive
        "    CMP     r2, #0             \n" // ; Check if Store-Exclusive succeeded
        "    BNE     1b                 \n" // ; Store failed - retry immediately
        "    CMP     r0, #1             \n" // ; Store successful - test if incremented from zero
        "    DMB                        \n" // ; Required before releasing protected resource
        "    BGE     2f                 \n" // ; If initial value was 0, signal update
        "    BX      lr                 \n"
        "2:                             \n" // ; Signal waiting processors or processes
        "    SIGNAL_UPDATE              \n"
        : [r0] "=r"(_semaphore) :);
}
