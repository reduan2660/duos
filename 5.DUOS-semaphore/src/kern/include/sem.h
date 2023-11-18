#ifndef __SEM_H
#define __SEM_H
#include <stdint.h>
#include <kstdio.h>
#include <schedule.h>
#include <kstring.h>
#include <types.h>
#include <types.h>

typedef uint32_t semaphore;

// volatile semaphore mutex = 1;
// volatile ReadyQ_TypeDef sem_queue;

extern void sem_dec(semaphore *semaphore);
extern void sem_inc(semaphore *semaphore);
void down(void);
void up(void);

void push_sem(TCB_TypeDef *task);
TCB_TypeDef *pop_sem(void);
int is_sem_empty(void);

extern volatile semaphore lock_sem;
#endif