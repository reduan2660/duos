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

#include <sys_init.h>
#include <cm4.h>
#include <kmain.h>
#include <kstdio.h>
#include <kstring.h>
#include <stdint.h>
#include <usart.h>
#include "../dev/include/se7en_segment.h"
#include <sys.h>
#include <test_interrupt.h>
#include <stm32_peps.h>
#include <types.h>
#include <schedule.h>

// userland
#include <main.h>
#include <ustd.h>
#include <schedule.h>
#include <sem.h>

/* Task Stuff */
// #define STOP 100000
#define STOP 100000
#define TASK_COUNT 10
TCB_TypeDef __task[MAX_TASKS], __sleep;
uint32_t GLOBAL_COUNT = 0;

void task(void)
{

	uint32_t value;
	uint32_t inc_count = 0;
	uint32_t pid = getpid();
	kprintf("------Task %d----------\n", pid - 1);
	while (1)
	{
		value = GLOBAL_COUNT;
		value++;
		if (value != GLOBAL_COUNT + 1)
		{																			   // we check is someother task(s) increase the count
			kprintf("Error in task %d -- %d != %d\n\r", pid, value, GLOBAL_COUNT + 1); /* It is an SVC call*/
		}
		else
		{
			// kprintf(".");
			GLOBAL_COUNT = value;
			inc_count++;
		}

		if (GLOBAL_COUNT >= STOP)
		{
			kprintf("Total increment done by task %d is: %d\n\r", pid, inc_count);

			break;
		}
	}
	task_exit();
}

void task_sem(void)
{
	uint32_t pid = getpid();
	uint32_t value, inc_count1 = 0;

	down();
	kprintf("----------Task %d ----------: \n\r", pid);
	up();
	while (1)
	{
		down();
		value = GLOBAL_COUNT;
		value++;
		uint8_t is_valid = (value != GLOBAL_COUNT + 1);
		up();
		if (is_valid)
		{
			down();
			kprintf("Error in pid %d with %d != %d\n\r", pid, value, GLOBAL_COUNT + 1);
			up();
		}
		else
		{
			down();
			GLOBAL_COUNT = value;
			inc_count1++;
			up();
		}
		down();
		is_valid = (GLOBAL_COUNT >= STOP);
		up();
		if (is_valid)
		{
			down();
			kprintf("Total increment done by task %d is: %d\n\r", pid, inc_count1);
			up();
			break;
		}
	}
	task_exit();
}

void sleep_state(void)
{
	// kprintf("Hello from sleep\n");
	__set_pending(0);
	print_stats();
	kprintf("Sleeping\n");
	while (1)
		;
}

void print_stats(void)
{

	uint32_t task_policy_count = 0, avg_response_time = 0, avg_waiting_time = 0, avg_turnaround_time = 0, avg_execution_time = 0;

	kprintf("ID\tStart-Time\tResponse-Time\tWaiting-Time\tExecution-Time\tTurn Around Time\n");
	for (int i = 0; i < TASK_COUNT; i++)
	{
		TCB_TypeDef *task = (TCB_TypeDef *)__task + i;

		kprintf("%d\t%d\t\t%d\t\t%d\t\t%d\t\t%d\n",
				task->task_id, task->start_time_t * 1, task->response_time_t * 1, task->waiting_time * 1, task->execution_time * 1, task->execution_time * 1 + task->waiting_time * 1);

		if (task->waiting_time < 10000)
		{
			task_policy_count++;
			avg_response_time += task->response_time_t * 1;
			avg_waiting_time += task->waiting_time * 1;
			avg_turnaround_time += task->execution_time * 1 + task->execution_time * 1;
			avg_execution_time += task->execution_time * 1;
		}
	}

	avg_execution_time /= task_policy_count;
	avg_response_time /= task_policy_count;
	avg_waiting_time /= task_policy_count;
	avg_turnaround_time /= task_policy_count;

	kprintf("\n\nAvg-Response\tAvg-Waiting\tAvg-Execution\tAvg-Turnaround\n");
	kprintf("%d\t\t%d\t\t%d\t\t%d\n", avg_response_time, avg_waiting_time, avg_execution_time, avg_turnaround_time);
}

void SVC_Init(void)
{
	uint32_t psp_stack[1024];
	PSP_Init(psp_stack + 1024);
	__asm volatile(
		".global PSP_Init\n"
		"PSP_Init:\n"
		"msr psp, r0\n"
		"mov r0, #3\n"
		"msr control, r0\n"
		"isb\n");
}

void round_robin_no_semaphore()
{
	kprintf("------Round Robin - No Semaphore ----------\n");
	SCHEDEULING_ALGORITHM = ROUND_ROBIN;
	init_queue();
	for (int i = 0; i < TASK_COUNT; i++)
	{
		__create_task(__task + i, task, (uint32_t *)TASK_STACK_START - i * TASK_STACK_SIZE);
		queue_add(__task + i);
	}
	__create_task(&__sleep, sleep_state, (uint32_t *)TASK_STACK_START - TASK_COUNT * TASK_STACK_SIZE);

	__set_sleep(&__sleep);
	__set_pending(1);
	__start_task();
}

void round_robin_semaphore()
{
	kprintf("------Round Robin - Semaphore ----------\n");
	SCHEDEULING_ALGORITHM = ROUND_ROBIN;
	init_queue();
	for (int i = 0; i < TASK_COUNT; i++)
	{
		__create_task(__task + i, task_sem, (uint32_t *)TASK_STACK_START - i * TASK_STACK_SIZE);
		queue_add(__task + i);
	}
	__set_sleep(&__sleep);
	__create_task(&__sleep, sleep_state, (uint32_t *)TASK_STACK_START - TASK_COUNT * TASK_STACK_SIZE);

	__set_pending(1);
	__start_task();
}

void fcfs_no_semaphore()
{
	kprintf("------FCFS - No Semaphore ----------\n");
	SCHEDEULING_ALGORITHM = FIRST_COME_FIRST_SERVE;
	init_queue();
	for (int i = 0; i < TASK_COUNT; i++)
	{
		__create_task(__task + i, task, (uint32_t *)TASK_STACK_START - i * TASK_STACK_SIZE);
		queue_add(__task + i);
	}
	__create_task(&__sleep, sleep_state, (uint32_t *)TASK_STACK_START - TASK_COUNT * TASK_STACK_SIZE);

	__set_sleep(&__sleep);
	__set_pending(1);
	__start_task();
}

void fcfs_semaphore()
{
	kprintf("------FCFS - Semaphore ----------\n");
	SCHEDEULING_ALGORITHM = FIRST_COME_FIRST_SERVE;
	init_queue();
	for (int i = 0; i < TASK_COUNT; i++)
	{
		__create_task(__task + i, task_sem, (uint32_t *)TASK_STACK_START - i * TASK_STACK_SIZE);
		queue_add(__task + i);
	}
	__create_task(&__sleep, sleep_state, (uint32_t *)TASK_STACK_START - TASK_COUNT * TASK_STACK_SIZE);

	__set_sleep(&__sleep);
	__set_pending(1);
	__start_task();
}

void kmain(void)
{
	__sys_init();
	__SysTick_init(); // 10ms

	__NVIC_SetPriority(SVCall_IRQn, 1);
	__NVIC_SetPriority(SysTick_IRQn, 0x2);
	__NVIC_SetPriority(PendSV_IRQn, 0xFF);

	SVC_Init();

	// round_robin_no_semaphore();
	// round_robin_semaphore();
	// fcfs_no_semaphore();
	fcfs_semaphore();

	while (1)
		;
}
