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

#include <syscall.h>
#include <syscall_def.h>
#include <errno.h>
#include <errmsg.h>

#include <kunistd.h>
#include <usart.h>
#include <types.h>
#include <cm4.h>
#include <stdint.h>
#include <schedule.h>
void syscall(uint32_t *svc_args)
{
	int callno = ((char *)svc_args[6])[-2];
	kprintf("Callno: %d\n", callno);
	uint32_t psp;
	uint32_t stacked_r0, stacked_r1, stacked_r2, stacked_r3, pid;

	stacked_r0 = svc_args[0];
	stacked_r1 = svc_args[1];
	stacked_r2 = svc_args[2];
	stacked_r3 = svc_args[3];
	pid = svc_args[10];

	TCB_TypeDef* task;

	switch (callno)
	{

	case SYS_read:
		break;
	case SYS_write:
		__sys_write(stacked_r3);
		break;
	case SYS_reboot:
		break;
	case SYS__exit:
		task = svc_args[16];
		task->status = KILLED;

		break;
	case SYS_getpid:
		// uint32_t pid = svc_args[10];
		// pid = svc_args[10];
		task = svc_args[16];
		__sys_getpid((unsigned int *)pid,task->task_id);
		break;
	case SYS___time:
		break;
	case SYS_yield:
		SCB->ICSR |= (1 << 28); // set PendSV bit
		break;
	case SYS_start:
		psp = (uint32_t)svc_args[0];
		__sys_start_task(psp);
		break;
	/* return error code see error.h and errmsg.h ENOSYS sys_errlist[ENOSYS]*/
	default:
		break;
	}
	/* Handle SVC return here */
}
