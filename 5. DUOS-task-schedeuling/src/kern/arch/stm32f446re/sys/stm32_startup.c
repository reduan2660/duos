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

#include <stm32_startup.h>
#include <sys.h>
#include <cm4.h>
#include <stm32_peps.h>
#include <kstdio.h>
#include <syscall.h>
#include <syscall_def.h>

void Reset_Handler(void)
{
	uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
	uint8_t *pDst = (uint8_t *)&_sdata;
	uint8_t *pSrc = (uint8_t *)&_la_data;
	for (uint32_t i = 0; i < size; i++)
	{
		*pDst++ = *pSrc++;
	}
	size = (uint32_t)&_ebss - (uint32_t)&_sbss;
	pDst = (uint8_t *)&_sbss;
	for (uint32_t i = 0; i < size; i++)
	{
		*pDst++ = 0;
	}
	_text_size = (uint32_t)&_etext - (uint32_t)&_stext;
	_data_size = (uint32_t)&_edata - (uint32_t)&_sdata;
	_bss_size = (uint32_t)&_ebss - (uint32_t)&_sbss;
	kmain();
}
void Default_Handler(void)
{
	while (1)
		;
}
// 2. implement the fault handlers
void HardFault_Handler(void)
{
	kprintf("Hardfault Exception Triggered\n");
	// kprintf("System Restarting....\n");
	// reboot();
	// kprintf("System Restart Failed.\n");
	while (1);
}

void MemManage_Handler(void)
{
	//	printf("Exception : MemManage\n");
	while (1)
		;
}

void BusFault_Handler(void)
{
	//	printf("Exception : BusFault\n");
	while (1)
		;
}

void SVCall_Handler(void)
{
	// kprintf("inside SVCall_Handler\n");

	__asm volatile(
		"TST lr, #4\n"
		"ITE EQ\n"
		"MRSEQ r0, MSP\n"
		"MRSNE r0, PSP\n"
		"B SVC_Handler_Main");

	kprintf("SVCall_Handler returned\n");
}

void SVC_Handler_Main(uint32_t *svc_args)
{
	// kprintf("in - SVC_Handler_Main\n");
	syscall(svc_args);
	// kprintf("SVC_Handler_Main - out\n");
}

void EXTI0_Handler(void)
{
	if (EXTI->PR & (1 << 0)) // If the PA0 triggered the interrupt
	{
		EXTI->PR |= (1 << 0); // Clear the interrupt flag by writing a 1
		kprintf("Button pressed\n");
	}
}

// void SysTick_Handler(void)
// {
// 	__mscount+=(SYSTICK->LOAD)/(PLL_N*1000);
// 	kprintf("Exception : Systick Exception Triggered - ");
// 	kprintf("%d\n", __getSysTickCount());
// }
