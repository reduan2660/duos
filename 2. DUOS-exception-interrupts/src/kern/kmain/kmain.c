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

void GPIO_Config(void)
{
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************

	1. Enable GPIO Clock
	2. Set the required Pin in the INPUT Mode
	3. Configure the PULL UP/ PULL DOWN According to your requirement

	********************************************************/

	RCC->AHB1ENR |= (1 << 0); // Enable GPIOA clock
	GPIOA->PUPDR |= (1 << 0); // Bits (1:0) = 1:0  --> PA0 is in Pull Up mode
}

void Interrupt_Config(void)
{
	/*
	1. Enable the SYSCNFG bit in RCC register
	2. Configure the EXTI configuration Regiter in the SYSCNFG
	3. Enable the EXTI using Interrupt Mask Register (IMR)
	4. Configure the Rising Edge / Falling Edge Trigger
	5. Set the Interrupt Priority
	6. Enable the interrupt
	*/

	RCC->APB2ENR |= (1 << 14); // Enable SYSCNFG

	SYSCFG->EXTICR[0] &= ~(0xf << 0); // Bits[3:2:1:0] = (0:0:0:0)  -> configure EXTI0 line for PA0

	EXTI->IMR |= (1 << 0); // Bit[0] = 1  --> Disable the Mask on EXTI 0

	EXTI->RTSR |= (1 << 0); // Enable Rising Edge Trigger for PA0

	EXTI->FTSR &= ~(1 << 0); // Disable Falling Edge Trigger for PA0

	__NVIC_SetPriority(EXTI0_IRQn, 2); // Set Priority to 2
	__NVIC_EnableIRQn(EXTI0_IRQn);	   // Enable Interrupt
}

void kmain(void)
{
	__sys_init();
	__SysTick_init(16777215);

	GPIO_Config();
	Interrupt_Config();

	kprintf("Press 1 to trigger Hardfault\n");
	kprintf("Press 2 to enable Systick Interrupt\n");
	kprintf("Press 3 to disable Systick Interrupt\n");
	kprintf("Press 4 to set BASEPRI\n");
	kprintf("Press 0 to reboot\n");

	while (1)
	{

		int option, priority;
		;

		kprintf("Waiting for input\n");
		kscanf("%d", &option);

		switch (option)
		{
		case 1:
			kprintf("Triggering Hardfault\n");
			enable_hardfault_event();
			break;

		case 2:
			kprintf("Enabling Systick Interrupt\n");
			enableSysTickInterrupt();
			break;

		case 3:
			kprintf("Disabling Systick Interrupt\n");
			disableSysTickInterrupt();
			break;

		case 4:
			kprintf("Enter priority: "); // 0-15
			kscanf("%d", &priority);

			kprintf("Setting BASEPRI");
			__set_BASEPRI(priority);

			kprintf("BASEPRI VALUE: %d\n", get_basepri_value());
			break;

		case 0:
			kprintf("Rebooting\n");
			reboot();
			break;

		default:
			break;
		}
	}
}
