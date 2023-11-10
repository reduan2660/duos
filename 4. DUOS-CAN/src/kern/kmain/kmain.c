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
#include <can.h>
#include <a9g.h>

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

/*----------------------------------------------------------------------------
  initialize CAN interface
 *----------------------------------------------------------------------------*/
unsigned int val_Tx = 0, val_Rx = 0; // Global variables used for display

void pseudo_delay(unsigned int nCount)
{
	for (; nCount != 0; nCount--)
		;
}

void can_Init(void)
{
	CAN_setup();					   // setup CAN interface
	CAN_wrFilter(33, STANDARD_FORMAT); // Enable reception of messages

	/* COMMENT THE LINE BELOW TO ENABLE DEVICE TO PARTICIPATE IN CAN NETWORK   */
	// CAN_testmode(CAN_BTR_SILM | CAN_BTR_LBKM); // Loopback, Silent Mode (self-test)

	CAN_start(); // leave init mode

	CAN_waitReady(); // wait til mbx is empty
}

void kmain(void)
{
	__sys_init();
	// __SysTick_init(16777215);
	GPIO_Config();
	a9g_init();

	// // CAN
	// kprintf("CAN initializing. May take a while.\n");

	// can_Init(); // initialise CAN interface
	// // pseudo_delay(4500000);
	// kprintf("CAN initialised.\n");

	// CAN_TxMsg.id = 33; // initialise message to send

	// int i;
	// for (i = 0; i < 8; i++)
	// 	CAN_TxMsg.data[i] = 0;
	// CAN_TxMsg.len = 1;
	// CAN_TxMsg.format = STANDARD_FORMAT;
	// CAN_TxMsg.type = DATA_FRAME;

	// while (1)
	// { // Loop forever
	// 	if (CAN_TxRdy)
	// 	{
	// 		CAN_TxRdy = 0;

	// 		// CAN_TxMsg.data[0] = adc_Get(); // data[0] field = ADC value
	// 		CAN_wrMsg(&CAN_TxMsg); // transmit message
	// 		val_Tx = CAN_TxMsg.data[0];
	// 	}

	// 	pseudo_delay(1000000); // Wait a while to receive the message

	// 	if (CAN_RxRdy)
	// 	{
	// 		CAN_RxRdy = 0;

	// 		val_Rx = CAN_RxMsg.data[0];
	// 	}

	// 	// print the values
	// 	kprintf("Tx: %d, Rx: %d\n", val_Tx, val_Rx);
	// }

	while (1)
	{
		pseudo_delay(1000);
		// kprintf("Enter a command: ");
		// kscanf("%s", atCommand);
		// kprintf(atCommand);
	}

	kprintf("Program Ended\n");
}
