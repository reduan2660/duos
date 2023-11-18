#include <schedule.h>
#include <ustd.h>
#include <types.h>
#include <sem.h>

ReadyQ_TypeDef queue;
TCB_TypeDef *sleep_task, *current_task;

uint32_t TASK_ID = 1;
uint32_t task_start_time = 0;
uint32_t SCHEDEULING_ALGORITHM = ROUND_ROBIN;

semaphore task_semaphore = 1;
ReadyQ_TypeDef sem_queue;

void init_queue(void)
{
	queue.size = 0;
	queue.max = MAX_TASKS;
	queue.st = 0;
	queue.ed = -1;
}

void queue_add(TCB_TypeDef *task)
{
	if (queue.size == queue.max)
	{
		return;
	}
	queue.ed = (queue.ed + 1) % queue.max;
	queue.q[queue.ed] = task;
	queue.size++;
}
TCB_TypeDef *pop()
{
	if (queue.size == 0)
	{
		queue_add(sleep_task);
	}

	TCB_TypeDef *task = queue.q[queue.st];
	queue.st = (queue.st + 1) % queue.max;
	queue.size--;
	return task;
}

//-------------scheduling functions----------------

void __schedule(void)
{
	// ------ROUND ROBIN SCHEDULING ALGORITHM------

	if (SCHEDEULING_ALGORITHM == ROUND_ROBIN)
	{

		if (current_task->status == RUNNING)
		{
			current_task->status = READY;
			queue_add(current_task);
		}

		if (current_task->status == KILLED)
		{
			uint32_t turn_around_time = __getTime() - current_task->start_time_t;
			current_task->waiting_time = turn_around_time - current_task->execution_time;

			// kprintf("time\tstart\texecution\tturnaround\twaiting\n");
			// kprintf("%d\t%d\t%d\t\t%d\t\t%d\n", __getTime(), current_task->start_time_t, current_task->execution_time, turn_around_time, current_task->waiting_time);
		}

		current_task->execution_time += PENDSV_TICK_TIME;
		TCB_TypeDef *front = pop();
		current_task = front;
		current_task->status = RUNNING;
		if (current_task->response_time_t == 0)
		{
			current_task->response_time_t = __getTime();
		}
		return;
	}

	// ------FCFS SCHEDULING ALGORITHM------
	else if (SCHEDEULING_ALGORITHM == FIRST_COME_FIRST_SERVE)
	{
		uint32_t currenctTime = __getTime();
		current_task->execution_time += currenctTime - current_task->response_time_t;
		current_task = pop();
		current_task->response_time_t = currenctTime;
		current_task->waiting_time += currenctTime - current_task->start_time_t;

		current_task->status = RUNNING;
		return;
	}
}

void __create_task(TCB_TypeDef *tcb, void (*task)(void), uint32_t *stack_start)
{
	tcb->magic_number = 0xFECABAA0;
	tcb->task_id = TASK_ID++;
	tcb->psp = stack_start;
	tcb->status = READY;

	tcb->start_time_t = __sys_get_time();
	tcb->response_time_t = 0;
	tcb->execution_time = 0;
	tcb->waiting_time = 0;

	tcb->priority = 1;
	tcb->digital_sinature = 0x00000001;

	*(--tcb->psp) = DUMMY_XPSR;		// xPSR
	*(--tcb->psp) = (uint32_t)task; // PC
	*(--tcb->psp) = 0xFFFFFFFD;		// LR

	// store R0 - R3, R12
	for (int i = 0; i < 5; i++)
		*(--tcb->psp) = 0x00000000;
	*(--tcb->psp) = (uint32_t)tcb;
	// store R4 - R11
	for (int i = 0; i < 7; i++)
		*(--tcb->psp) = 0x00000000;
}

void __start_task(void)
{

	if (queue.size == 0)
		return;

	current_task = pop(); // current task = front of queue

	// print_task_info(current_task);
	// retarted_dealy();

	if (current_task->magic_number != 0xFECABAA0 || current_task->digital_sinature != 0x00000001)
	{
		kprintf("Invalid task\n");
		return;
	}
	uint32_t cur_time = __sys_get_time();
	current_task->start_time_t = cur_time;
	current_task->response_time_t = cur_time;
	task_start_time = cur_time;
	current_task->status = RUNNING;

	start_task(current_task->psp);
}

void __set_sleep(TCB_TypeDef *task)
{
	sleep_task = task;
	return;
}

void __attribute__((naked)) PendSV_Handler(void)
{
	// Clear all pending interrupts
	SCB->ICSR |= (1 << 27);

	// save current context
	__asm volatile(
		"mrs r0, psp\n"
		"stmdb r0!, {r4-r11}\n"
		"push {lr}\n");

	__asm volatile("mov %0, r0"
				   : "=r"(current_task->psp)
				   :);
	// Schedule next task
	__schedule();

	__asm volatile(
		"mov r0, %0"
		:
		: "r"(current_task->psp));
	__asm volatile(
		"ldmia r0!,{r4-r11}\n"
		"msr psp, r0\n"
		"pop {lr}\n"
		"bx lr\n");
}

void retarted_dealy(void)
{
	int x = 100000;
	while (x--)
		__asm volatile("nop");
}

void print_entire_queue(void)
{
	kprintf("Printint entire queue ___________\n");
	kprintf("Queue size = %d\n", queue.size);
	kprintf("Queue max = %d\n", queue.max);
	kprintf("Queue st = %d\n", queue.st);
	kprintf("Queue ed = %d\n", queue.ed);

	for (int i = 0; i < queue.size; i++)
	{
		kprintf("Queue q[%d] = %x\n", i, queue.q[i]);
		print_task_info(queue.q[i]);
	}
	kprintf("END QUEUE PRINT ___________\n");
}

void print_task_info(TCB_TypeDef *task)
{
	kprintf("_____________TASK INFO_____________\n");
	kprintf("Task ID = %d\n", task->task_id);
	kprintf("\n");
}

// -------- Semaphore ---------------------------

void down(void)
{
	sem_dec(&task_semaphore);
	if (task_semaphore < 0)
	{
		push_sem(current_task);
		current_task->status = SLEEPING;
	}
}

void up(void)
{
	sem_inc(&task_semaphore);
	if (task_semaphore >= 0)
	{
		TCB_TypeDef *task = pop_sem();
		task->status = READY;
	}
}

void push_sem(TCB_TypeDef *task)
{
	if (sem_queue.size == sem_queue.max)
	{
		return;
	}
	sem_queue.ed = (sem_queue.ed + 1) % sem_queue.max;
	sem_queue.q[sem_queue.ed] = task;
	sem_queue.size++;
}

TCB_TypeDef *pop_sem(void)
{
	if (sem_queue.size == 0)
	{
		return;
	}
	TCB_TypeDef *task = sem_queue.q[sem_queue.st];
	sem_queue.st = (sem_queue.st + 1) % sem_queue.max;
	sem_queue.size--;
	return task;
}

int is_sem_empty(void)
{
	return sem_queue.size == 0;
}
