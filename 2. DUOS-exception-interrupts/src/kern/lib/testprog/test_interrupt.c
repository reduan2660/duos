#include <test_interrupt.h>
#include <cm4.h>
#include <kstdio.h>
#include <sys.h>

#define SYS_TICK_INTERVAL_MS 10 // 10 ms interval
#define HSE_VALUE 8000000UL
volatile int hardfault_event_enabled = 0; // Volatile to prevent optimization
uint32_t clockChoice = 1;
uint32_t clockRateMHz = 180;

void enable_hardfault_event(void)
{
      SCB->SHCSR |= (1 << 16);               // enable mem fault
      volatile int *ptr = (int *)0xFFFFFFFF; // Invalid memory address
      int value = *ptr;                      // This will trigger a HardFault
      (void)value;                           // Just to avoid compiler warnings
}

void enableSysTickInterrupt(void)
{
      SYSTICK->CTRL |= (1 << 0);
      kprintf("Systick Interrupt Enabled\n");
}

void disableSysTickInterrupt(void)
{
      SYSTICK->CTRL &= ~(1 << 0);
      kprintf("Systick Interrupt Disabled\n");
}

void reboot()
{
      kprintf("Rebooting...\n");
      SCB->AIRCR = (0x5FA << 16) | (0x4 << 0); // SCB_AIRCR = VECTKEY | SYSRESETREQ
}