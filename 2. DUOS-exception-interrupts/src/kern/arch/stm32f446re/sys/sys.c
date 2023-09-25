#include <sys.h>
#include <kstdio.h>

void __NVIC_SetPriority(IRQn_TypeDef IRQn, uint32_t priority)
{
      if (IRQn >= 0) NVIC->IP[IRQn] = (uint8_t)((priority << 4));
      else SCB->SHPR[(IRQn & 15)-4] = (uint8_t)(priority << 4);
}

uint32_t __NVIC_GetPriority(IRQn_TypeDef IRQn)
{
      if (IRQn >= 0) return (NVIC->IP[IRQn] >> 4);
      else return (SCB->SHPR[(IRQn & 15)-4] >> 4);
}

void __NVIC_EnableIRQn(IRQn_TypeDef IRQn)
{
      if (IRQn >= 0) NVIC->ISER[IRQn / 32] |= (1 << (IRQn % 32));
}

void __NVIC_DisableIRQn(IRQn_TypeDef IRQn)
{
      if (IRQn >= 0) NVIC->ICER[IRQn / 32] |= (1 << (IRQn % 32));
}

void __disable_irq()
{
      asm("mov r0,#1");
      asm("msr primask,r0");
}
void __set_BASEPRI(uint32_t value) {

    value = (value << 4);
    asm volatile("MSR BASEPRI, %0" : : "r" (value) : "memory");
    kprintf("BASEPRI disabled interrupt with priority higher than %d\n",(value>>4));
}

uint32_t get_basepri_value(void) {
   
    uint32_t value;
    asm("mrs %0, basepri" : "=r"(value));
    return (value >> 4);
}
void __unset_BASEPRI(uint32_t value)
{
      asm("mov r0,#0");
      asm("msr basepri,r0");
      kprintf("BASEPRI reset complete.");
}
void __enable_irq()
{
      asm("mov r0,#0");
      asm("msr primask,r0");
}

void __set_PRIMASK(uint32_t priMask)
{
      asm("mov r0, #1");
      asm("msr primask, r0");
}

uint32_t get_PRIMASK(void)
{
      uint32_t value;
      asm("mrs r0,primask");
      asm("mov %0,r0"
          : "=r"(value));
      return value;
}

void __enable_fault_irq(void)
{
      asm("mov r0, #0");
      asm("msr primask, r0");
}

void __set_FAULTMASK(uint32_t faultMask)
{
      asm("mov r0, #1");
      asm("msr primask, r0");
}

void __disable_fault_irq(void)
{
      asm("mov r0, #1");
      asm("msr primask, r0");
}

uint32_t __get_FAULTMASK(void)
{
      uint32_t value;
      asm("mrs r0,faultmask");
      asm("mov %0,r0"
          : "=r"(value));
      return value;
}

void __clear_pending_IRQn(IRQn_TypeDef IRQn)
{
      if (IRQn >= 0) NVIC->ICPR[IRQn / 32] |= (1 << (IRQn % 32));
}

uint32_t __get_pending_IRQn(IRQn_TypeDef IRQn)
{
      uint32_t pendingState;
      if (IRQn >= 0)
      {
            int regNumber = IRQn / 32;
            // int offset = IRQn % 32;
            pendingState = NVIC->ICPR[regNumber] & (1 << regNumber);
            pendingState = pendingState >> 5;
            return pendingState;
      }
}

uint32_t __NVIC_GetActive(IRQn_TypeDef IRQn)
{
      uint32_t pendingState;
      if (IRQn >= 0)
      {
            int regNumber = IRQn / 32;
            // int offset = IRQn % 32;
            pendingState = NVIC->IABR[regNumber] & (1 << regNumber);
            pendingState = pendingState >> 5;
            return pendingState;
      }
}