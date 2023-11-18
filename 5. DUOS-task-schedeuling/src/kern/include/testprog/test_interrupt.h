#ifndef TEST_INTERRUPT_H
#define TEST_INTERRUPT_H

void enable_hardfault_event(void);
void enableSysTickInterrupt(void);
void disableSysTickInterrupt(void);
void reboot(void);
#endif // TEST_INTERRUPT_H