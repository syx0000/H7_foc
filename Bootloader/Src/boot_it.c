/**
 * @file    boot_it.c
 * @brief   Minimal interrupt vector handlers for the bootloader.
 *
 * Only the core exception handlers are needed: NMI, HardFault and friends
 * (so a fault is loud, not silent), plus SysTick which drives HAL_Delay /
 * HAL_GetTick. All peripheral IRQs default to the weak symbols in the
 * startup file (which are infinite loops).
 */
#include "stm32h7xx_hal.h"

void NMI_Handler(void)         { while (1); }
void HardFault_Handler(void)   { while (1); }
void MemManage_Handler(void)   { while (1); }
void BusFault_Handler(void)    { while (1); }
void UsageFault_Handler(void)  { while (1); }
void SVC_Handler(void)         { }
void DebugMon_Handler(void)    { }
void PendSV_Handler(void)      { }

void SysTick_Handler(void)
{
    HAL_IncTick();
}
