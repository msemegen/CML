/*
    Name: system.c

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <stdint.h>

//externals
#include <stm32l452xx.h>

uint32_t SystemCoreClock = 4000000;

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  /* set CP10 and CP11 Full Access */
#endif
    RCC->CR |= RCC_CR_MSION;
    RCC->CFGR = 0x00000000;
    RCC->CR &= (uint32_t)0xEAF6FFFF;
    RCC->PLLCFGR = 0x00001000;
    RCC->CR &= (uint32_t)0xFFFBFFFF;
    RCC->CIER = 0x00000000;
    SCB->VTOR = FLASH_BASE; /* Vector Table Relocation in Internal FLASH */
}

__attribute__((weak)) int _init()
{
    return 0;
}

__attribute__((used)) void _fini(void)
{

}