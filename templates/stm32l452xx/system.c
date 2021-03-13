/*
 *   Name: system.c
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <stdint.h>

// externals
#include <stm32l452xx.h>

uint32_t SystemCoreClock = 4000000;

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif
    RCC->CR |= RCC_CR_MSION;
    RCC->CFGR = 0x00000000;
    RCC->CR &= 0xEAF6FFFFu;
    RCC->PLLCFGR = 0x00001000;
    RCC->CR &= 0xFFFBFFFFu;
    RCC->CIER = 0x00000000;
    SCB->VTOR = FLASH_BASE;
}

__attribute__((weak)) int _init()
{
    return 0;
}

__attribute__((used)) void _fini(void) {}