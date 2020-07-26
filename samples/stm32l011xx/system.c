/*
    Name: system.c

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <stdint.h>

//externals
#include <stm32l011xx.h>

uint32_t SystemCoreClock = 2000000U;

void SystemInit(void)
{
    RCC->CR |= (uint32_t)0x00000100U;
    RCC->CFGR &= (uint32_t)0x88FF400CU;
    RCC->CR &= (uint32_t)0xFEF6FFF6U;
    RCC->CRRCR &= (uint32_t)0xFFFFFFFEU;
    RCC->CR &= (uint32_t)0xFFFBFFFFU;
    RCC->CFGR &= (uint32_t)0xFF02FFFFU;
    RCC->CIER = 0x00000000U;
    SCB->VTOR = FLASH_BASE; /* Vector Table Relocation in Internal FLASH */
}

__attribute__((weak)) int _init()
{
    return 0;
}

__attribute__((used)) void _fini(void)
{

}