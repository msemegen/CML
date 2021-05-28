/*
 *   Name: syscalls.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cerrno>
#include <cstddef>
#include <sys/stat.h>

// externals
#include <stm32l452xx.h>

extern "C" {

static uint8_t* __sbrk_heap_end = nullptr;

int _read(int, char* data, int len)
{
    return -1;
}

int _write(int, char* data, int len)
{
    int ret = 0;
    for (; ret < len; ret++)
    {
        ITM_SendChar(data[ret]);
    }

    return ret;
}

void* _sbrk(std::ptrdiff_t incr)
{
    extern uint8_t _end;
    extern uint8_t _estack;
    extern uint32_t _Min_Stack_Size;
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    const uint8_t* max_heap    = (uint8_t*)stack_limit;
    uint8_t* prev_heap_end;

    if (nullptr == __sbrk_heap_end)
    {
        __sbrk_heap_end = &_end;
    }

    if (__sbrk_heap_end + incr > max_heap)
    {
        errno = ENOMEM;
        return (void*)-1;
    }

    prev_heap_end = __sbrk_heap_end;
    __sbrk_heap_end += incr;

    return (void*)prev_heap_end;
}

int _close(int file)
{
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}
int _fstat(int file, struct stat* st)
{
    st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int file)
{
    return 0;
}

void _exit(int status)
{
    while (1)
        ;
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

int _getpid()
{
    return 1;
}

} // extern "C";
