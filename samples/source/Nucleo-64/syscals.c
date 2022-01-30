/*
 *   Name: syscals.c
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>

caddr_t _sbrk(int incr)
{
    register int8_t* stack_ptr asm("sp");

    extern int8_t end asm("end");
    static int8_t* heap_end;
    int8_t* prev_heap_end;

    if (heap_end == 0) heap_end = &end;

    prev_heap_end = heap_end;
    if (heap_end + incr > stack_ptr)
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;

    return (caddr_t)prev_heap_end;
}
