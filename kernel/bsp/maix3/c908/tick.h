/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/10/28     Bernard      The unify RISC-V porting code.
 */

#ifndef TICK_H__
#define TICK_H__

#include <stdint.h>

// #define TICK_USE_SBI
#define TIMER_CLK_FREQ  (27000000)

int tick_isr(void);
int rt_hw_tick_init(void);

static inline __attribute__((always_inline)) uint64_t cpu_ticks(void)
{
    uint64_t time;
    __asm__ __volatile__("rdtime %0" : "=r"(time));
    return time;
}

static __inline __attribute__((__always_inline__)) uint64_t cpu_ticks_ms(void)
{
    uint64_t time;
    __asm__ __volatile__("rdtime %0" : "=r"(time));
    return (time / (TIMER_CLK_FREQ / 1000));
}

static __inline __attribute__((__always_inline__)) uint64_t cpu_ticks_us(void)
{
    uint64_t time;
    __asm__ __volatile__("rdtime %0" : "=r"(time));
    return (time / (TIMER_CLK_FREQ / 1000000));
}

static __inline __attribute__((__always_inline__)) uint64_t cpu_ticks_ns(void)
{
    uint64_t time;
    __asm__ __volatile__("rdtime %0" : "=r"(time));
    return (time * 1000000000ULL) / TIMER_CLK_FREQ;
}

void cpu_ticks_delay_us(uint64_t us);
void cpu_ticks_delay_ms(uint64_t ms);

#endif
