#pragma once

#include "riscv_io.h"
#include "tick.h"

static inline void __cpu_relax(void) { __asm__ __volatile__("" ::: "memory"); }

#define poll_timeout_us(op, cond, delay_us, timeout_us, delay_before_op)                                                       \
    ({                                                                                                                         \
        uint64_t __timeout_us = (timeout_us);                                                                                  \
        uint64_t __delay_us   = (delay_us);                                                                                    \
        uint64_t __start_time = cpu_ticks_us();                                                                                \
        uint64_t __end_time   = __timeout_us ? (__start_time + __timeout_us) : 0;                                              \
        int      ___ret;                                                                                                       \
                                                                                                                               \
        if ((delay_before_op) && __delay_us)                                                                                   \
            cpu_ticks_delay_us(__delay_us);                                                                                    \
                                                                                                                               \
        for (;;) {                                                                                                             \
            bool __expired = false;                                                                                            \
            /* Check timeout condition */                                                                                      \
            if (__timeout_us) {                                                                                                \
                uint64_t __current_time = cpu_ticks_us();                                                                      \
                /* Basic timeout check: current >= end time */                                                                 \
                if (__current_time >= __end_time && __current_time >= __start_time) {                                          \
                    __expired = true;                                                                                          \
                }                                                                                                              \
                /* Handle timer wrap-around (if end_time was before wrap) */                                                   \
                if (__current_time < __start_time && __current_time >= __end_time) {                                           \
                    __expired = true;                                                                                          \
                }                                                                                                              \
            }                                                                                                                  \
                                                                                                                               \
            /* guarantee 'op' and 'cond' are evaluated after timeout expired check */                                          \
            __io_ar();                                                                                                         \
            op; /* Perform the operation */                                                                                    \
                                                                                                                               \
            if (cond) {                                                                                                        \
                ___ret = 0; /* Condition met */                                                                                \
                break;                                                                                                         \
            }                                                                                                                  \
            if (__expired) {                                                                                                   \
                ___ret = -ETIMEDOUT; /* Timeout */                                                                             \
                break;                                                                                                         \
            }                                                                                                                  \
                                                                                                                               \
            if (__delay_us)                                                                                                    \
                cpu_ticks_delay_us(__delay_us); /* Busy-wait delay */                                                          \
                                                                                                                               \
            __cpu_relax(); /* Add a simple relax */                                                                            \
        }                                                                                                                      \
        ___ret;                                                                                                                \
    })

#define read_poll_timeout(op, val, cond, delay_us, timeout_us, delay_before_read, args...)                                     \
    poll_timeout_us((val) = op(args), cond, delay_us, timeout_us, delay_before_read)

#define readx_poll_timeout(op, addr, val, cond, delay_us, timeout_us)                                                          \
    read_poll_timeout(op, val, cond, delay_us, timeout_us, false, addr)

#define readb_poll_timeout(addr, val, cond, delay_us, timeout_us)                                                              \
    readx_poll_timeout(readb, addr, val, cond, delay_us, timeout_us)

#define readw_poll_timeout(addr, val, cond, delay_us, timeout_us)                                                              \
    readx_poll_timeout(readw, addr, val, cond, delay_us, timeout_us)

#define readl_poll_timeout(addr, val, cond, delay_us, timeout_us)                                                              \
    readx_poll_timeout(readl, addr, val, cond, delay_us, timeout_us)
