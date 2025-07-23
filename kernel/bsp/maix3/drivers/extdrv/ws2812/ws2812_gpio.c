/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

#include <rthw.h>
#include <rtthread.h>

#include "tick.h"

#include "drv_gpio.h"

#include "ws2812.h"

#define DBG_TAG "ws2812_gpio"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if defined(WS2812_USE_DRV_GPIO)

// Convert nanoseconds to timer ticks (rounded)
static inline uint32_t ns_to_ticks(uint32_t ns) { return (uint64_t)ns * TIMER_CLK_FREQ / 1000000000ULL; }

rt_err_t ws2812_stream_over_gpio(struct ws2812_stream* stream)
{
    uint32_t pin = stream->pin;

    size_t stream_len = stream->len;
    if (stream_len == 0) {
        LOG_E("Invalid stream length");
        return -RT_ERROR;
    }

    if (kd_pin_mode(pin, GPIO_DM_OUTPUT) != RT_EOK) {
        LOG_E("Failed to set pin %d to output mode", pin);
        return -RT_ERROR;
    }

    if (stream_len > WS2812_OVER_GPIO_MAX_LEN) {
        LOG_W("Stream length exceeds maximum allowed length: %ld > %d", stream_len, WS2812_OVER_GPIO_MAX_LEN);
        stream_len = WS2812_OVER_GPIO_MAX_LEN;
    }

    // Convert timing from ns to ticks
    uint32_t timing_ticks[4]; // [bit0_hi, bit0_total], [bit1_hi, bit1_total]
    for (int i = 0; i < 2; ++i) {
        uint32_t hi             = ns_to_ticks(stream->timing_ns[i * 2]);
        uint32_t lo             = ns_to_ticks(stream->timing_ns[i * 2 + 1]);
        timing_ticks[i * 2 + 0] = hi;
        timing_ticks[i * 2 + 1] = hi + lo;
    }

    const uint8_t* stream_data = stream->data;
    const uint8_t* stream_end  = stream_data + stream_len;

    // uint64_t ticks_start = cpu_ticks_us(); // For logging duration

    // Disable IRQs to ensure accurate timing
    rt_base_t level = rt_hw_interrupt_disable();

    while (stream_data < stream_end) {
        uint8_t byte = *stream_data++;

        for (int i = 0; i < 8; ++i) {
            uint64_t start = cpu_ticks();

            kd_pin_set_dr(pin, 1);

            int      bit     = (byte & 0x80) ? 1 : 0;
            uint32_t t_hi    = timing_ticks[bit * 2 + 0];
            uint32_t t_total = timing_ticks[bit * 2 + 1];

            // Delay for high time
            while ((uint32_t)(cpu_ticks() - start) < t_hi) {
                __asm__ volatile("nop");
            }

            kd_pin_set_dr(pin, 0);
            byte <<= 1;

            // Delay for remainder of bit period
            while ((uint32_t)(cpu_ticks() - start) < t_total) {
                __asm__ volatile("nop");
            }
        }
    }

    rt_hw_interrupt_enable(level);

    // LOG_D("Stream sent in %lu us", (uint32_t)(cpu_ticks_us() - ticks_start));

    return RT_EOK;
}

#endif /* WS2812_USE_DRV_GPIO */
