/* Copyright (c) 2025, Canaan Bright Sight Co., Ltd
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
#include <stdio.h>

#include "rtconfig.h"
#include "rtthread.h"

#include "drv_gpio.h"

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

/* include configs/{CONFIG_BOARD}/pinmux_config.c */
#include TOSTRING(BOARD_CFG_FILE)

int board_pinmux_init(void)
{
    uint32_t curr, set;

    kd_fpioa_init();

    for (int i = 0; i < K230_PIN_COUNT; i++) {
        curr = fpioa_get_pin_cfg(i);
        curr &= 0x7FFFFFFF; /* ignore di */
        set = board_pinmux_cfg[i].u.value;

        if (curr != set) {
            rt_kprintf("Pin %d setting not same\n", i);
            rt_kprintf("curr 0x%08X, set 0x%08X\n", curr, set);
        } else {
            fpioa_set_pin_cfg(i, set);
        }
    }

    return 0;
}

static int _board_specific_pin_init_sequence_wrap()
{
    rt_hw_gpio_init();

    board_specific_pin_init_sequence();

    return 0;
}
INIT_PREV_EXPORT(_board_specific_pin_init_sequence_wrap);
