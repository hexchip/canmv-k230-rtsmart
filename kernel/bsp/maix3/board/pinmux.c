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

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

/* include configs/{CONFIG_BOARD}/pinmux_config.c */
#include TOSTRING(BOARD_CFG_FILE)

static inline int should_ignore_pin(int pin)
{
    /* List of pins to ignore for voltage bank consistency checks */
    const int    ignore_pins[]   = { 9, 42, 43, 46, 47, 50, 51, 52, 53, 54, 55, 56, 57, 58 };
    const size_t num_ignore_pins = sizeof(ignore_pins) / sizeof(ignore_pins[0]);

    for (size_t i = 0; i < num_ignore_pins; i++) {
        if (pin == ignore_pins[i]) {
            return 1;
        }
    }
    return 0;
}

#ifdef CONFIG_BOARD_CHIP_K230D

#if !defined(VOL_BANK0_IO2_13) || !defined(VOL_BANK3_IO38_49) || !defined(VOL_BANK4_IO50_61)
#error "must define VOL_BANK4_IO50_61"
#endif

struct k230d_drop_pin_cfg {
    int                   pin;
    struct st_iomux_reg_t reg;
};

static const struct k230d_drop_pin_cfg k230d_drop_pin_cfgs[] = {
    { 9, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK0_IO2_13, .io_sel = 0 } } },

    { 42, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK3_IO38_49, .io_sel = 0 } } },
    { 43, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK3_IO38_49, .io_sel = 0 } } },
    { 46, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK3_IO38_49, .io_sel = 0 } } },
    { 47, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK3_IO38_49, .io_sel = 0 } } },

    { 50, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 51, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 52, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 53, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 54, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 55, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 56, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 57, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
    { 58, { .u.bit = { .st = 0, .ds = 0, .pd = 0, .pu = 0, .oe = 0, .ie = 0, .msc = VOL_BANK4_IO50_61, .io_sel = 0 } } },
};

#endif

static int board_pinmux_init(void)
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
        }
    }

    return 0;
}
INIT_BOARD_EXPORT(board_pinmux_init);
