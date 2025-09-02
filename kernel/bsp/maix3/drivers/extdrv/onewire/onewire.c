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
#include <stdlib.h>
#include <string.h>

#include <sys/ioctl.h>

#include <rtthread.h>

#include "lwp.h"
#include "lwp_arch.h"
#include "lwp_user_mm.h"
#include "rtdef.h"

#include "drv_gpio.h"
#include "tick.h"

#include "onewire.h"

#define DBG_TAG "onewire"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define TIMING_RESET1 (480)
#define TIMING_RESET2 (70)
#define TIMING_RESET3 (410)
#define TIMING_READ1  (10)
#define TIMING_READ2  (10)
#define TIMING_READ3  (40)
#define TIMING_WRITE1 (10)
#define TIMING_WRITE2 (50)
#define TIMING_WRITE3 (10)

#define ONEWIRE_IOCTL_RESET      _IOWR('o', 0x00, struct onewire_rdwr_t*)
#define ONEWIRE_IOCTL_WRITE_BYTE _IOWR('o', 0x01, struct onewire_rdwr_t*)
#define ONEWIRE_IOCTL_READ_BYTE  _IOWR('o', 0x02, struct onewire_rdwr_t*)
#define ONEWIRE_IOCTL_SEARCH_ROM _IOWR('o', 0x03, struct onwwire_search_rom_t*)
#define PIN_PULSE_US             _IOWR('o', 0x04, struct pin_pulse_t*)

struct onewire_rdwr_t {
    int     pin;
    uint8_t data;
};

struct onwwire_search_rom_t {
    int     pin;
    uint8_t rom[8];
    uint8_t l_rom[8];
    int     diff;

    int result;
};

struct pin_pulse_t {
    int      pin;
    int      pulse_level;
    uint64_t timeout_us;

    uint64_t result;
};

static rt_err_t onewire_reset(struct onewire_rdwr_t* cfg)
{
    rt_base_t         level;
    gpio_drive_mode_t mode;
    int               status;

    int      pin     = cfg->pin;
    uint16_t timing1 = TIMING_RESET1;
    uint16_t timing2 = TIMING_RESET2;
    uint16_t timing3 = TIMING_RESET3;

    kd_pin_mode_get(pin, (rt_base_t*)&mode);
    if (GPIO_DM_OUTPUT_OD != mode) {
        if (kd_pin_mode(pin, GPIO_DM_OUTPUT_OD) != RT_EOK) {
            LOG_E("Failed to set pin %d to od mode", pin);
            return -RT_ERROR;
        }
    }

    kd_pin_set_dr(pin, 0);
    level = rt_hw_interrupt_disable();
    cpu_ticks_delay_us(timing1);
    kd_pin_set_dr(pin, 1);
    cpu_ticks_delay_us(timing2);
    status = kd_pin_get_dr(pin);
    rt_hw_interrupt_enable(level);
    cpu_ticks_delay_us(timing3);

    cfg->data = status ? 0 : 1;

    return RT_EOK;
}

static inline void onewire_write_bit(int pin, uint16_t timing1, uint16_t timing2, uint16_t timing3, int value)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    kd_pin_set_dr(pin, 0);
    cpu_ticks_delay_us(timing1);
    if (value) {
        kd_pin_set_dr(pin, 1);
    }
    cpu_ticks_delay_us(timing2);
    kd_pin_set_dr(pin, 1);
    cpu_ticks_delay_us(timing3);
    rt_hw_interrupt_enable(level);
}

static rt_err_t onewire_write_byte(struct onewire_rdwr_t* cfg)
{
    int      pin     = cfg->pin;
    uint16_t timing1 = TIMING_WRITE1;
    uint16_t timing2 = TIMING_WRITE2;
    uint16_t timing3 = TIMING_WRITE3;

    int value = cfg->data & 0xFF;

    rt_enter_critical();
    kd_pin_set_dr(pin, 1);
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(pin, timing1, timing2, timing3, value & 0x01);
        value >>= 1;
    }
    rt_exit_critical();

    return RT_EOK;
}

static inline int onewire_read_bit(int pin, uint16_t timing1, uint16_t timing2, uint16_t timing3)
{
    rt_base_t level;
    int       value;

    kd_pin_set_dr(pin, 1);
    level = rt_hw_interrupt_disable();
    kd_pin_set_dr(pin, 0);
    cpu_ticks_delay_us(timing1);
    kd_pin_set_dr(pin, 1);
    cpu_ticks_delay_us(timing2);
    value = kd_pin_get_dr(pin);
    rt_hw_interrupt_enable(level);
    cpu_ticks_delay_us(timing3);

    return value;
}

static rt_err_t onewire_read_byte(struct onewire_rdwr_t* cfg)
{
    int      pin     = cfg->pin;
    uint16_t timing1 = TIMING_READ1;
    uint16_t timing2 = TIMING_READ2;
    uint16_t timing3 = TIMING_READ3;

    uint8_t value = 0;

    rt_enter_critical();
    for (int i = 0; i < 8; ++i) {
        value |= onewire_read_bit(pin, timing1, timing2, timing3) << i;
    }
    rt_exit_critical();

    cfg->data = value;

    return RT_EOK;
}

static rt_err_t onewire_search_rom(struct onwwire_search_rom_t* cfg)
{
    int pin = cfg->pin;

    struct onewire_rdwr_t temp_cfg = { .pin = pin };

    onewire_reset(&temp_cfg);
    if (0x00 == temp_cfg.data) {
        cfg->result = -1;

        return RT_EOK;
    }
    cpu_ticks_delay_us(1);

    /* write byte 0xF0 */
    kd_pin_set_dr(pin, 1);
    rt_enter_critical();
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(pin, TIMING_WRITE1, TIMING_WRITE2, TIMING_WRITE3, (0xF0 >> i) & 0x01);
    }
    rt_exit_critical();

    int i         = 64;
    int next_diff = 0;
    int diff      = cfg->diff;

    rt_enter_critical();
    for (int byte = 0; byte < 8; byte++) {
        uint8_t r_b = 0;

        for (int bit = 0; bit < 8; bit++) {
            int b1 = onewire_read_bit(pin, TIMING_READ1, TIMING_READ2, TIMING_READ3);
            int b2 = onewire_read_bit(pin, TIMING_READ1, TIMING_READ2, TIMING_READ3);

            if (b2) {
                if (b1) {
                    // there are no devices or there is an error on the bus
                    rt_exit_critical();

                    cfg->result = -1;

                    return RT_EOK;
                }
            } else {
                if (0x00 == b1) {
                    // collision, two devices with different bit meaning
                    if (diff > i || ((cfg->l_rom[byte] & (1 << bit)) && (diff != i))) {
                        b1        = 1;
                        next_diff = i;
                    }
                }
            }

            kd_pin_set_dr(pin, 1);
            onewire_write_bit(pin, TIMING_WRITE1, TIMING_WRITE2, TIMING_WRITE3, b1);

            if (b1) {
                r_b |= (1 << bit);
            }
            i -= 1;
        }

        cfg->rom[byte] = r_b;
    }
    rt_exit_critical();

    cfg->diff = next_diff;

    cfg->result = 0;

    return RT_EOK;
}

static rt_err_t pin_pulse_us(struct pin_pulse_t* cfg)
{
    int      pin         = cfg->pin;
    int      pulse_level = cfg->pulse_level;
    uint64_t start, timeout_us = cfg->timeout_us;

    rt_enter_critical();

    start = cpu_ticks_us();

    while (kd_pin_get_dr(pin) != pulse_level) {
        if ((cpu_ticks_us() - start) >= timeout_us) {
            cfg->result = -2;

            rt_exit_critical();
            return RT_EOK;
        }
    }

    start = cpu_ticks_us();
    while (kd_pin_get_dr(pin) == pulse_level) {
        if ((cpu_ticks_us() - start) >= timeout_us) {
            cfg->result = -1;

            rt_exit_critical();
            return RT_EOK;
        }
    }

    cfg->result = cpu_ticks_us() - start;
    rt_exit_critical();

    return RT_EOK;
}

static rt_err_t _onewire_dev_control(rt_device_t dev, int cmd, void* args)
{
    rt_err_t ret = RT_EOK;

    uint32_t _cmd = (uint32_t)cmd;

    if ((ONEWIRE_IOCTL_RESET == _cmd) || (ONEWIRE_IOCTL_WRITE_BYTE == _cmd) || (ONEWIRE_IOCTL_READ_BYTE == _cmd)) {
        struct onewire_rdwr_t cfg;

        if (sizeof(cfg) != lwp_get_from_user(&cfg, args, sizeof(cfg))) {
            return -RT_EIO;
        }

        if (ONEWIRE_IOCTL_RESET == _cmd) {
            ret = onewire_reset(&cfg);
        } else if (ONEWIRE_IOCTL_WRITE_BYTE == _cmd) {
            ret = onewire_write_byte(&cfg);
        } else if (ONEWIRE_IOCTL_READ_BYTE == _cmd) {
            ret = onewire_read_byte(&cfg);
        } else {
            ret = -RT_EINVAL;
        }

        if (sizeof(cfg) != lwp_put_to_user(args, &cfg, sizeof(cfg))) {
            return -RT_EIO;
        }

        return ret;
    } else if (ONEWIRE_IOCTL_SEARCH_ROM == _cmd) {
        struct onwwire_search_rom_t cfg;

        if (sizeof(cfg) != lwp_get_from_user(&cfg, args, sizeof(cfg))) {
            return -RT_EIO;
        }

        ret = onewire_search_rom(&cfg);

        if (sizeof(cfg) != lwp_put_to_user(args, &cfg, sizeof(cfg))) {
            return -RT_EIO;
        }

        return ret;
    } else if (PIN_PULSE_US == _cmd) {
        struct pin_pulse_t cfg;

        if (sizeof(cfg) != lwp_get_from_user(&cfg, args, sizeof(cfg))) {
            return -RT_EIO;
        }

        ret = pin_pulse_us(&cfg);

        if (sizeof(cfg) != lwp_put_to_user(args, &cfg, sizeof(cfg))) {
            return -RT_EIO;
        }

        return ret;
    }

    LOG_E("Unsupported command: %d", cmd);

    return -RT_EINVAL;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops onewire_dev_ops = {
    .init    = RT_NULL,
    .open    = RT_NULL,
    .close   = RT_NULL,
    .read    = RT_NULL,
    .write   = RT_NULL,
    .control = _onewire_dev_control,
};
#endif

static int onewire_device_init(void)
{
    rt_err_t err = RT_EOK;

    static struct rt_device onewire_dev;
    static int              onewire_inited = 0;

    if (onewire_inited) {
        return 0;
    }

    rt_memset(&onewire_dev, 0, sizeof(struct rt_device));

    onewire_dev.user_data = NULL;
    onewire_dev.type      = RT_Device_Class_Char;

#ifdef RT_USING_DEVICE_OPS
    onewire_dev.ops = &onewire_dev_ops;
#else
    onewire_dev.init    = _wlan_mgmt_init;
    onewire_dev.open    = RT_NULL;
    onewire_dev.close   = RT_NULL;
    onewire_dev.read    = RT_NULL;
    onewire_dev.write   = RT_NULL;
    onewire_dev.control = _rt_wlan_dev_control;
#endif

    /* register to device manager */
    rt_device_register(&onewire_dev, "onewire", RT_DEVICE_FLAG_RDWR);

    return 0;
}
INIT_APP_EXPORT(onewire_device_init);
