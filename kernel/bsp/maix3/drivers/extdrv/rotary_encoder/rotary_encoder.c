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

#include "rtconfig.h"
#include "rtdef.h"
#include "rtservice.h"
#include <rtdevice.h>
#include <rtthread.h>

#include "lwp.h"
#include "lwp_arch.h"
#include "lwp_user_mm.h"

#include "drv_gpio.h"
#include "tick.h"

#include "rotary_encoder.h"

#define DBG_TAG "rotary_encoder"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define ENCODER_EVENT_DATA_FLAG (0x01)

struct encoder_device {
    struct rt_device device;

    // Management
    int                      index;
    struct encoder_pin_cfg_t cfg;

    rt_list_t list;

    /* GPIO pins */
    rt_base_t clk_pin;
    rt_base_t dt_pin;
    rt_base_t sw_pin;

    /* State tracking */
    rt_int64_t count;
    rt_int32_t pending_delta;
    rt_uint8_t last_clk_state;
    rt_uint8_t direction; /* 1: CW, 0xFF: CCW, 0: no movement */

    /* Button state */
    rt_uint8_t  button_pressed;
    rt_tick_t   button_press_time;
    rt_uint64_t button_last_press_time;

    /* Synchronization */
    rt_base_t       level; /* For interrupt level */
    struct rt_event data_evt;

    /* Configuration state */
    rt_uint8_t configured;
};

/* Interrupt handler for CLK pin */
static void encoder_clk_isr(void* args)
{
    struct encoder_device* dev = (struct encoder_device*)args;

    rt_uint8_t clk_state, dt_state;

    if (!dev || !dev->configured) {
        return;
    }

    /* Read current pin states */
    dt_state  = kd_pin_get_dr(dev->dt_pin);
    clk_state = kd_pin_get_dr(dev->clk_pin);

    /* Detect CLK edge change (either rising or falling) */
    if (clk_state != dev->last_clk_state) {
        /* Determine rotation based on CLK and DT sequence */
        if (clk_state == GPIO_PV_HIGH) {
            /* Rising edge of CLK */
            if (dt_state == GPIO_PV_LOW) {
                /* Clockwise rotation */
                dev->count++;
                dev->pending_delta++;
                dev->direction = ENCODER_DIR_CW;
            } else {
                /* Counter-clockwise rotation */
                dev->count--;
                dev->pending_delta--;
                dev->direction = ENCODER_DIR_CCW;
            }
        } else {
            /* Falling edge of CLK */
            if (dt_state == GPIO_PV_HIGH) {
                /* Clockwise rotation */
                dev->count++;
                dev->pending_delta++;
                dev->direction = ENCODER_DIR_CW;
            } else {
                /* Counter-clockwise rotation */
                dev->count--;
                dev->pending_delta--;
                dev->direction = ENCODER_DIR_CCW;
            }
        }

        /* Signal data available */
        rt_event_send(&dev->data_evt, ENCODER_EVENT_DATA_FLAG);
    }

    /* Update the last state of CLK for the next comparison */
    dev->last_clk_state = clk_state;
}

/* Interrupt handler for SW (button) pin */
static void encoder_sw_isr(void* args)
{
    struct encoder_device* dev = (struct encoder_device*)args;
    rt_uint8_t             sw_state;
    rt_uint64_t            sw_tick;

    if (!dev || !dev->configured)
        return;

    sw_tick = cpu_ticks_ms();
    if (3 > (sw_tick - dev->button_last_press_time)) {
        return;
    }
    dev->button_last_press_time = sw_tick;

    sw_state = kd_pin_get_dr(dev->sw_pin);

    if (sw_state == GPIO_PV_LOW && !dev->button_pressed) {
        /* Button pressed */
        dev->button_pressed    = 1;
        dev->button_press_time = rt_tick_get();
    } else if (sw_state == GPIO_PV_HIGH && dev->button_pressed) {
        /* Button released */
        dev->button_pressed = 0;
    }

    /* Signal button event */
    rt_event_send(&dev->data_evt, ENCODER_EVENT_DATA_FLAG);
}

static rt_err_t _encoder_dev_control(rt_device_t dev, int cmd, void* args)
{
    rt_err_t ret = RT_EOK;

    struct encoder_device* encoder = (struct encoder_device*)dev;

    switch (cmd) {
    case ENCODER_CMD_GET_DATA: {
        struct encoder_data data;

        /* Use interrupt disable for critical section */
        rt_base_t level = rt_hw_interrupt_disable();

        data.delta        = encoder->pending_delta;
        data.total_count  = encoder->count;
        data.direction    = encoder->direction;
        data.button_state = encoder->button_pressed;
        data.timestamp    = rt_tick_get();

        /* Clear pending delta after reading */
        encoder->pending_delta = 0;
        encoder->direction     = ENCODER_DIR_NONE;

        rt_hw_interrupt_enable(level);

        if (0x00 != LWP_PUT_TO_USER(args, &data, struct encoder_data)) {
            ret = -RT_ERROR;
        }
        break;
    }

    case ENCODER_CMD_RESET: {
        rt_base_t level        = rt_hw_interrupt_disable();
        encoder->count         = 0;
        encoder->pending_delta = 0;
        encoder->direction     = 0;
        rt_hw_interrupt_enable(level);
        break;
    }

    case ENCODER_CMD_SET_COUNT: {
        rt_int64_t new_count;

        if (0x00 != LWP_GET_FROM_USER(&new_count, args, rt_int64_t)) {
            ret = -RT_ERROR;
            break;
        }

        rt_base_t level        = rt_hw_interrupt_disable();
        encoder->count         = new_count;
        encoder->pending_delta = 0;
        rt_hw_interrupt_enable(level);
        break;
    }

    case ENCODER_CMD_CONFIG: {
        struct encoder_dev_cfg_t cfg;

        if (0x00 != LWP_GET_FROM_USER(&cfg, args, struct encoder_dev_cfg_t)) {
            ret = -RT_ERROR;
            break;
        }

        /* Detach old interrupts */
        if (encoder->configured) {
            if (encoder->clk_pin != -1) {
                kd_pin_detach_irq(encoder->clk_pin);
            }

            if (encoder->sw_pin != -1) {
                kd_pin_detach_irq(encoder->sw_pin);
            }
        }

        /* Update pins */
        encoder->clk_pin = cfg.cfg.clk_pin;
        encoder->dt_pin  = cfg.cfg.dt_pin;
        encoder->sw_pin  = cfg.cfg.sw_pin;

        /* Configure new pins */
        if (encoder->dt_pin != -1 && ret == RT_EOK) {
            kd_pin_mode(encoder->dt_pin, GPIO_DM_INPUT_PULLDOWN);
        }

        if (encoder->clk_pin != -1) {
            kd_pin_mode(encoder->clk_pin, GPIO_DM_INPUT_PULLDOWN);
            ret = kd_pin_attach_irq(encoder->clk_pin, GPIO_PE_BOTH, encoder_clk_isr, encoder);
            if (ret == RT_EOK) {
                ret = kd_pin_irq_enable(encoder->clk_pin, KD_GPIO_IRQ_ENABLE);
            }
        }

        if (encoder->sw_pin != -1 && ret == RT_EOK) {
            kd_pin_mode(encoder->sw_pin, GPIO_DM_INPUT_PULLDOWN);
            ret = kd_pin_attach_irq(encoder->sw_pin, GPIO_PE_BOTH, encoder_sw_isr, encoder);
            if (ret == RT_EOK) {
                ret = kd_pin_irq_enable(encoder->sw_pin, KD_GPIO_IRQ_ENABLE);
            }
        }

        if (ret == RT_EOK) {
            encoder->configured = 1;
            LOG_I("Encoder configured: CLK=%d, DT=%d, SW=%d", encoder->clk_pin, encoder->dt_pin, encoder->sw_pin);
        } else {
            encoder->configured = 0;
            LOG_E("Failed to configure encoder");
        }
        break;
    }

    case ENCODER_CMD_WAIT_DATA: {
        rt_int32_t  timeout_ms = 0;
        rt_uint32_t event;

        if (0x00 != LWP_GET_FROM_USER(&timeout_ms, args, rt_int32_t)) {
            ret = -RT_ERROR;
            break;
        }

        rt_tick_t timeout = (timeout_ms == -1) ? RT_WAITING_FOREVER : rt_tick_from_millisecond(timeout_ms);

        if (rt_event_recv(&encoder->data_evt, ENCODER_EVENT_DATA_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout, &event)
            != RT_EOK) {
            ret = -RT_ETIMEOUT;
        }

        break;
    }

    default:
        LOG_E("Unsupported command: %d", cmd);
        ret = -RT_EINVAL;
        break;
    }

    return ret;
}

static rt_err_t _encoder_dev_close(rt_device_t dev)
{
    struct encoder_device* encoder = (struct encoder_device*)dev;

    /* Detach interrupts on close */
    if (encoder->configured) {
        if (encoder->clk_pin != -1) {
            kd_pin_detach_irq(encoder->clk_pin);
        }
        if (encoder->sw_pin != -1) {
            kd_pin_detach_irq(encoder->sw_pin);
        }
        encoder->configured = 0;
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops encoder_dev_ops = {
    .init    = RT_NULL,
    .open    = RT_NULL,
    .close   = _encoder_dev_close,
    .read    = RT_NULL,
    .write   = RT_NULL,
    .control = _encoder_dev_control,
};
#endif

static struct encoder_device* rotary_encoder_device_register(struct encoder_dev_cfg_t* cfg)
{
    rt_err_t ret = RT_EOK;
    char     name[RT_NAME_MAX];

    struct encoder_device* dev = rt_malloc(sizeof(struct encoder_device));
    if (!dev) {
        LOG_E("Failed to allocate memory for encoder device");
        return NULL;
    }
    rt_memset(dev, 0, sizeof(struct encoder_device));

    /* Initialize device structure */
    dev->device.type      = RT_Device_Class_Char;
    dev->device.user_data = NULL;

    /* Initialize pins to invalid state */
    dev->clk_pin    = cfg->cfg.clk_pin;
    dev->dt_pin     = cfg->cfg.dt_pin;
    dev->sw_pin     = cfg->cfg.sw_pin;
    dev->configured = 0;
    dev->index = cfg->index;
    rt_memcpy(&dev->cfg, cfg, sizeof(struct encoder_dev_cfg_t));

    /* Initialize synchronization objects */
    rt_event_init(&dev->data_evt, "enc_sem", RT_IPC_FLAG_PRIO);

#ifdef RT_USING_DEVICE_OPS
    dev->device.ops = &encoder_dev_ops;
#else
    dev->device.init    = RT_NULL;
    dev->device.open    = RT_NULL;
    dev->device.close   = _encoder_dev_close;
    dev->device.read    = RT_NULL;
    dev->device.write   = RT_NULL;
    dev->device.control = _encoder_dev_control;
#endif

    rt_snprintf(name, sizeof(name), "encoder%d", cfg->index);
    /* Register device */
    ret = rt_device_register(&dev->device, name, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK) {
        LOG_E("Failed to register encoder device");
        rt_event_detach(&dev->data_evt);
        rt_free(dev);
        return NULL;
    }

    LOG_I("Rotary encoder device initialized");

    return dev;
}

static volatile int encoder_list_flag = 0;
static rt_list_t    encoder_dev_list  = RT_LIST_OBJECT_INIT(encoder_dev_list);

int encoder_dev_create(struct encoder_dev_cfg_t* cfg)
{
    struct encoder_device *dev = NULL, *temp_dev = NULL;

    if (!cfg) {
        return -1;
    }

    while (encoder_list_flag) {
        rt_thread_mdelay(1);
    }
    encoder_list_flag = 1;

    rt_list_for_each_entry(temp_dev, &encoder_dev_list, list)
    {
        if (temp_dev->index == cfg->index) {
            dev = temp_dev;
            break;
        }
    }

    if (NULL == dev) {
        if (NULL == (dev = rotary_encoder_device_register(cfg))) {
            encoder_list_flag = 0;

            return RT_ERROR;
        }
        rt_list_insert_after(&encoder_dev_list, &dev->list);
    }
    encoder_list_flag = 0;

    return rt_device_control(&dev->device, ENCODER_CMD_CONFIG, cfg);
}

int encoder_dev_delete(int index)
{
    struct encoder_device *dev = NULL, *temp_dev = NULL;

    while (encoder_list_flag) {
        rt_thread_mdelay(1);
    }
    encoder_list_flag = 1;

    rt_list_for_each_entry_safe(dev, temp_dev, &encoder_dev_list, list)
    {
        if (dev->index == index) {
            rt_device_close(&dev->device);
            rt_device_unregister(&dev->device);
            rt_event_detach(&dev->data_evt);

            rt_list_remove(&dev->list);

            rt_free(dev);

            encoder_list_flag = 0;

            return RT_EOK;
        }
    }

    encoder_list_flag = 0;

    return RT_ERROR;
}
