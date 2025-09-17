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
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>

#include "lwp.h"
#include "lwp_arch.h"
#include "lwp_user_mm.h"
#include "rtdef.h"

#include "drv_gpio.h"
#include "rotary_encoder.h"

#define DBG_TAG "rotary_encoder"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define USE_DT_ISR (0)

struct encoder_device {
    struct rt_device device;

    /* GPIO pins */
    rt_base_t clk_pin;
    rt_base_t dt_pin;
    rt_base_t sw_pin;

    /* State tracking */
    rt_int64_t count;
    rt_int32_t pending_delta;
    rt_uint8_t last_clk_state;
#if USE_DT_ISR
    rt_uint8_t last_dt_state;
#endif
    rt_uint8_t direction;  /* 1: CW, 0xFF: CCW, 0: no movement */

    /* Button state */
    rt_uint8_t button_pressed;
    rt_tick_t button_press_time;

    /* Synchronization */
    rt_base_t level;  /* For interrupt level */
    struct rt_semaphore data_sem;

    /* Configuration state */
    rt_uint8_t configured;
};

static struct encoder_device *g_encoder_dev = RT_NULL;

/* Interrupt handler for CLK pin */
static void encoder_clk_isr(void *args)
{
    struct encoder_device *dev = (struct encoder_device *)args;
    rt_uint8_t clk_state, dt_state;

    if (!dev || !dev->configured) return;

    /* Read current pin states */
    clk_state = kd_pin_get_dr(dev->clk_pin);
    dt_state = kd_pin_get_dr(dev->dt_pin);

    /* Detect rising edge on CLK */
    if (clk_state != dev->last_clk_state && clk_state == GPIO_PV_HIGH) {
        /* Determine direction based on DT state */
        if (dt_state != clk_state) {
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

        /* Signal data available */
        rt_sem_release(&dev->data_sem);
    }

    dev->last_clk_state = clk_state;
}

#if USE_DT_ISR
/* Interrupt handler for DT pin - for better accuracy */
static void encoder_dt_isr(void *args)
{
    struct encoder_device *dev = (struct encoder_device *)args;
    rt_uint8_t clk_state, dt_state;

    if (!dev || !dev->configured) return;

    /* Read current pin states */
    clk_state = kd_pin_get_dr(dev->clk_pin);
    dt_state = kd_pin_get_dr(dev->dt_pin);

    /* Detect rising edge on DT */
    if (dt_state != dev->last_dt_state && dt_state == GPIO_PV_HIGH) {
        /* Determine direction based on CLK state */
        if (clk_state == dt_state) {
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

        /* Signal data available */
        rt_sem_release(&dev->data_sem);
    }

    dev->last_dt_state = dt_state;
}
#endif

/* Interrupt handler for SW (button) pin */
static void encoder_sw_isr(void *args)
{
    struct encoder_device *dev = (struct encoder_device *)args;
    rt_uint8_t sw_state;

    if (!dev || !dev->configured) return;

    sw_state = kd_pin_read(dev->sw_pin);

    if (sw_state == GPIO_PV_LOW && !dev->button_pressed) {
        /* Button pressed */
        dev->button_pressed = 1;
        dev->button_press_time = rt_tick_get();
    } else if (sw_state == GPIO_PV_HIGH && dev->button_pressed) {
        /* Button released */
        dev->button_pressed = 0;
    }

    /* Signal button event */
    rt_sem_release(&dev->data_sem);
}

static rt_err_t _encoder_dev_control(rt_device_t dev, int cmd, void *args)
{
    struct encoder_device *encoder = (struct encoder_device *)dev;
    rt_err_t ret = RT_EOK;
    int pid = lwp_getpid();

    switch (cmd) {
    case ENCODER_CMD_GET_DATA:
        {
            struct encoder_data data;

            /* Use interrupt disable for critical section */
            rt_base_t level = rt_hw_interrupt_disable();

            data.delta = encoder->pending_delta;
            data.total_count = encoder->count;
            data.direction = encoder->direction;
            data.button_state = encoder->button_pressed;
            data.timestamp = rt_tick_get();

            /* Clear pending delta after reading */
            encoder->pending_delta = 0;

            rt_hw_interrupt_enable(level);

            /* Copy to user space if needed */
            if (pid != 0) {
                if (sizeof(data) != lwp_put_to_user(args, &data, sizeof(data))) {
                    LOG_E("Failed to copy data to user space");
                    ret = -RT_ERROR;
                }
            } else {
                rt_memcpy(args, &data, sizeof(data));
            }
            break;
        }

    case ENCODER_CMD_RESET:
        {
            rt_base_t level = rt_hw_interrupt_disable();
            encoder->count = 0;
            encoder->pending_delta = 0;
            encoder->direction = 0;
            rt_hw_interrupt_enable(level);
            break;
        }

    case ENCODER_CMD_SET_COUNT:
        {
            rt_int64_t new_count;

            if (pid != 0) {
                if (sizeof(new_count) != lwp_get_from_user(&new_count, args, sizeof(new_count))) {
                    LOG_E("Failed to get count from user space");
                    ret = -RT_ERROR;
                    break;
                }
            } else {
                new_count = *(rt_int64_t *)args;
            }

            rt_base_t level = rt_hw_interrupt_disable();
            encoder->count = new_count;
            encoder->pending_delta = 0;
            rt_hw_interrupt_enable(level);
            break;
        }

    case ENCODER_CMD_CONFIG:
        {
            struct encoder_config config;

            if (pid != 0) {
                if (sizeof(config) != lwp_get_from_user(&config, args, sizeof(config))) {
                    LOG_E("Failed to get config from user space");
                    ret = -RT_ERROR;
                    break;
                }
            } else {
                rt_memcpy(&config, args, sizeof(config));
            }

            /* Detach old interrupts */
            if (encoder->configured) {
                if (encoder->clk_pin != -1) {
                    kd_pin_detach_irq(encoder->clk_pin);
                }
#if USE_DT_ISR
                if (encoder->dt_pin != -1) {
                    kd_pin_detach_irq(encoder->dt_pin);
                }
#endif
                if (encoder->sw_pin != -1) {
                    kd_pin_detach_irq(encoder->sw_pin);
                }
            }

            /* Update pins */
            encoder->clk_pin = config.clk_pin;
            encoder->dt_pin = config.dt_pin;
            encoder->sw_pin = config.sw_pin;

            /* Configure new pins */
            if (encoder->clk_pin != -1) {
                kd_pin_mode(encoder->clk_pin, GPIO_DM_INPUT_PULLUP);
                ret = kd_pin_attach_irq(encoder->clk_pin, GPIO_PE_BOTH,
                                        encoder_clk_isr, encoder);
                if (ret == RT_EOK) {
                    ret = kd_pin_irq_enable(encoder->clk_pin, KD_GPIO_IRQ_ENABLE);
                }
                encoder->last_clk_state = kd_pin_read(encoder->clk_pin);
            }

            if (encoder->dt_pin != -1 && ret == RT_EOK) {
                kd_pin_mode(encoder->dt_pin, GPIO_DM_INPUT_PULLUP);
#if USE_DT_ISR
                ret = kd_pin_attach_irq(encoder->dt_pin, GPIO_PE_BOTH,
                                        encoder_dt_isr, encoder);
                if (ret == RT_EOK) {
                    ret = kd_pin_irq_enable(encoder->dt_pin, KD_GPIO_IRQ_ENABLE);
                }

                encoder->last_dt_state = kd_pin_read(encoder->dt_pin);
#endif
            }

            if (encoder->sw_pin != -1 && ret == RT_EOK) {
                kd_pin_mode(encoder->sw_pin, GPIO_DM_INPUT_PULLUP);
                ret = kd_pin_attach_irq(encoder->sw_pin, GPIO_PE_BOTH,
                                        encoder_sw_isr, encoder);
                if (ret == RT_EOK) {
                    ret = kd_pin_irq_enable(encoder->sw_pin, KD_GPIO_IRQ_ENABLE);
                }
            }

            if (ret == RT_EOK) {
                encoder->configured = 1;
                LOG_I("Encoder configured: CLK=%d, DT=%d, SW=%d",
                      encoder->clk_pin, encoder->dt_pin, encoder->sw_pin);
            } else {
                encoder->configured = 0;
                LOG_E("Failed to configure encoder");
            }
            break;
        }

    case ENCODER_CMD_WAIT_DATA:
        {
            rt_int32_t timeout_ms = 0;

            if (args) {
                if (pid != 0) {
                    if (sizeof(timeout_ms) != lwp_get_from_user(&timeout_ms, args, sizeof(timeout_ms))) {
                        LOG_E("Failed to get timeout from user space");
                        ret = -RT_ERROR;
                        break;
                    }
                } else {
                    timeout_ms = *(rt_int32_t *)args;
                }
            }

            rt_tick_t timeout = (timeout_ms == 0) ? RT_WAITING_FOREVER :
                rt_tick_from_millisecond(timeout_ms);

            if (rt_sem_take(&encoder->data_sem, timeout) != RT_EOK) {
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
    struct encoder_device *encoder = (struct encoder_device *)dev;

    /* Detach interrupts on close */
    if (encoder->configured) {
        if (encoder->clk_pin != -1) {
            kd_pin_detach_irq(encoder->clk_pin);
        }
#if USE_DT_ISR
        if (encoder->dt_pin != -1) {
            kd_pin_detach_irq(encoder->dt_pin);
        }
#endif
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

static int rotary_encoder_device_init(void)
{
    rt_err_t ret = RT_EOK;

    if (g_encoder_dev) {
        return 0;  /* Already initialized */
    }

    g_encoder_dev = rt_malloc(sizeof(struct encoder_device));
    if (!g_encoder_dev) {
        LOG_E("Failed to allocate memory for encoder device");
        return -RT_ENOMEM;
    }

    rt_memset(g_encoder_dev, 0, sizeof(struct encoder_device));

    /* Initialize device structure */
    g_encoder_dev->device.type = RT_Device_Class_Char;
    g_encoder_dev->device.user_data = g_encoder_dev;

    /* Initialize pins to invalid state */
    g_encoder_dev->clk_pin = -1;
    g_encoder_dev->dt_pin = -1;
    g_encoder_dev->sw_pin = -1;
    g_encoder_dev->configured = 0;

    /* Initialize synchronization objects */
    rt_sem_init(&g_encoder_dev->data_sem, "enc_sem", 0, RT_IPC_FLAG_PRIO);

#ifdef RT_USING_DEVICE_OPS
    g_encoder_dev->device.ops = &encoder_dev_ops;
#else
    g_encoder_dev->device.init    = RT_NULL;
    g_encoder_dev->device.open    = RT_NULL;
    g_encoder_dev->device.close   = _encoder_dev_close;
    g_encoder_dev->device.read    = RT_NULL;
    g_encoder_dev->device.write   = RT_NULL;
    g_encoder_dev->device.control = _encoder_dev_control;
#endif

    /* Register device */
    ret = rt_device_register(&g_encoder_dev->device, "encoder", RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK) {
        LOG_E("Failed to register encoder device");
        rt_sem_detach(&g_encoder_dev->data_sem);
        rt_free(g_encoder_dev);
        g_encoder_dev = RT_NULL;
        return ret;
    }

    LOG_I("Rotary encoder device initialized");
    return 0;
}
INIT_APP_EXPORT(rotary_encoder_device_init);

#if 0
#include <finsh.h>

static void encoder_demo_handler(void *args)
{
    rt_kprintf(">> Encoder event!\n");
}

static void encoder_cmd(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage:\n");
        rt_kprintf("  encoder config <clk> <dt> <sw>\n");
        rt_kprintf("  encoder read\n");
        rt_kprintf("  encoder reset\n");
        rt_kprintf("  encoder test\n");
        return;
    }

    if (!g_encoder_dev) {
        rt_kprintf("Encoder device not initialized\n");
        return;
    }

    const char *cmd = argv[1];

    if (!strcmp(cmd, "config") && argc == 5) {
        struct encoder_config config;
        config.clk_pin = atoi(argv[2]);
        config.dt_pin = atoi(argv[3]);
        config.sw_pin = atoi(argv[4]);

        if (_encoder_dev_control(&g_encoder_dev->device, ENCODER_CMD_CONFIG, &config) == RT_EOK) {
            rt_kprintf("Encoder configured: CLK=%d, DT=%d, SW=%d\n",
                       config.clk_pin, config.dt_pin, config.sw_pin);
        } else {
            rt_kprintf("Failed to configure encoder\n");
        }
    } else if (!strcmp(cmd, "read")) {
        struct encoder_data data;
        if (_encoder_dev_control(&g_encoder_dev->device, ENCODER_CMD_GET_DATA, &data) == RT_EOK) {
            rt_kprintf("Count: %ld, Delta: %d, Direction: %s, Button: %s\n",
                       data.total_count, data.delta,
                       data.direction == ENCODER_DIR_CW ? "CW" :
                       (data.direction == ENCODER_DIR_CCW ? "CCW" : "None"),
                       data.button_state ? "Pressed" : "Released");
        }
    } else if (!strcmp(cmd, "reset")) {
        if (_encoder_dev_control(&g_encoder_dev->device, ENCODER_CMD_RESET, NULL) == RT_EOK) {
            rt_kprintf("Encoder reset\n");
        }
    } else if (!strcmp(cmd, "test")) {
        rt_kprintf("Encoder test mode - rotate encoder to see changes\n");
        struct encoder_data data;
        for (int i = 0; i < 20; i++) {
            rt_thread_mdelay(500);
            if (_encoder_dev_control(&g_encoder_dev->device, ENCODER_CMD_GET_DATA, &data) == RT_EOK) {
                if (data.delta != 0) {
                    rt_kprintf("[%d] Count: %ld, Delta: %d\n", i, data.total_count, data.delta);
                }
            }
        }
    } else {
        rt_kprintf("Unknown command\n");
    }
}
MSH_CMD_EXPORT_ALIAS(encoder_cmd, encoder, rotary encoder test command);
#endif

