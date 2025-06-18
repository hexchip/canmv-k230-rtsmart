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
#include <rtthread.h>
#include <rtdevice.h>
#include "riscv_io.h"
#include "board.h"
#include "ioremap.h"
#include <rtdbg.h>
#include <stdbool.h>
#include "sysctl_clk.h"
#include "drv_pwm.h"

#define DBG_TAG "pwm"
#ifdef RT_DEBUG
#define DBG_LVL DBG_LOG
#else
#define DBG_LVL DBG_WARNING
#endif
#define DBG_COLOR
#include <rtdbg.h>

typedef struct {
    uint8_t used;
    uint8_t ach[2];
    uint8_t scale;
    uint32_t period;
    uint32_t pulse;
    uint32_t* pulse_reg;
} channel_cfg_t;

static struct rt_device_pwm kd_pwm;
static channel_cfg_t ch_cfg[6];

static int pwm_start(kd_pwm_t* reg, int channel)
{
    if ((ch_cfg[ch_cfg[channel].ach[0]].used && ch_cfg[ch_cfg[channel].ach[0]].period != ch_cfg[channel].period) ||
        (ch_cfg[ch_cfg[channel].ach[1]].used && ch_cfg[ch_cfg[channel].ach[1]].period != ch_cfg[channel].period)) {
        LOG_E("channel %d configure conflict", channel);
        return -RT_EBUSY;
    }

    if (channel > 2)
        reg = (kd_pwm_t*)((void*)reg + 0x40);
    ch_cfg[channel].used = 1;
    reg->pwmcfg = reg->pwmcfg & ~0xf | ch_cfg[channel].scale;
    reg->pwmcmp0 = ch_cfg[channel].period >> ch_cfg[channel].scale;
    *ch_cfg[channel].pulse_reg = (ch_cfg[channel].period - ch_cfg[channel].pulse) >> ch_cfg[channel].scale;
    reg->pwmcfg |= (1 << 12);

    return RT_EOK;
}

static int pwm_stop(kd_pwm_t* reg, int channel)
{
    *ch_cfg[channel].pulse_reg = (*ch_cfg[channel].pulse_reg & 0x8000) ? -1 : 0;
    ch_cfg[channel].used = 0;
    if (!(ch_cfg[ch_cfg[channel].ach[0]].used || ch_cfg[ch_cfg[channel].ach[1]].used)) {
        if (channel > 2)
            reg = (kd_pwm_t*)((void*)reg + 0x40);
        reg->pwmcfg &= ~(1 << 12);
    }

    return RT_EOK;
}

static rt_err_t kd_pwm_get(kd_pwm_t* reg, rt_uint8_t channel, struct rt_pwm_configuration* configuration)
{
    int ret;
    uint64_t pulse, period;
    uint32_t pwm_pclock, pwmscale;

    pwm_pclock = sysctl_clk_get_leaf_freq(SYSCTL_CLK_PWM_PCLK_GATE);

    if (ch_cfg[channel].used) {
        if (channel > 2)
            reg = (kd_pwm_t*)((void*)reg + 0x40);
        pwmscale = reg->pwmcfg & 0xf;
        period = reg->pwmcmp0;
        pulse = *ch_cfg[channel].pulse_reg;
        pulse = period - pulse;
        pwm_pclock >>= pwmscale;
    } else {
        period = ch_cfg[channel].period;
        pulse = ch_cfg[channel].pulse;
    }

    period = period * NSEC_PER_SEC / pwm_pclock;
    pulse = pulse * NSEC_PER_SEC / pwm_pclock;

    configuration->period = period;
    configuration->pulse = pulse;

    return RT_EOK;
}

static int kd_pwm_set(kd_pwm_t* reg, int channel, struct rt_pwm_configuration* configuration)
{
    int ret;
    uint64_t pulse, period, pwmcmpx_max;
    uint32_t pwm_pclock, pwmscale = 0;

    pwm_pclock = sysctl_clk_get_leaf_freq(SYSCTL_CLK_PWM_PCLK_GATE);
    pulse = (uint64_t)configuration->pulse * pwm_pclock / NSEC_PER_SEC;
    period = (uint64_t)configuration->period * pwm_pclock / NSEC_PER_SEC;
    if ((pulse > period) || (period > ((1 << (15 + 16)) - 1LL))) {
        LOG_E("channel %d configure is invalid", channel);
        return -RT_EINVAL;
    }

    if ((ch_cfg[channel].used) && (period != ch_cfg[channel].period) &&
        (ch_cfg[ch_cfg[channel].ach[0]].used || ch_cfg[ch_cfg[channel].ach[1]].used)) {
        LOG_E("channel %d configure conflict", channel);
        return -RT_EBUSY;
    }

    pwmcmpx_max = (1 << 16) - 1;
    while ((period >> pwmscale) > pwmcmpx_max)
        pwmscale++;

    ch_cfg[channel].scale = pwmscale;
    ch_cfg[channel].period = period;
    ch_cfg[channel].pulse = pulse;

    if (ch_cfg[channel].used) {
        if (channel > 2)
            reg = (kd_pwm_t*)((void*)reg + 0x40);
        if (period != ch_cfg[channel].period) {
            reg->pwmcfg = reg->pwmcfg & ~0xf | pwmscale;
            reg->pwmcmp0 = period >> pwmscale;
        }
        *ch_cfg[channel].pulse_reg = (period - pulse) >> pwmscale;
    }

    return RT_EOK;
}

static rt_err_t kd_pwm_control(struct rt_device_pwm* device, int cmd, void* arg)
{
    int ret;
    struct rt_pwm_configuration* configuration = (struct rt_pwm_configuration*)arg;
    kd_pwm_t* reg;
    rt_uint32_t channel;

    reg = (kd_pwm_t*)(device->parent.user_data);
    channel = configuration->channel;

    if (channel < 0 || channel > 5) {
        LOG_E("channel %d is invalid\n", channel);
        return -RT_EINVAL;
    }

    switch (cmd) {
    case PWM_CMD_ENABLE:
    case KD_PWM_CMD_ENABLE:
        ret = pwm_start(reg, channel);
        break;
    case PWM_CMD_DISABLE:
    case KD_PWM_CMD_DISABLE:
        ret = pwm_stop(reg, channel);
        break;
    case PWM_CMD_SET:
    case KD_PWM_CMD_SET:
        ret = kd_pwm_set(reg, channel, configuration);
        break;
    case PWM_CMD_GET:
    case KD_PWM_CMD_GET:
        ret = kd_pwm_get(reg, channel, configuration);
        break;
    default:
        return -RT_EINVAL;
    }

    return ret;
}

static struct rt_pwm_ops drv_ops = {
    kd_pwm_control
};

int rt_hw_pwm_init(void)
{
    void* reg;
    kd_pwm_t *pwm0, *pwm1;

    reg = rt_ioremap((void*)PWM_BASE_ADDR, PWM_IO_SIZE);
    kd_pwm.ops = &drv_ops;
    rt_device_pwm_register(&kd_pwm, "pwm", &drv_ops, reg);

    pwm0 = (kd_pwm_t*)((void*)reg + 0);
    pwm1 = (kd_pwm_t*)((void*)reg + 0x40);

    pwm0->pwmcfg = (1 << 9) | (1 << 10);
    pwm1->pwmcfg = (1 << 9) | (1 << 10);

    for (int i = 0; i < 3; i++) {
        ch_cfg[i].used = 0;
        ch_cfg[i].period = 0;
        ch_cfg[i].pulse = 0;
        ch_cfg[i].ach[0] = (i + 1) % 3;
        ch_cfg[i].ach[1] = (i + 2) % 3;
        ch_cfg[i].pulse_reg = &pwm0->pwmcmp1 + (i % 3);
        *ch_cfg[i].pulse_reg = 0;
    }
    for (int i = 3; i < 6; i++) {
        ch_cfg[i].used = 0;
        ch_cfg[i].period = 0;
        ch_cfg[i].pulse = 0;
        ch_cfg[i].ach[0] = (i + 1) % 6;
        ch_cfg[i].ach[1] = (i + 2) % 6;
        ch_cfg[i].pulse_reg = &pwm1->pwmcmp1 + (i % 3);
        *ch_cfg[i].pulse_reg = 0;
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_pwm_init);
