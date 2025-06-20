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
#include <stdbool.h>
#include <stdint.h>

#include <rtdef.h>
#include <rtdevice.h>
#include <rtthread.h>

#include "board.h"
#include "ioremap.h"
#include "riscv_io.h"

#include "drv_pwm.h"
#include "sysctl_clk.h"

#ifdef RT_DEBUG
#define DBG_LVL DBG_LOG
#else
#define DBG_LVL DBG_WARNING
#endif

#define DBG_COLOR
#define DBG_TAG "pwm"
#include <rtdbg.h>

/* PWM Register Definition */
typedef struct {
    uint32_t scale : 4; /* Clock scale factor */
    uint32_t reserve : 4;
    uint32_t sticky : 1; /* Sticky counter */
    uint32_t zerocmp : 1; /* Zero compare */
    uint32_t deglitch : 1; /* Deglitch enable */
    uint32_t reserve1 : 1;
    uint32_t enalways : 1; /* Always enable */
    uint32_t enoneshot : 1; /* One-shot mode */
    uint32_t reserve2 : 2;
    uint32_t cmp0center : 1; /* Center-aligned for channel 0 */
    uint32_t cmp1center : 1; /* Center-aligned for channel 1 */
    uint32_t cmp2center : 1; /* Center-aligned for channel 2 */
    uint32_t cmp3center : 1; /* Center-aligned for channel 3 */
    uint32_t reserve3 : 4;
    uint32_t cmp0gang : 1; /* Channel 0 gang mode */
    uint32_t cmp1gang : 1; /* Channel 1 gang mode */
    uint32_t cmp2gang : 1; /* Channel 2 gang mode */
    uint32_t cmp3gang : 1; /* Channel 3 gang mode */
    uint32_t cmp0ip : 1; /* Channel 0 interrupt pending */
    uint32_t cmp1ip : 1; /* Channel 1 interrupt pending */
    uint32_t cmp2ip : 1; /* Channel 2 interrupt pending */
    uint32_t cmp3ip : 1; /* Channel 3 interrupt pending */
} pwm_cfg_t;

typedef struct {
    volatile uint32_t pwmcfg; /* 0x00: PWM Configuration */
    volatile uint32_t reserved0;
    volatile uint32_t pwmcount; /* 0x08: PWM Counter */
    volatile uint32_t reserved1;
    volatile uint32_t pwms; /* 0x10: PWM Scaled Counter */
    volatile uint32_t reserved2[3];
    volatile uint32_t pwmcmp0; /* 0x20: PWM Compare 0 */
    volatile uint32_t pwm_chn_pulse[3]; /* 0x24-0x2C: Channel Pulse Width */
} kd_pwm_t;

/* PWM Instance Structure */
struct pwm_inst {
    volatile kd_pwm_t* reg; /* PWM register base */
    uint32_t           period; /* Current period in clock cycles */
    uint32_t           scale; /* Current clock scale factor */
    uint32_t           enable[3]; /* Channel enable status */
    uint32_t           pulses[3]; /* Current pulse width in clock cycles */
};

/* PWM Device Wrapper */
struct pwm_inst_wrap {
    struct rt_device_pwm device; /* RT-Thread PWM device */
    void*                inst_type; /* Instance type identifier */
    struct pwm_inst      pwm[2]; /* Two PWM instances (0 and 1) */
};

static uint32_t _pwm_dev_inst_type;

/* PWM Channel Mapping */
#define PWM_CHANNEL_TO_INST(ch)  ((ch) / 3) /* 0-2: instance 0, 3-5: instance 1 */
#define PWM_CHANNEL_TO_SUBCH(ch) ((ch) % 3) /* Sub-channel within instance */

/**
 * @brief Start PWM output on specified channel
 * @param inst PWM instance
 * @param channel Channel number (0-2)
 * @return RT_EOK on success, error code on failure
 */
static int pwm_start(struct pwm_inst* inst, int channel)
{
    if (channel < 0 || channel > 2) {
        LOG_E("Invalid channel %d", channel);
        return -RT_EINVAL;
    }

    uint32_t period = inst->period;
    uint32_t scale  = inst->scale;
    uint32_t pulse  = inst->pulses[channel];

    /* Configure PWM scale and period */
    inst->reg->pwmcfg  = (inst->reg->pwmcfg & (~0x0F)) | scale;
    inst->reg->pwmcmp0 = period >> scale;

    /* Set pulse width for the channel */
    inst->reg->pwm_chn_pulse[channel] = (period - pulse) >> scale;

    /* Enable PWM */
    inst->reg->pwmcfg |= (1 << 12); /* Enable bit */
    inst->enable[channel] = 1;

    return RT_EOK;
}

/**
 * @brief Stop PWM output on specified channel
 * @param inst PWM instance
 * @param channel Channel number (0-2)
 * @return RT_EOK on success, error code on failure
 */
static int pwm_stop(struct pwm_inst* inst, int channel)
{
    if (channel < 0 || channel > 2) {
        LOG_E("Invalid channel %d", channel);
        return -RT_EINVAL;
    }

    inst->enable[channel] = 0;

    inst->reg->pwm_chn_pulse[channel] = UINT32_MAX;

    /* Disable PWM if no channels are active */
    if (!inst->enable[0] && !inst->enable[1] && !inst->enable[2]) {
        inst->reg->pwmcount = UINT32_MAX;

        rt_thread_delay(1);
        inst->reg->pwmcfg &= ~(1 << 12);
    }

    return RT_EOK;
}

/**
 * @brief Get current PWM configuration
 * @param inst PWM instance
 * @param channel Channel number (0-2)
 * @param configuration Pointer to configuration structure
 * @return RT_EOK on success, error code on failure
 */
static rt_err_t kd_pwm_get(struct pwm_inst* inst, rt_uint8_t channel, struct rt_pwm_configuration* configuration)
{
    if (!inst || !configuration || channel > 2) {
        return -RT_EINVAL;
    }

    uint64_t pulse, period;
    uint32_t pwm_pclock = sysctl_clk_get_leaf_freq(SYSCTL_CLK_PWM_PCLK_GATE);

    if (inst->enable[channel]) {
        uint32_t scale = inst->reg->pwmcfg & 0x0f;
        pwm_pclock >>= scale;
        period = inst->reg->pwmcmp0;
        pulse  = period - inst->reg->pwm_chn_pulse[channel];
    } else {
        period = inst->period;
        pulse  = inst->pulses[channel];
    }

    /* Convert to nanoseconds */
    configuration->period = period * NSEC_PER_SEC / pwm_pclock;
    configuration->pulse  = pulse * NSEC_PER_SEC / pwm_pclock;

    return RT_EOK;
}

/**
 * @brief Set PWM configuration
 * @param inst PWM instance
 * @param channel Channel number (0-2)
 * @param configuration Pointer to configuration structure
 * @return RT_EOK on success, error code on failure
 */
static int kd_pwm_set(struct pwm_inst* inst, int channel, struct rt_pwm_configuration* configuration)
{
    if (!inst || !configuration || channel > 2) {
        return -RT_EINVAL;
    }

    uint64_t pulse, period, pwmcmpx_max;
    uint32_t pwm_pclock = sysctl_clk_get_leaf_freq(SYSCTL_CLK_PWM_PCLK_GATE);
    uint32_t pwmscale   = 0;

    /* Convert from nanoseconds to clock cycles */
    pulse  = (uint64_t)configuration->pulse * pwm_pclock / NSEC_PER_SEC;
    period = (uint64_t)configuration->period * pwm_pclock / NSEC_PER_SEC;

    /* Validate parameters */
    if (pulse > period || period > ((1 << (15 + 16)) - 1LL)) {
        LOG_E("Invalid configuration for channel %d (pulse=%llu, period=%llu)", channel, pulse, period);
        return -RT_EINVAL;
    }

    /* Calculate appropriate scale factor */
    pwmcmpx_max = (1 << 16) - 1;
    while ((period >> pwmscale) > pwmcmpx_max) {
        pwmscale++;

        if (pwmscale > 0xF) {
            LOG_E("Period too large for channel %d", channel);
            return -RT_EINVAL;
        }
    }

    /* Update hardware if channel is active */
    if (inst->enable[channel]) {
        if (period != inst->period) {
            inst->reg->pwmcfg  = (inst->reg->pwmcfg & (~0x0F)) | pwmscale;
            inst->reg->pwmcmp0 = period >> pwmscale;
        }

        inst->reg->pwm_chn_pulse[channel] = (period - pulse) >> pwmscale;
    }

    /* Store current settings */
    inst->scale           = pwmscale;
    inst->period          = period;
    inst->pulses[channel] = pulse;

    return RT_EOK;
}

/**
 * @brief Get PWM channel state
 * @param inst PWM instance
 * @param channel Channel number (0-2)
 * @return RT_EOK on success, error code on failure
 */
static int kd_pwm_get_state(struct pwm_inst* inst, int channel)
{
    if (channel < 0 || channel > 2) {
        LOG_E("Invalid channel %d", channel);
        return -RT_EINVAL;
    }

    if (inst->enable[channel]) {
        return RT_EOK;
    }

    return RT_ERROR;
}

/**
 * @brief PWM control function
 * @param device PWM device pointer
 * @param cmd Command to execute
 * @param arg Command argument
 * @return RT_EOK on success, error code on failure
 */
static rt_err_t kd_pwm_control(struct rt_device_pwm* device, int cmd, void* arg)
{
    if (!device || !arg) {
        return -RT_EINVAL;
    }

    struct rt_pwm_configuration* configuration = (struct rt_pwm_configuration*)arg;
    struct pwm_inst_wrap*        pwm_inst_wrap = (struct pwm_inst_wrap*)(device->parent.user_data);

    /* Validate instance type */
    if (&_pwm_dev_inst_type != pwm_inst_wrap->inst_type) {
        return -RT_EINVAL;
    }

    /* Validate channel number */
    if (configuration->channel < 0 || configuration->channel > 5) {
        LOG_E("Invalid channel %d", configuration->channel);
        return -RT_EINVAL;
    }

    /* Map channel to instance and sub-channel */
    uint32_t         inst_idx = PWM_CHANNEL_TO_INST(configuration->channel);
    uint32_t         sub_ch   = PWM_CHANNEL_TO_SUBCH(configuration->channel);
    struct pwm_inst* pwm_inst = &pwm_inst_wrap->pwm[inst_idx];

    /* Execute command */
    switch (cmd) {
    case PWM_CMD_ENABLE:
    case KD_PWM_CMD_ENABLE:
        return pwm_start(pwm_inst, sub_ch);

    case PWM_CMD_DISABLE:
    case KD_PWM_CMD_DISABLE:
        return pwm_stop(pwm_inst, sub_ch);

    case PWM_CMD_SET:
    case KD_PWM_CMD_SET_CFG:
        return kd_pwm_set(pwm_inst, sub_ch, configuration);

    case PWM_CMD_GET:
    case KD_PWM_CMD_GET_CFG:
        return kd_pwm_get(pwm_inst, sub_ch, configuration);

    case KD_PWM_CMD_GET_STAT:
        return kd_pwm_get_state(pwm_inst, sub_ch);

    default:
        LOG_W("Unsupported command %d", cmd);
        return -RT_EINVAL;
    }
}

/* PWM Driver Operations */
static struct rt_pwm_ops drv_ops = { .control = kd_pwm_control };

/**
 * @brief Initialize PWM hardware
 * @return RT_EOK on success, error code on failure
 */
int rt_hw_pwm_init(void)
{
    /* Check if already registered */
    if (rt_device_find("pwm")) {
        LOG_E("PWM device already registered");
        return -RT_ERROR;
    }

    /* Map PWM registers */
    void* reg = rt_ioremap((void*)PWM_BASE_ADDR, PWM_IO_SIZE);
    if (!reg) {
        LOG_E("PWM ioremap failed");
        return -RT_ERROR;
    }

    /* Initialize PWM wrapper */
    static struct pwm_inst_wrap pwm_inst_wrap;
    rt_memset(&pwm_inst_wrap, 0, sizeof(pwm_inst_wrap));
    pwm_inst_wrap.inst_type = &_pwm_dev_inst_type;

    /* Initialize PWM instance 0 (channels 0-2) */
    pwm_inst_wrap.pwm[0].reg         = (kd_pwm_t*)((char*)reg + 0x00);
    pwm_inst_wrap.pwm[0].reg->pwmcfg = (1 << 9) | (1 << 10); /* deglitch and enalways and cmpxcenter */

    /* Initialize PWM instance 1 (channels 3-5) */
    pwm_inst_wrap.pwm[1].reg         = (kd_pwm_t*)((char*)reg + 0x40);
    pwm_inst_wrap.pwm[1].reg->pwmcfg = (1 << 9) | (1 << 10); /* deglitch and enalways and cmpxcenter */

    /* Register PWM device */
    pwm_inst_wrap.device.ops = &drv_ops;
    if (rt_device_pwm_register(&pwm_inst_wrap.device, "pwm", &drv_ops, &pwm_inst_wrap) != RT_EOK) {
        LOG_E("PWM device registration failed");
        rt_iounmap(reg);
        return -RT_ERROR;
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_pwm_init);
