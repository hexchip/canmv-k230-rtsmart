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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rtthread.h>

#include "lwp.h"
#include "lwp_arch.h"
#include "lwp_user_mm.h"
#include "rtdef.h"

#include "ws2812.h"

#define DBG_TAG "ws2812"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_err_t _ws2812_dev_control(rt_device_t dev, int cmd, void* args)
{
    rt_err_t ret = RT_EOK;

    if (cmd == WS2812_IOCTL_STREAM) {
        int                   free_stream = 0;
        struct ws2812_stream* stream      = NULL;

        if (lwp_in_user_space(args)) {
            uint32_t stream_size;
            void*    stream_buffer = NULL;

            if (sizeof(uint32_t) != lwp_get_from_user(&stream_size, args, sizeof(uint32_t))) {
                LOG_E("Failed to get stream size from user space");
                return RT_EINVAL;
            }

            if (0x00 == stream_size) {
                LOG_E("Stream size is zero");
                return RT_EINVAL;
            }

            stream_buffer = rt_malloc_align(stream_size, RT_CPU_CACHE_LINE_SZ);
            if (!stream_buffer) {
                LOG_E("Failed to allocate memory for stream buffer");
                return RT_ENOMEM;
            }

            if (stream_size != lwp_get_from_user(stream_buffer, args, stream_size)) {
                LOG_E("Failed to get stream data from user space");
                rt_free_align(stream_buffer);
                return RT_EINVAL;
            }

            free_stream = 1;
            stream      = (struct ws2812_stream*)stream_buffer;
        } else {
            free_stream = 0;
            stream      = (struct ws2812_stream*)args;
        }

        if (0x00 == stream->len) {
            LOG_E("Stream length is zero");
            if (free_stream) {
                rt_free_align(stream);
            }
            return RT_EINVAL;
        }

        if (stream->struct_sz != sizeof(struct ws2812_stream) + stream->len) {
            LOG_E("Invalid stream structure size: %d", stream->struct_sz);
            if (free_stream) {
                rt_free_align(stream);
            }
            return RT_EINVAL;
        }

        if (WS2812_STREAM_OVER_GPIO == stream->stream_type) {
            ret = ws2812_stream_over_gpio(stream);
            if (ret != RT_EOK) {
                LOG_E("Failed to stream over GPIO");
            }
        } else {
            ret = RT_EINVAL;
            LOG_E("Unsupported stream type: %d", stream->stream_type);
        }

        if (free_stream) {
            rt_free_align(stream);
        }
        return ret;
    }

    LOG_E("Unsupported command: %d", cmd);

    return RT_EINVAL;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops ws2182_dev_ops = {
    .init    = RT_NULL,
    .open    = RT_NULL,
    .close   = RT_NULL,
    .read    = RT_NULL,
    .write   = RT_NULL,
    .control = _ws2812_dev_control,
};
#endif

static int ws2812_device_init(void)
{
    rt_err_t err = RT_EOK;

    static struct rt_device ws2812_dev;
    static int              ws2812_inited = 0;

    if (ws2812_inited) {
        return 0;
    }

    rt_memset(&ws2812_dev, 0, sizeof(struct rt_device));

    ws2812_dev.user_data = NULL;
    ws2812_dev.type      = RT_Device_Class_Char;

#ifdef RT_USING_DEVICE_OPS
    ws2812_dev.ops = &ws2182_dev_ops;
#else
    ws2812_dev.init    = _wlan_mgmt_init;
    ws2812_dev.open    = RT_NULL;
    ws2812_dev.close   = RT_NULL;
    ws2812_dev.read    = RT_NULL;
    ws2812_dev.write   = RT_NULL;
    ws2812_dev.control = _rt_wlan_dev_control;
#endif

    /* register to device manager */
    rt_device_register(&ws2812_dev, "ws2812", RT_DEVICE_FLAG_RDWR);

    return 0;
}
INIT_APP_EXPORT(ws2812_device_init);
