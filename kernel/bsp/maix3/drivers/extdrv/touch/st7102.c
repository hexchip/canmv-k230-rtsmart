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
#include "drv_touch.h"
#include "rtthread.h"

#include <stdint.h>

#define DBG_TAG "st7102"
#define DBG_LVL DBG_WARNING
#include <rtdbg.h>

/* ST7102 register definitions */
#define ST7102_SLAVE_ADDRESS 0x56

#define FW_VERSION_REG  (0x0000)
#define DEV_CONTROL_REG (0x0002)
#define FW_REVISION_REG (0x000C)

#define MAX_X_COORD_H_REG (0x0005)
#define MAX_X_COORD_L_REG (0x0006)
#define MAX_Y_COORD_H_REG (0x0007)
#define MAX_Y_COORD_L_REG (0x0008)
#define MAX_TOUCHES_REG   (0x0009)

#define REPORT_STATUS_REG  (0x0010)
#define REPORT_COORD_0_REG (0x0014)

struct st7102_status {
    uint8_t reserved_0_1 : 2;
    uint8_t with_prox : 1;
    uint8_t with_coord : 1;
    uint8_t prox_status : 3;
    uint8_t rst_chip : 1;
};

struct st7102_point {
    uint8_t x_h : 6;
    uint8_t reserved_6 : 1;
    uint8_t valid : 1;
    uint8_t x_l;
    uint8_t y_h;
    uint8_t y_l;
    uint8_t area;
    uint8_t intensity;
    uint8_t reserved_49_55;
};

struct st7102_reg {
    // REPORT_STATUS_REG
    struct st7102_status status;

    // WHAT IS THESE ?
    uint8_t resv_11_13[3];

    // REPORT_COORD_0_REG
    struct st7102_point points[0];
};

_Static_assert((sizeof(struct st7102_reg) + sizeof(struct st7102_point) * TOUCH_MAX_POINT_NUMBER) < TOUCH_READ_REG_MAX_SIZE,
               "st7102 reg data size > TOUCH_READ_REG_MAX_SIZE");

// APIs ///////////////////////////////////////////////////////////////////////
static int read_register(struct drv_touch_dev* dev, struct touch_register* reg)
{
    int     ret       = 0;
    size_t  read_size = 0;
    uint8_t cmd[2]    = { 0 };

    reg->time = rt_tick_get();

    // REPORT_STATUS_REG
    cmd[0] = 0x00;
    cmd[1] = 0x10;

    read_size = sizeof(struct st7102_point) * dev->touch.point_num + sizeof(struct st7102_reg);

    ret = touch_dev_write_read_reg(dev, cmd, 2, (uint8_t*)&reg->reg[0], read_size);
    if (ret != 0) {
        LOG_D("st7102 read point failed");
        return -1;
    }

    return 0;
}

static int parse_register(struct drv_touch_dev* dev, struct touch_register* reg, struct touch_point* result)
{
    int      finger_num = 0, max_touch = 0;
    uint8_t  xh, xl, yh, yl;
    uint16_t point_x, point_y, point_size;
    int      result_index = 0, point_index = 0;

    rt_tick_t time = reg->time;

    struct rt_touch_data* point      = NULL;
    struct st7102_reg*    st7102_reg = (struct st7102_reg*)reg->reg;

    result->point_num = 0;

    if (!st7102_reg->status.with_coord) {
        rt_memset(result->point, 0x00, sizeof(result->point));

        touch_dev_update_event(finger_num, result->point);

        return 0;
    }

    max_touch = dev->touch.point_num;
    if (max_touch > TOUCH_MAX_POINT_NUMBER) {
        LOG_W("st7102 touch point %d > max %d", max_touch, TOUCH_MAX_POINT_NUMBER);

        max_touch = TOUCH_MAX_POINT_NUMBER;
    }

    for (result_index = 0, point_index = 0; result_index < max_touch; result_index++, point_index++) {
        point = &result->point[point_index];

        if (!st7102_reg->points[result_index].valid) {
            point_index--;
            continue;
        }

        xh      = st7102_reg->points[result_index].x_h;
        xl      = st7102_reg->points[result_index].x_l;
        point_x = (xh << 8) | xl;
        if (point_x > dev->touch.range_x) {
            point_index--;
            continue;
        }

        yh      = st7102_reg->points[result_index].y_h;
        yl      = st7102_reg->points[result_index].y_l;
        point_y = (yh << 8) | yl;
        if (point_y > dev->touch.range_y) {
            point_index--;
            continue;
        }

        point_size = st7102_reg->points[result_index].area;

        point->event        = RT_TOUCH_EVENT_NONE;
        point->track_id     = result_index;
        point->width        = point_size;
        point->x_coordinate = point_x;
        point->y_coordinate = point_y;
        point->timestamp    = time;

        finger_num++;
    }

    result->point_num = finger_num;
    touch_dev_update_event(finger_num, result->point);

    return 0;
}

static int reset(struct drv_touch_dev* dev)
{
    if ((0 <= dev->pin.rst) && (63 >= dev->pin.rst)) {
        kd_pin_write(dev->pin.rst, 1 - dev->pin.rst_valid);
        rt_thread_mdelay(20);
        kd_pin_write(dev->pin.rst, dev->pin.rst_valid);
        rt_thread_mdelay(10);
        kd_pin_write(dev->pin.rst, 1 - dev->pin.rst_valid);
        rt_thread_mdelay(50);
    }

    rt_thread_mdelay(50); // wait reset done.

    return 0;
}

static int get_info(struct drv_touch_dev* dev)
{
    uint8_t version     = 0;
    uint8_t cmd[2]      = { 0 };
    uint8_t revision[4] = { 0 };

    struct {
        uint8_t max_x_h; // MAX_X_COORD_H_REG
        uint8_t max_x_l; // MAX_X_COORD_L_REG
        uint8_t max_y_h; // MAX_Y_COORD_H_REG
        uint8_t max_y_l; // MAX_Y_COORD_L_REG
        uint8_t max_touches; // MAX_TOUCHES_REG
    } info;

    // FW_VERSION_REG
    cmd[0] = 0x00;
    cmd[1] = 0x00;
    if (0x00 != touch_dev_write_read_reg(dev, cmd, 2, &version, 1)) {
        LOG_E("failed to get touch info 1");
        return -1;
    }

    // FW_REVISION_REG
    cmd[0] = 0x00;
    cmd[1] = 0x0C;
    if (0x00 != touch_dev_write_read_reg(dev, cmd, 2, &version, 4)) {
        LOG_E("failed to get touch info 2");
        return -1;
    }

    // MAX_X_COORD_H_REG
    cmd[0] = 0x00;
    cmd[1] = 0x05;
    if (0x00 != touch_dev_write_read_reg(dev, cmd, 2, (uint8_t*)&info, sizeof(info))) {
        LOG_E("failed to get touch info 3");
        return -1;
    }

    dev->touch.range_x   = (info.max_x_h << 8) | info.max_x_l;
    dev->touch.range_y   = (info.max_y_h << 8) | info.max_y_l;
    dev->touch.point_num = info.max_touches;

    LOG_I("Firmware version: %d(%d.%d.%d.%d), Max.X: %d, Max.Y: %d, Max.Touchs: %d", version, revision[0], revision[1],
          revision[2], revision[3], dev->touch.range_x, dev->touch.range_y, dev->touch.point_num);

    return 0;
}

static int get_default_rotate(struct drv_touch_dev* dev)
{
    // Default rotation - adjust based on your board configuration
    return RT_TOUCH_ROTATE_DEGREE_0;
}

int drv_touch_probe_st7102(struct drv_touch_dev* dev)
{
    // Initialize device parameters
    dev->i2c.addr      = ST7102_SLAVE_ADDRESS;
    dev->i2c.reg_width = 2; // ST7102 uses 2-byte register addresses

    // Perform reset
    if (reset(dev) != 0) {
        return -1;
    }

    // Get device information
    if (get_info(dev) != 0) {
        return -2;
    }

    // Set driver name
    rt_strncpy(dev->dev.drv_name, "st7102", sizeof(dev->dev.drv_name));

    // Register callbacks
    dev->dev.read_register      = read_register;
    dev->dev.parse_register     = parse_register;
    dev->dev.reset              = reset;
    dev->dev.get_default_rotate = get_default_rotate;

    rt_kprintf("st7102-> resolution %dx%d, max touches: %d\n", dev->touch.range_x, dev->touch.range_y, dev->touch.point_num);

    return 0;
}
