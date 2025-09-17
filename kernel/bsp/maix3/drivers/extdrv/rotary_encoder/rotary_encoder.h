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

#ifndef __DRV_ROTARY_ENCODER_H__
#define __DRV_ROTARY_ENCODER_H__

#include <stdint.h>
#include <rtdef.h>

/* Encoder direction definitions */
#define ENCODER_DIR_CW   1    /* Clockwise */
#define ENCODER_DIR_CCW  0xFF /* Counter-clockwise */
#define ENCODER_DIR_NONE 0    /* No movement */

/* IOCTL commands */
#define ENCODER_CMD_GET_DATA   _IOR('E', 1, struct encoder_data*)
#define ENCODER_CMD_RESET      _IO('E', 2)
#define ENCODER_CMD_SET_COUNT  _IOW('E', 3, int64_t*)
#define ENCODER_CMD_CONFIG     _IOW('E', 4, struct encoder_config*)
#define ENCODER_CMD_WAIT_DATA  _IOW('E', 5, int32_t*)

/* Data structures */
struct encoder_data {
    int32_t  delta;        /* Change since last read */
    int64_t  total_count;  /* Total count */
    uint8_t  direction;    /* Last direction */
    uint8_t  button_state; /* Button pressed state */
    uint32_t timestamp;    /* Tick timestamp */
};

struct encoder_config {
    int clk_pin;     /* Clock/A phase pin */
    int dt_pin;      /* Data/B phase pin */
    int sw_pin;      /* Switch/button pin (use -1 if not connected) */
};

#endif /* __DRV_ROTARY_ENCODER_H__ */
