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

#ifndef __K230_ATAG_RTT_H__
#define __K230_ATAG_RTT_H__

#include <rtthread.h>
#include <stdint.h>

/* ATAG tag definitions - must match U-Boot definitions */
#define ATAG_NONE		0x00000000
#define ATAG_CORE       0x54410001

/* K230-specific ATAG extensions */
#define ATAG_K230_DDR_SIZE	0x54410002	/* K230 DDR size information */
#define ATAG_K230_RTAPP		0x54410003	/* K230 RTAPP preload information */

struct tag_core {
    rt_uint32_t flags;
};

struct tag_k230_ddr_size {
	rt_uint64_t ddr_size;		/* DDR size in bytes */
};

struct tag_k230_rtapp {
	rt_uint64_t size;	/* RTAPP load address */
	rt_uint64_t load_address;	/* RTAPP load address */
};

struct tag_header {
	rt_uint32_t size;
	rt_uint32_t tag;
};

struct tag {
	struct tag_header hdr;
	union {
        struct tag_core             core;
        struct tag_k230_ddr_size	k230_ddr;
        struct tag_k230_rtapp		k230_rtapp;
	} u;
};

/* Tag list parsing macros */
#define tag_next(t)	((struct tag *)((rt_uint32_t *)(t) + (t)->hdr.size))
#define for_each_tag(t, base) \
	for (t = base; t->hdr.tag != ATAG_NONE; t = tag_next(t))

/* Memory layout for ATAG area - must match U-Boot definitions */
#define K230_ATAG_BASE		(CONFIG_RTSMART_OPENSIB_MEMORY_SIZE - 0x4000)	/* 128K - 16K */
#define K230_ATAG_MAX_SIZE	(CONFIG_RTSMART_OPENSIB_MEMORY_SIZE - 0x10)	/* 128K - 16B */

/* ATAG parsing result structure */
struct k230_atag_info {
	rt_uint64_t ddr_size;
	rt_uint64_t rtapp_size;
	rt_uint64_t rtapp_load_addr;
};

/* ATAG parser table structure */
struct tagtable {
	rt_uint32_t tag;
	int (*parse)(const struct tag *);
};

/* Linux-style tagtable declaration macros */
#define __tag __attribute__((used, section(".taglist.init")))
#define __tagtable(tag, fn) \
static const struct tagtable __tagtable_##fn __tag = { tag, fn }

/* Function prototypes */
int k230_atag_parse(struct k230_atag_info *info);
rt_uint64_t k230_atag_get_ddr_size(void);
rt_bool_t k230_atag_get_rtapp_size(rt_uint64_t *size);
rt_bool_t k230_atag_get_rtapp_loadaddr(rt_uint64_t *addr);

#endif /* __K230_ATAG_RTT_H__ */
