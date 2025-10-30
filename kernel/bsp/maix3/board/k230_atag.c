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

#include <rtthread.h>
#include <string.h>
#include <ioremap.h>
#include "k230_atag.h"

static struct k230_atag_info g_atag_info = {0};
static rt_bool_t g_atag_parsed = RT_FALSE;

/* External tagtable symbols - will be placed in .taglist.init section */
extern const struct tagtable __tagtable_begin[];
extern const struct tagtable __tagtable_end[];

static int parse_tag_k230_core(const struct tag *tag)
{
    return 0;
}

static int parse_tag_k230_ddr_size(const struct tag *tag)
{
	g_atag_info.ddr_size = tag->u.k230_ddr.ddr_size;

	return 0;
}

static int parse_tag_k230_rtapp(const struct tag *tag)
{
	g_atag_info.rtapp_size = tag->u.k230_rtapp.size;
	g_atag_info.rtapp_load_addr = tag->u.k230_rtapp.load_address;

	return 0;
}

__tagtable(ATAG_CORE, parse_tag_k230_core);
__tagtable(ATAG_K230_DDR_SIZE, parse_tag_k230_ddr_size);
__tagtable(ATAG_K230_RTAPP, parse_tag_k230_rtapp);

static int parse_tag(const struct tag *tag)
{
	const struct tagtable *t;

	for (t = &__tagtable_begin[0]; t < &__tagtable_end[0]; t++) {
		if (tag->hdr.tag == t->tag) {
			t->parse(tag);
			return 1;
		}
	}

	return 0;
}

int k230_atag_parse(struct k230_atag_info *info)
{
	struct tag *tag;
	void *atag_addr;

	if (g_atag_parsed) {
		if (info) {
			*info = g_atag_info;
		}
		return 0;
	}

	rt_memset(&g_atag_info, 0, sizeof(g_atag_info));

    atag_addr = (void *)K230_ATAG_BASE;

	for_each_tag(tag, atag_addr) {
		/* Safety check - prevent infinite loop */
		if ((rt_uint32_t)tag >= K230_ATAG_MAX_SIZE) {
			rt_kprintf("ATAG: Tag address 0x%p exceeds bounds\n", (void *)tag);
			break;
		}

		if (!parse_tag(tag)) {
			rt_kprintf("ATAG: Ignoring unrecognised tag 0x%08x\n", tag->hdr.tag);
		}
	}

	g_atag_parsed = RT_TRUE;

	if (info) {
		*info = g_atag_info;
	}

	return 0;
}

rt_uint64_t k230_atag_get_ddr_size(void)
{
	if (!g_atag_parsed) {
		if (k230_atag_parse(NULL) != 0) {
			return 0; /* Default or error value */
		}
	}

	return g_atag_info.ddr_size;
}

rt_bool_t k230_atag_get_rtapp_size(rt_uint64_t *size)
{
	if (!g_atag_parsed) {
		if (k230_atag_parse(NULL) != 0) {
			return RT_FALSE;
		}
	}

	if (size) {
		*size = g_atag_info.rtapp_size;
	}

	return RT_TRUE;
}

rt_bool_t k230_atag_get_rtapp_loadaddr(rt_uint64_t *addr)
{
	if (!g_atag_parsed) {
		if (k230_atag_parse(NULL) != 0) {
			return RT_FALSE;
		}
	}

	if (addr) {
		*addr = g_atag_info.rtapp_load_addr;
	}

	return RT_TRUE;
}

