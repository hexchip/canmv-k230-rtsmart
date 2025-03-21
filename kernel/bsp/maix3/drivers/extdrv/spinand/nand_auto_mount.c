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

#include "nand_auto_mount.h"
#include "rtthread.h"
#include <drivers/spi.h>

#define DBG_TAG "nand_drv"
#ifdef RT_SPI_DEBUG
#define DBG_LVL DBG_LOG
#else
#define DBG_LVL DBG_WARNING
#endif
#include <rtdbg.h>

struct rt_qspi_device nand_spi_dev;

struct mtd_nand_partition {
    rt_uint32_t block_start;
    rt_uint32_t block_end;
};

static struct mtd_nand_partition mtd_nand_part_tbl[] = {
    // bin: 40M ~ 50M
    { 0x2800000, 0x2800000 + 80 * 2048 * 64 },
};

rt_err_t nand_standard_transfer(rt_uint8_t* in, rt_uint8_t* out, rt_uint32_t size)
{
    struct rt_qspi_message msg = {
        .parent.cs_take         = 0,
        .parent.cs_release      = 0,
        .parent.next            = NULL,
        .parent.send_buf        = in,
        .parent.recv_buf        = out,
        .parent.length          = size,
        .instruction.content    = 0,
        .instruction.size       = 0,
        .instruction.qspi_lines = 0,
        .address.content        = 0,
        .address.size           = 0,
        .address.qspi_lines     = 0,
        .dummy_cycles           = 0,
        .qspi_data_lines        = 1,
    };

    if (0 > rt_qspi_transfer_message(&nand_spi_dev, &msg)) {
        return -RT_EIO;
    }

    return 0;
}

static int nand_auto_mount_run(void)
{
    int    ret;
    size_t part_count = 0;
    size_t block_size = 0;
    char   dev_name[RT_NAME_MAX];

    struct rt_mtd_nand_device chip_info;

    static struct rt_mtd_nand_device* devs = NULL;

    struct rt_qspi_configuration cfg = {
        .parent.mode       = 0,
        .parent.hard_cs    = 0x80 | (1 << 0), // OSPI_CS0
        .parent.soft_cs    = 0x00, // not use
        .parent.data_width = 8,
        .parent.max_hz     = 10 * 1000 * 1000,
        .ddr_mode          = 0,
        .medium_size       = 0,
        .qspi_dl_width     = 4,
    };

    ret = rt_spi_bus_attach_device((struct rt_spi_device*)&nand_spi_dev, "spi_nand", SPI_NAND_SPI_DEV, NULL);
    if (ret != RT_EOK) {
        LOG_E("attach %s failed!\n", SPI_NAND_SPI_DEV);
        return ret;
    }

    ret = rt_qspi_configure(&nand_spi_dev, &cfg);
    if (ret != RT_EOK) {
        LOG_E("configure bus failed!\n");
        return ret;
    }

    /* init device */
#if defined(ENABLE_SPI_NAND_WINBOND)
    if (0x00 != winbond_nand_init(&chip_info)) {
        LOG_E("init winbond nand failed");
        return -1;
    }
#else
    LOG_E("not enable nand chip support");
    return -1;
#endif

    /* create parts */
    block_size = (chip_info.page_size) * chip_info.pages_per_block;
    part_count = sizeof(mtd_nand_part_tbl) / sizeof(mtd_nand_part_tbl[0]);

    if (devs) {
        rt_free_align(devs);
        devs = NULL;
    }

    devs = rt_malloc_align(sizeof(struct rt_mtd_nand_device) * part_count, RT_CPU_CACHE_LINE_SZ);
    if (NULL == devs) {
        LOG_E("malloc failed");
        return -1;
    }

    for (size_t i = 0; i < part_count; i++) {
        struct rt_mtd_nand_device* dev  = &devs[i];
        struct mtd_nand_partition* part = &mtd_nand_part_tbl[i];

        if ((part->block_start % block_size) || (part->block_end % block_size)) {
            LOG_E("part start (%d) or end(%d) not align to block size(%d)", part->block_start, part->block_end,
                  block_size);
            return -1;
        }

        rt_memcpy(dev, &chip_info, sizeof(struct rt_mtd_nand_device));

        dev->block_start = part->block_start / block_size;
        dev->block_end   = (part->block_end / block_size) - 1;
        dev->block_total = dev->block_end - dev->block_start + 1;

        if (dev->block_end > chip_info.block_end) {
            LOG_E("part exceed nand device size.");
            return -1;
        }

        rt_snprintf(dev_name, sizeof(dev_name), "nand%d", i);

        if (RT_EOK != rt_mtd_nand_register_device(dev_name, dev)) {
            LOG_E("register nand part dev failed");
            return -1;
        }

#if 1
        rt_kprintf("page_size:\t %d(0x%08X)\n", dev->page_size, dev->page_size);
        rt_kprintf("oob_size:\t %d(0x%08X)\n", dev->oob_size, dev->oob_size);
        rt_kprintf("oob_free:\t %d(0x%08X)\n", dev->oob_free, dev->oob_free);
        rt_kprintf("plane_num:\t %d(0x%08X)\n", dev->plane_num, dev->plane_num);
        rt_kprintf("pages_per_block:\t %d(0x%08X)\n", dev->pages_per_block, dev->pages_per_block);
        rt_kprintf("block_total:\t %d(0x%08X)\n", dev->block_total, dev->block_total);
        rt_kprintf("block_start:\t %d(0x%08X)\n", dev->block_start, dev->block_start);
        rt_kprintf("block_end:\t %d(0x%08X)\n", dev->block_end, dev->block_end);
#endif
    }

    return 0;
}
INIT_COMPONENT_EXPORT(nand_auto_mount_run);
