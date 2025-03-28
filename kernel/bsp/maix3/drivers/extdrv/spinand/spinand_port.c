/*
* Copyright (c) 2022-2024, ArtInChip Technology Co., Ltd
*
* SPDX-License-Identifier: Apache-2.0
*
* Authors: xuan.wen <xuan.wen@artinchip.com>
*/

#include <stdint.h>
#include <string.h>
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "spinand.h"
#include <drivers/spi.h>

#define DBG_TAG "spi_nand_port"
#define DBG_LVL DBG_WARNING
#include <rtdbg.h>

static struct rt_mtd_nand_device *g_mtd_partitions;
static int g_mtd_partitions_cnt;

void qspi_messages_init(struct rt_qspi_message* qspi_messages, struct spi_nand_cmd_cfg* cfg, uint32_t addr,
                        uint8_t* sendbuff, uint8_t* recvbuff, uint32_t datacount)
{
    uint8_t inst_sz = __builtin_ffs(cfg->opcode);
    if(0x00 == inst_sz) {
        inst_sz = 0;
    } else if(8 > inst_sz) {
        inst_sz = 8;
    } else if(16 > inst_sz) {
        inst_sz = 16;
    } else {
        LOG_E("invalid inst sz\n");
        return;
    }

    /* 1-bit mode */
    qspi_messages->instruction.content      = cfg->opcode;
    qspi_messages->instruction.size         = 1 * 8;
    qspi_messages->instruction.qspi_lines   = cfg->opcode_bits;

    qspi_messages->address.content    = addr;
    qspi_messages->address.size       = cfg->addr_bytes * 8;
    qspi_messages->address.qspi_lines = cfg->addr_bits;

    if(cfg->dummy_bytes) {
        if (cfg->addr_bits) {
            qspi_messages->dummy_cycles = (cfg->dummy_bytes * 8) / qspi_messages->address.qspi_lines;
        } else {
            qspi_messages->dummy_cycles = (cfg->dummy_bytes * 8) / qspi_messages->instruction.qspi_lines;
        }
    } else {
        qspi_messages->dummy_cycles = 0;
    }

    /* 4-bit mode */
    qspi_messages->qspi_data_lines   = cfg->data_bits;
    qspi_messages->parent.cs_take    = 1;
    qspi_messages->parent.cs_release = 1;
    qspi_messages->parent.send_buf   = sendbuff;
    qspi_messages->parent.recv_buf   = recvbuff;
    qspi_messages->parent.length     = datacount;
    qspi_messages->parent.next       = NULL;

#if 0
    rt_kprintf("inst 0x%02X %d %d, addr 0x%X %d %d, dummy %d, %d, count %d, sbuf %p, rbuf %p\n", 
        qspi_messages->instruction.content, qspi_messages->instruction.size, qspi_messages->instruction.qspi_lines,
        qspi_messages->address.content, qspi_messages->address.size, qspi_messages->address.qspi_lines,
        qspi_messages->dummy_cycles, qspi_messages->qspi_data_lines, datacount, sendbuff, recvbuff
    );
#endif
}

int aic_spinand_transfer_message(struct aic_spinand *flash,
                                struct spi_nand_cmd_cfg *cfg, uint32_t addr,
                                uint8_t *sendbuff, uint8_t *recvbuff, uint32_t datacount)
{
    int result;
    struct rt_qspi_message qspi_messages = { 0 };
    struct rt_qspi_device *device = (struct rt_qspi_device *)flash->user_data;

    RT_ASSERT(flash != RT_NULL);
    RT_ASSERT(device != RT_NULL);

    qspi_messages_init(&qspi_messages, cfg, addr, sendbuff, recvbuff,
                    datacount);

    result = rt_mutex_take(&(device->parent.bus->lock), RT_WAITING_FOREVER);
    if (result != SPINAND_SUCCESS) {
        rt_set_errno(-RT_EBUSY);
        return result;
    }

    /* reset errno */
    rt_set_errno(SPINAND_SUCCESS);

    /* configure SPI bus */
    if (device->parent.bus->owner != &device->parent) {
        /* not the same owner as current, re-configure SPI bus */
        result = device->parent.bus->ops->configure(&device->parent,
                                                    &device->config.parent);
        if (result == SPINAND_SUCCESS) {
            /* set SPI bus owner */
            device->parent.bus->owner = &device->parent;
        } else {
            LOG_E("Configure SPI bus failed\n");
            rt_set_errno(-SPINAND_ERR);
            goto __exit;
        }
    }

    /* transmit each SPI message */
    if (device->parent.bus->ops->xfer(&device->parent, &qspi_messages.parent) < 0) {
        LOG_E("Xfer SPI bus failed\n");
        result = -SPINAND_ERR;
        rt_set_errno(-SPINAND_ERR);
        goto __exit;
    }

    result = SPINAND_SUCCESS;

__exit:
    /* release bus lock */
    rt_mutex_release(&(device->parent.bus->lock));

    return result;
}

static void spinand_dump_buffer(int page, rt_uint8_t *buf, int len,
                                const char *title)
{
#if DBG_LVL >= DBG_LOG
    int i;

    if(!buf || !len) {
        return;
    }

    rt_kprintf("[%s-Page-%d]\n", title, page);

    for (i = 0; i < len; i++) {
        rt_kprintf("%02X ", buf[i]);
        if (i % 16 == 15)
            rt_kprintf("\n");
    }
    rt_kprintf("\n");
#endif
}

static rt_err_t spinand_read_id(struct rt_mtd_nand_device *device)
{
    rt_err_t result = RT_EOK;
    uint32_t id = 0;
    struct aic_spinand *flash = (struct aic_spinand *)device->priv;

    RT_ASSERT(device != RT_NULL);

    result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    spinand_read_id_op(flash, (uint8_t *)&id);

    result = rt_mutex_release(flash->lock);
    RT_ASSERT(result == RT_EOK);

    LOG_I("Id: 0x%08x\n", id);

    result = (id != 0x0) ? RT_EOK : -RT_ERROR;
    return result;
}

static rt_err_t spinand_mtd_read(struct rt_mtd_nand_device *device,
                                rt_off_t page, rt_uint8_t *data,
                                rt_uint32_t data_len, rt_uint8_t *spare,
                                rt_uint32_t spare_len)
{
    struct aic_spinand *flash;
    rt_err_t result;

    RT_ASSERT(device != RT_NULL);

    if (page / device->pages_per_block > device->block_end) {
        LOG_E("[Error] read page:%d\n", page);
        return -RT_MTD_EIO;
    }

    flash = (struct aic_spinand *)device->priv;

    result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    result = spinand_read_page(flash, page, data, data_len, spare, spare_len);

    spinand_dump_buffer(page, (uint8_t *)data, data_len, "READ DATA");
    spinand_dump_buffer(page, (uint8_t *)spare, spare_len, "READ SPARE");

    rt_mutex_release(flash->lock);

    return result;
}

// #ifdef AIC_SPINAND_CONT_READ
// static rt_err_t spinand_mtd_continuous_read(struct rt_mtd_nand_device *device,
//                                             rt_off_t page, rt_uint8_t *data,
//                                             rt_uint32_t size)
// {
//     rt_err_t result;
//     struct aic_spinand *flash = (struct aic_spinand *)device->priv;

//     result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
//     RT_ASSERT(result == RT_EOK);

//     result = spinand_continuous_read(flash, page, data, size);

//     rt_mutex_release(flash->lock);

//     return result;
// }
// #else
// static rt_err_t spinand_mtd_continuous_read(struct rt_mtd_nand_device *device,
//                                             rt_off_t page, rt_uint8_t *data,
//                                             rt_uint32_t size)
// {
//     LOG_E("Please enable config AIC_SPINAND_CONT_READ!.\n");
//     return -1;
// }
// #endif

static rt_err_t spinand_mtd_write(struct rt_mtd_nand_device *device,
                                rt_off_t page, const rt_uint8_t *data,
                                rt_uint32_t data_len, const rt_uint8_t *spare,
                                rt_uint32_t spare_len)
{
    struct aic_spinand *flash = (struct aic_spinand *)device->priv;
    RT_ASSERT(device != RT_NULL);
    rt_err_t result = RT_EOK;

    if (page / device->pages_per_block > device->block_end) {
        LOG_E("[Error] write page:%d\n", page);
        return -RT_MTD_EIO;
    }

    spinand_dump_buffer(page, (uint8_t *)data, data_len, "WRITE DATA");
    spinand_dump_buffer(page, (uint8_t *)spare, spare_len, "WRITE SPARE");

    result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    result = spinand_write_page(flash, page, data, data_len, spare, spare_len);
    rt_mutex_release(flash->lock);
    return result;
}

static rt_err_t spinand_mtd_erase(struct rt_mtd_nand_device *device,
                                rt_uint32_t block)
{
    rt_err_t result = RT_EOK;
    struct aic_spinand *flash = (struct aic_spinand *)device->priv;

    RT_ASSERT(device != RT_NULL);

    if (block > device->block_end) {
        LOG_E("[Error] block:%d block_end:%d\n", block, device->block_end);
        return -RT_MTD_EIO;
    }

    LOG_D("erase block: %d\n", block);

    result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    result = spinand_block_erase(flash, block);
    if (result != RT_EOK)
        goto exit_spinand_mtd_erase;

    result = RT_EOK;

exit_spinand_mtd_erase:

    rt_mutex_release(flash->lock);

    return result;
}

static rt_err_t spinand_mtd_block_isbad(struct rt_mtd_nand_device *device,
                                        rt_uint32_t block)
{
    struct aic_spinand *flash = (struct aic_spinand *)device->priv;
    rt_err_t result = RT_EOK;

    RT_ASSERT(device != RT_NULL);

    if (block > device->block_end) {
        LOG_E("[Error] block:%d\n", block);
        return -RT_MTD_EIO;
    }

    pr_debug("check block status: %d\n", block);

    result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    result = spinand_block_isbad(flash, block);

    rt_mutex_release(flash->lock);

    return result;
}

static rt_err_t spinand_mtd_block_markbad(struct rt_mtd_nand_device *device,
                                        rt_uint32_t block)
{
    rt_err_t result = RT_EOK;
    struct aic_spinand *flash = (struct aic_spinand *)device->priv;

    RT_ASSERT(device != RT_NULL);

    if (block > device->block_end) {
        LOG_E("[Error] block:%d\n", block);
        return -RT_MTD_EIO;
    }

    LOG_I("mark bad block: %d\n", block);

    result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    /* Erase block after checking it is bad or not. */
    if (spinand_block_isbad(flash, block) != 0) {
        LOG_W("Block %d is bad.\n", block);
        result = RT_EOK;
    } else {
        result = spinand_block_markbad(flash, block);
    }

    rt_mutex_release(flash->lock);

    return result;
}

// static rt_err_t spinand_set_block_status(struct rt_mtd_nand_device *device,
//     rt_uint32_t block, rt_uint32_t block_pos,
//     rt_uint32_t status)
// {
// rt_err_t result = RT_EOK;
// struct aic_spinand *flash = (struct aic_spinand *)device->priv;

// RT_ASSERT(device != RT_NULL);

// if (block > device->block_end) {
// LOG_E("[Error] block:%d\n", block);
// return -RT_MTD_EIO;
// }

// result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
// RT_ASSERT(result == RT_EOK);

// result = spinand_set_status(flash, block, block_pos, status);

// rt_mutex_release(flash->lock);

// return result;
// }

// static rt_uint32_t spinand_get_block_status(struct rt_mtd_nand_device *device,
//                                         rt_uint32_t block)
// {
//     rt_uint32_t result = RT_EOK;
//     struct aic_spinand *flash = (struct aic_spinand *)device->priv;

//     RT_ASSERT(device != RT_NULL);

//     if (block > device->block_end) {
//         LOG_E("[Error] block:%d\n", block);
//         return -RT_MTD_EIO;
//     }

//     result = rt_mutex_take(flash->lock, RT_WAITING_FOREVER);
//     RT_ASSERT(result == RT_EOK);

//     result = spinand_get_status(flash, block);

//     rt_mutex_release(flash->lock);

//     return result;
// }

// static rt_err_t spinand_map_user(struct rt_mtd_nand_device *device,
//                                         rt_uint8_t *oobbuf, rt_uint8_t *buf,
//                                         rt_base_t start, rt_base_t nbytes)
// {
//     rt_err_t result = RT_EOK;
//     struct aic_spinand *flash = (struct aic_spinand *)device->priv;

//     RT_ASSERT(device != RT_NULL);

//     result = spinand_ooblayout_map_user(flash, oobbuf, buf, start, nbytes);

//     return result;
// }

// static rt_err_t spinand_unmap_user(struct rt_mtd_nand_device *device,
//                                         rt_uint8_t *dst, rt_uint8_t *src,
//                                         rt_base_t start, rt_base_t nbytes)
// {
//     rt_err_t result = RT_EOK;
//     struct aic_spinand *flash = (struct aic_spinand *)device->priv;

//     RT_ASSERT(device != RT_NULL);

//     result = spinand_ooblayout_unmap_user(flash, dst, src, start, nbytes);

//     return result;
// }

static struct rt_mtd_nand_driver_ops spinand_ops = {
    spinand_read_id,                // read_id
    spinand_mtd_read,               // read_page
    spinand_mtd_write,              // write_page
    NULL,                           // move_page
    spinand_mtd_erase,              // erase_block
    spinand_mtd_block_isbad,        // check_block
    spinand_mtd_block_markbad,      // mark_badblock
    // spinand_mtd_continuous_read,
    // spinand_set_block_status,
    // spinand_get_block_status,
    // spinand_map_user, 
    // spinand_unmap_user
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
struct mtd_nand_partition {
    const char *name;
    rt_uint32_t start;
    rt_uint32_t size;
};

static struct mtd_nand_partition mtd_nand_part_tbl[] = {
    #include "spinand_parts.h"
};

rt_err_t rt_hw_mtd_spinand_init(struct aic_spinand *flash)
{
    struct mtd_nand_partition *parts, *p;
    rt_uint32_t blocksize;
    rt_err_t result;

    if (flash->IsInited) {
        return RT_EOK;
    }

    flash->lock = rt_mutex_create("spinand", RT_IPC_FLAG_PRIO);
    if (flash->lock == RT_NULL) {
        LOG_E("Create mutex in rt_hw_mtd_spinand_init failed\n");
        return -1;
    }

    result = spinand_flash_init(flash);
    if (result != RT_EOK) {
        return -RT_ERROR;
    }
    rt_kprintf("Found spi nand flash %s\n", flash->info->sz_description);

    g_mtd_partitions_cnt = sizeof(mtd_nand_part_tbl) / sizeof(mtd_nand_part_tbl[0]);
    g_mtd_partitions = rt_malloc(sizeof(struct rt_mtd_nand_device) * g_mtd_partitions_cnt);
    if (!g_mtd_partitions) {
        LOG_E("malloc buf failed\n");
        return -1;
    }

    blocksize = flash->info->page_size * flash->info->pages_per_eraseblock;
    p = &mtd_nand_part_tbl[0];
    for (int i = 0; i < g_mtd_partitions_cnt; i++, p++) {
        g_mtd_partitions[i].page_size = flash->info->page_size;
        g_mtd_partitions[i].pages_per_block = flash->info->pages_per_eraseblock;
        g_mtd_partitions[i].oob_size = flash->info->oob_size;
        g_mtd_partitions[i].oob_free = 32;
        g_mtd_partitions[i].ops = &spinand_ops;
        g_mtd_partitions[i].block_start = p->start / blocksize;
        g_mtd_partitions[i].block_end = (p->start + p->size - 1) / blocksize;
        g_mtd_partitions[i].block_total = p->size / blocksize;
        g_mtd_partitions[i].priv = flash;

        result = rt_mtd_nand_register_device(p->name, &g_mtd_partitions[i]);
        RT_ASSERT(result == RT_EOK);
    }

    flash->databuf = rt_malloc_align(
        flash->info->page_size + flash->info->oob_size, RT_CPU_CACHE_LINE_SZ);
    if (!flash->databuf) {
        LOG_E("malloc buf failed\n");
        return -1;
    }
    flash->oobbuf = flash->databuf + flash->info->page_size;

    flash->IsInited = true;

    return result;
}

rt_err_t rt_hw_mtd_spinand_register(const char *device_name)
{
    rt_device_t pDev;
    rt_err_t result;
    struct rt_qspi_device *device;
    struct aic_spinand *spinand;

    struct rt_qspi_configuration cfg = {
        .parent.mode       = 0,
        .parent.hard_cs    = 0x80 | (1 << 0), // OSPI_CS0
        .parent.soft_cs    = 0x00,
        .parent.data_width = 8,
        .parent.max_hz     = SPI_NAND_SPI_FREQ,
        .ddr_mode          = 0,
        .medium_size       = 0,
        .qspi_dl_width     = SPI_NAND_SPI_BUS_WIDTH,
    };

    if ((pDev = rt_device_find(device_name)) == RT_NULL) {
        return -RT_ERROR;
    }

    result = rt_qspi_configure((struct rt_qspi_device *)pDev, &cfg);
    if (result != RT_EOK) {
        LOG_E("configure bus failed!\n");
        return result;
    }

    spinand = rt_calloc(1, sizeof(struct aic_spinand));
    if (!spinand) {
        LOG_E("malloc buf failed\n");
        return -RT_ERROR;
    }

    device = (struct rt_qspi_device *)pDev;
    spinand->user_data = device;
    spinand->qspi_dl_width = device->config.qspi_dl_width;

    return rt_hw_mtd_spinand_init(spinand);
}

static int rt_hw_spinand_register(void)
{
    int    ret;
    static struct rt_qspi_device *nand_spi_dev = NULL;
    
    if (NULL == nand_spi_dev) {
        nand_spi_dev = (struct rt_qspi_device*)rt_malloc(sizeof(struct rt_qspi_device));
        if (nand_spi_dev == RT_NULL) {
            LOG_E("Failed to malloc memory for qspi device.\n");
            return -RT_ENOMEM;
        }
    }

    ret = rt_spi_bus_attach_device((struct rt_spi_device*)nand_spi_dev, "spinand0", SPI_NAND_SPI_DEV, NULL);
    if (ret != RT_EOK) {
        LOG_E("attach %s failed!\n", SPI_NAND_SPI_DEV);
        return ret;
    }

    rt_hw_mtd_spinand_register("spinand0");

    return RT_EOK;
}

INIT_COMPONENT_EXPORT(rt_hw_spinand_register);
