/*
 * RT-Thread Device Interface for uffs
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "dfs_uffs.h"

#if  RT_CONFIG_UFFS_ECC_MODE != UFFS_ECC_HW_AUTO
    #error "UFFS Only support ecc_hw_auto"
#endif

static void inline dump_buffer(const char *tag, const uint8_t *buffer, size_t size)
{
#if 0
rt_kprintf("%s->%d\n", tag, size);
    for(size_t i = 0; i < size; i++) {
        rt_kprintf("%02X ", buffer[i]);
        if(15 == (i % 16)) {
            rt_kprintf("\n");
        }
    }
    rt_kprintf("\n");
#endif
}

static int nand_init_flash(uffs_Device *dev)
{
    return UFFS_FLASH_NO_ERR;
}

static int nand_release_flash(uffs_Device *dev)
{
    return UFFS_FLASH_NO_ERR;
}

static int nand_erase_block(uffs_Device *dev, unsigned block)
{
    int res;

    res = rt_mtd_nand_erase_block(RT_MTD_NAND_DEVICE(dev->_private), block);

    return res == RT_EOK ? UFFS_FLASH_NO_ERR : UFFS_FLASH_IO_ERR;
}

#if defined(RT_UFFS_USE_CHECK_MARK_FUNCITON)
static int nand_check_block(uffs_Device *dev, unsigned block)
{
    int res;

    res = rt_mtd_nand_check_block(RT_MTD_NAND_DEVICE(dev->_private), block);

    return res == RT_EOK ? UFFS_FLASH_NO_ERR : UFFS_FLASH_BAD_BLK;
}

static int nand_mark_badblock(uffs_Device *dev, unsigned block)
{
    int res;

    res = rt_mtd_nand_mark_badblock(RT_MTD_NAND_DEVICE(dev->_private), block);

    return res == RT_EOK ? UFFS_FLASH_NO_ERR : UFFS_FLASH_IO_ERR;
}
#endif

static int WritePageWithLayout(uffs_Device         *dev,
                               u32                  block,
                               u32                  page,
                               const u8            *data,
                               int                  data_len,
                               const u8            *ecc,  //NULL
                               const uffs_TagStore *ts)
{
    int res;
    int spare_len, oob_size;
    rt_uint8_t spare[2][UFFS_MAX_SPARE_SIZE];

    // RT_ASSERT(UFFS_MAX_SPARE_SIZE >= dev->attr->spare_size);

    if (data == NULL && ts == NULL) {
        RT_ASSERT(0); //should not be here
    }

    spare_len = dev->mem.spare_data_size;
    page = block * dev->attr->pages_per_block + page;

    oob_size = RT_MTD_NAND_DEVICE(dev->_private)->oob_size;
    if(oob_size > UFFS_MAX_SPARE_SIZE) {
        oob_size = UFFS_MAX_SPARE_SIZE;
    }

    if (data != NULL && data_len != 0) {
        RT_ASSERT(data_len == dev->attr->page_data_size);

        dev->st.page_write_count++;
        dev->st.io_write += data_len;
    }

    rt_memset(spare, 0xFF, sizeof(spare));

    if (ts != RT_NULL) {
        uffs_FlashMakeSpare(dev, ts, RT_NULL, (u8 *)&spare[0]);
        dev->st.spare_write_count++;
        dev->st.io_write += spare_len;

        rt_mtd_nand_map_user(RT_MTD_NAND_DEVICE(dev->_private), (rt_uint8_t *)&spare[1], (rt_uint8_t *)&spare[0], 0, 16);
    }

    res = rt_mtd_nand_write(RT_MTD_NAND_DEVICE(dev->_private),
                            page, data, data_len, (const rt_uint8_t *)&spare[1], oob_size);
    if (res != RT_EOK) {
        goto __error;
    }

    return UFFS_FLASH_NO_ERR;

__error:
    return UFFS_FLASH_IO_ERR;
}

static URET ReadPageWithLayout(uffs_Device   *dev,
                               u32            block,
                               u32            page,
                               u8            *data,
                               int            data_len,
                               u8            *ecc,              //NULL
                               uffs_TagStore *ts,
                               u8            *ecc_store)        //NULL
{
    int res = UFFS_FLASH_NO_ERR;
    int spare_len, oob_size;
    rt_uint8_t spare[2][UFFS_MAX_SPARE_SIZE];

    // RT_ASSERT(UFFS_MAX_SPARE_SIZE >= dev->attr->spare_size);

    if (data == RT_NULL && ts == RT_NULL) {
        RT_ASSERT(0); //should not be here
    }

    spare_len = dev->mem.spare_data_size;
    page = block * dev->attr->pages_per_block + page;

    oob_size = RT_MTD_NAND_DEVICE(dev->_private)->oob_size;
    if(oob_size > UFFS_MAX_SPARE_SIZE) {
        oob_size = UFFS_MAX_SPARE_SIZE;
    }

    if (data != RT_NULL) {
        dev->st.io_read += data_len;
        dev->st.page_read_count++;
    }

    rt_memset(spare, 0xFF, sizeof(spare));

    res = rt_mtd_nand_read(RT_MTD_NAND_DEVICE(dev->_private),
                           page, data, data_len, ts ? (rt_uint8_t *)&spare[0] : NULL,
                           ts ? oob_size : 0);
    if (res == 0) {
        res = UFFS_FLASH_NO_ERR;
    } else if (res == -1) {
        //TODO ecc correct, add code to use hardware do ecc correct
        res = UFFS_FLASH_ECC_OK;
    } else {
        res = UFFS_FLASH_ECC_FAIL;
    }

    if (ts != RT_NULL) {
        rt_mtd_nand_unmap_user(RT_MTD_NAND_DEVICE(dev->_private), (rt_uint8_t *)&spare[1], (rt_uint8_t *)&spare[0], 0, 16);

        // unload ts and ecc from spare, you can modify it if you like
        uffs_FlashUnloadSpare(dev, (const rt_uint8_t *)&spare[1], ts, RT_NULL);

        if ((spare[1][spare_len - 1] == 0xFF) && (res == UFFS_FLASH_NO_ERR)) {
            res = UFFS_FLASH_NOT_SEALED;
        }

        dev->st.io_read += spare_len;
        dev->st.spare_read_count++;
    }

    return res;
}

const uffs_FlashOps nand_ops =
{
    nand_init_flash,    /* InitFlash() */
    nand_release_flash, /* ReleaseFlash() */
    NULL,               /* ReadPage() */
    ReadPageWithLayout, /* ReadPageWithLayout */
    NULL,               /* WritePage() */
    WritePageWithLayout,/* WritePageWithLayout */

#if defined(RT_UFFS_USE_CHECK_MARK_FUNCITON)
    nand_check_block,
    nand_mark_badblock,
#else
    NULL,               /* IsBadBlock(), let UFFS take care of it. */
    NULL,               /* MarkBadBlock(), let UFFS take care of it. */
#endif
    nand_erase_block,   /* EraseBlock() */
    NULL,               /* CheckErasedBlock() */
};

static rt_uint8_t hw_flash_ecc_layout[UFFS_SPARE_LAYOUT_SIZE] = {0xFF, 0x00};
static rt_uint8_t hw_flash_data_layout[UFFS_SPARE_LAYOUT_SIZE] ={0x00, 0x09, 0xFF, 0x00};

void uffs_setup_storage(struct uffs_StorageAttrSt *attr,
                        struct rt_mtd_nand_device *nand)
{
    rt_memset(attr, 0, sizeof(struct uffs_StorageAttrSt));

//  attr->total_blocks = nand->end_block - nand->start_block + 1;/* no use */
    attr->page_data_size = nand->page_size;                /* page data size */
    attr->pages_per_block = nand->pages_per_block;         /* pages per block */
    attr->spare_size = nand->oob_size;                     /* page spare size */
    attr->ecc_opt = RT_CONFIG_UFFS_ECC_MODE;               /* ecc option */
    attr->ecc_size = nand->oob_size - nand->oob_free;        /* ecc size */
    attr->block_status_offs = 0;              /* indicate block bad or good, offset in spare */
    attr->layout_opt = RT_CONFIG_UFFS_LAYOUT;              /* let UFFS do the spare layout */

    /* initialize  _uffs_data_layout and _uffs_ecc_layout */
    rt_memcpy(attr->_uffs_data_layout, hw_flash_data_layout, UFFS_SPARE_LAYOUT_SIZE);
    rt_memcpy(attr->_uffs_ecc_layout, hw_flash_ecc_layout, UFFS_SPARE_LAYOUT_SIZE);

    attr->data_layout = attr->_uffs_data_layout;
    attr->ecc_layout = attr->_uffs_ecc_layout;
}
