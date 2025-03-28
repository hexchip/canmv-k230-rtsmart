/*
 * Copyright (c) 2023-2024, ArtInChip Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Authors: xuan.wen <xuan.wen@artinchip.com>
 */

#ifndef __SPINAND_H__
#define __SPINAND_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef BIT
#define BIT(s) (1U << (s))
#define BIT_UL(s) (1UL << (s))
#endif

#define BITS_PER_LONG sizeof(long)

#define GENMASK(h, l)       (((~(0U)) - ((1U) << (l)) + 1) & \
                             (~(0U) >> (BITS_PER_LONG - 1 - (h))))
#define GENMASK_UL(h, l)    (((~(0UL)) - ((1UL) << (l)) + 1) & \
                             (~(0UL) >> (BITS_PER_LONG - 1 - (h))))


#define SPINAND_SUCCESS     0 /**< success */
#define SPINAND_ERR         1 /**< not found or not supported */
#define SPINAND_ERR_TIMEOUT 2 /**< timeout error */
#define SPINAND_ERR_ECC     3 /**< ecc check error */
#define SPINAND_TRUE        1
#define SPINAND_FALSE       0

#define SPINAND_DIE_ID0 (0)
#define SPINAND_DIE_ID1 (1)

#undef pr_debug
#ifdef AIC_SPINAND_DRV_DEBUG
#define pr_debug pr_info
#else
#define pr_debug(fmt, ...)
#endif

/**
* struct nand_bbt - bad block table object
* @cache: in memory BBT cache
*/
struct nand_bbt {
    uint8_t *cache;
};

struct aic_spinand;

/*
 * struct spinand_devid - SPI NAND device id structure
 * @id: device id of current chip
 * @len: number of bytes in device id
*/
struct spinand_devid {
    const uint8_t *id;
    const uint8_t len;
};

#define DEVID(...)                              \
    {                                           \
        .id = (const uint8_t[]){ __VA_ARGS__ },      \
        .len = sizeof((uint8_t[]){ __VA_ARGS__ }),   \
    }

struct aic_oob_region {
    uint32_t offset;
    uint32_t length;
};

/* SPI NAND flash information */
struct aic_spinand_info {
    struct spinand_devid devid;
    uint16_t page_size;
    uint16_t oob_size;
    uint16_t block_per_lun;
    uint16_t pages_per_eraseblock;
    uint8_t planes_per_lun;
    uint8_t is_die_select;
    const char *sz_description;
    struct spi_nand_cmd_cfg *cmd;
    int (*get_status)(struct aic_spinand *flash, uint8_t status);
    int (*oob_get_user)(struct aic_spinand *flash, int section,
                            struct aic_oob_region *oobuser);
};
typedef struct aic_spinand_info *aic_spinand_info_t;

#define PAGESIZE(x) (uint16_t)(x)
#define OOBSIZE(x)  (uint16_t)(x)
#define BPL(x) (uint16_t)(x)
#define PPB(x) (uint16_t)(x)
#define PLANENUM(x) (uint8_t)(x)
#define DIE(x) (uint16_t)(x)

#define SPINAND_MAX_ID_LEN 4

struct spinand_id {
    uint8_t data[SPINAND_MAX_ID_LEN];
};

struct aic_spinand {
    const struct aic_spinand_info *info;
    struct spinand_id id;
    void *user_data;
    void *lock;
    uint8_t use_continuous_read;
    uint8_t qspi_dl_width;
    uint8_t IsInited;
    uint8_t *databuf;
    uint8_t *oobbuf;
    struct nand_bbt bbt;
};
typedef struct aic_spinand *aic_spinand_t;

int spinand_read_id_op(struct aic_spinand *flash, uint8_t *id);
int spinand_block_erase(struct aic_spinand *flash, uint16_t blk);
int spinand_flash_init(struct aic_spinand *flash);
int spinand_read_page(struct aic_spinand *flash, uint32_t page, uint8_t *data,
                      uint32_t data_len, uint8_t *spare, uint32_t spare_len);
int spinand_block_isbad(struct aic_spinand *flash, uint16_t blk);
int spinand_get_status(struct aic_spinand *flash, uint16_t blk);
int spinand_set_status(struct aic_spinand *flash, uint16_t blk, uint16_t pos, uint16_t status);
int spinand_continuous_read(struct aic_spinand *flash, uint32_t page, uint8_t *data,
                            uint32_t size);
int spinand_write_page(struct aic_spinand *flash, uint32_t page, const uint8_t *data,
                       uint32_t data_len, const uint8_t *spare, uint32_t spare_len);
int spinand_block_markbad(struct aic_spinand *flash, uint16_t blk);
int spinand_config_set(struct aic_spinand *flash, uint8_t mask, uint8_t val);
int spinand_erase(struct aic_spinand *flash, uint32_t offset, uint32_t size);
int spinand_read(struct aic_spinand *flash, uint8_t *addr, uint32_t offset, uint32_t size);
int spinand_write(struct aic_spinand *flash, uint8_t *addr, uint32_t offset, uint32_t size);

#ifdef AIC_SPINAND_CONT_READ

#define SPINAND_CMD_READ_FROM_CACHE_X4_CONT_CFG       \
    {                                                 \
        SPINAND_CMD_READ_FROM_CACHE_X4, 1, 0, 0, 4, 4 \
    }
#endif

/*
 * Standard SPI-NAND flash commands
 */
#define SPINAND_CMD_RESET                   0xff
#define SPINAND_CMD_GET_FEATURE             0x0f
#define SPINAND_CMD_SET_FEATURE             0x1f
#define SPINAND_CMD_PAGE_READ               0x13
#define SPINAND_CMD_READ_PAGE_CACHE_RDM     0x30
#define SPINAND_CMD_READ_PAGE_CACHE_LAST    0x3f
#define SPINAND_CMD_READ_FROM_CACHE         0x03
#define SPINAND_CMD_READ_FROM_CACHE_FAST    0x0b
#define SPINAND_CMD_READ_FROM_CACHE_X2      0x3b
#define SPINAND_CMD_READ_FROM_CACHE_DUAL_IO 0xbb
#define SPINAND_CMD_READ_FROM_CACHE_X4      0x6b
#define SPINAND_CMD_READ_FROM_CACHE_QUAD_IO 0xeb
#define SPINAND_CMD_BLK_ERASE               0xd8
#define SPINAND_CMD_PROG_EXC                0x10
#define SPINAND_CMD_PROG_LOAD               0x02
#define SPINAND_CMD_PROG_LOAD_RDM_DATA      0x84
#define SPINAND_CMD_PROG_LOAD_X4            0x32
#define SPINAND_CMD_PROG_LOAD_RDM_DATA_X4   0x34
#define SPINAND_CMD_READ_ID                 0x9f
#define SPINAND_CMD_WR_DISABLE              0x04
#define SPINAND_CMD_WR_ENABLE               0x06
#define SPINAND_CMD_END                     0x0

#define SPINAND_CMD_GET_FEATURE_CFG            \
    {                                          \
        SPINAND_CMD_GET_FEATURE, 1, 1, 1, 0, 1 \
    }
#define SPINAND_CMD_SET_FEATURE_CFG            \
    {                                          \
        SPINAND_CMD_SET_FEATURE, 1, 1, 1, 0, 1 \
    }
#define SPINAND_CMD_WR_ENABLE_CFG            \
    {                                        \
        SPINAND_CMD_WR_ENABLE, 1, 0, 0, 0, 0 \
    }
#define SPINAND_CMD_WR_DISABLE_CFG            \
    {                                         \
        SPINAND_CMD_WR_DISABLE, 1, 0, 0, 0, 0 \
    }
#define SPINAND_CMD_PROG_EXC_CFG            \
    {                                       \
        SPINAND_CMD_PROG_EXC, 1, 3, 1, 0, 0 \
    }
#define SPINAND_CMD_PAGE_READ_CFG            \
    {                                        \
        SPINAND_CMD_PAGE_READ, 1, 3, 1, 0, 0 \
    }
#define SPINAND_CMD_BLK_ERASE_CFG            \
    {                                        \
        SPINAND_CMD_BLK_ERASE, 1, 3, 1, 0, 0 \
    }
#define SPINAND_CMD_READ_ID_CFG            \
    {                                      \
        SPINAND_CMD_READ_ID, 1, 0, 1, 1, 1 \
    }
#define SPINAND_CMD_RESET_CFG            \
    {                                    \
        SPINAND_CMD_RESET, 1, 0, 0, 0, 0 \
    }

struct spi_nand_cmd_cfg {
    uint8_t opcode;
    uint8_t opcode_bits;
    uint8_t addr_bytes;
    uint8_t addr_bits;
    uint8_t dummy_bytes;
    uint8_t data_bits;
};

/* feature registers */
#define REG_BLOCK_LOCK 0xa0
#define REG_DIE_SELECT 0xd0

/* configuration register */
#define REG_CFG         0xb0
#define CFG_OTP_ENABLE  BIT(6)
#define CFG_ECC_ENABLE  BIT(4)
#define CFG_BUF_ENABLE  BIT(3)
#define CFG_QUAD_ENABLE BIT(0)

/* status register */
#define REG_STATUS          0xc0
#define STATUS_BUSY         BIT(0)
#define STATUS_ERASE_FAILED BIT(2)
#define STATUS_PROG_FAILED  BIT(3)

#define STATUS_ECC_MASK             GENMASK(5, 4)
#define STATUS_ECC_NO_BITFLIPS      (0 << 4)
#define STATUS_ECC_HAS_1_4_BITFLIPS (1 << 4)
#define STATUS_ECC_UNCOR_ERROR      (2 << 4)

#ifdef SPI_NAND_WINBOND
extern const struct spinand_manufacturer winbond_spinand_manufacturer;
#endif
#ifdef SPI_NAND_XTX
extern const struct spinand_manufacturer xtx_spinand_manufacturer;
#endif
#ifdef SPI_NAND_GIGADEVICE
extern const struct spinand_manufacturer gigadevice_spinand_manufacturer;
#endif
#ifdef SPI_NAND_FORESEE
extern const struct spinand_manufacturer foresee_spinand_manufacturer;
#endif
#ifdef SPI_NAND_TOSHIBA
extern const struct spinand_manufacturer toshiba_spinand_manufacturer;
#endif
#ifdef SPI_NAND_MACRONIX
extern const struct spinand_manufacturer macronix_spinand_manufacturer;
#endif
#ifdef SPI_NAND_ZETTA
extern const struct spinand_manufacturer zetta_spinand_manufacturer;
#endif
#ifdef SPI_NAND_DOSILICON
extern const struct spinand_manufacturer dosilicon_spinand_manufacturer;
#endif
#ifdef SPI_NAND_ETRON
extern const struct spinand_manufacturer etron_spinand_manufacturer;
#endif
#ifdef SPI_NAND_MICRON
extern const struct spinand_manufacturer micron_spinand_manufacturer;
#endif
#ifdef SPI_NAND_ZBIT
extern const struct spinand_manufacturer zbit_spinand_manufacturer;
#endif
#ifdef SPI_NAND_ESMT
extern const struct spinand_manufacturer esmt_spinand_manufacturer;
#endif
#ifdef SPI_NAND_UMTEK
extern const struct spinand_manufacturer umtek_spinand_manufacturer;
#endif
#ifdef SPI_NAND_QUANXING
extern const struct spinand_manufacturer quanxing_spinand_manufacturer;
#endif
#ifdef SPI_NAND_XINCUN
extern const struct spinand_manufacturer xincun_spinand_manufacturer;
#endif
#ifdef SPI_NAND_FUDANMICRO
extern const struct spinand_manufacturer fudanmicro_spinand_manufacturer;
#endif

extern struct spi_nand_cmd_cfg cmd_cfg_table[];

const struct aic_spinand_info *
spinand_match_and_init(uint8_t *devid, const struct aic_spinand_info *table,
                       uint32_t table_size);
int aic_spinand_transfer_message(struct aic_spinand *flash,
                                 struct spi_nand_cmd_cfg *cfg, uint32_t addr,
                                 uint8_t *sendBuff, uint8_t *recvBuff, uint32_t DataCount);
/* ooblayout
 * distinguish the ECC protected bytes on the oob region
 */
int spinand_ooblayout_map_user(struct aic_spinand *flash, uint8_t *oobbuf,
                       const uint8_t *spare, int start, int nbytes);
int spinand_ooblayout_unmap_user(struct aic_spinand *flash, uint8_t *dst,
                       uint8_t *src, int start, int nbytes);

#endif /* __SPINAND_H__ */
