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

#include "drv_fpioa.h"

#define VOL_BANK_IO0_1 BANK_VOL_1V8_MSC
#define VOL_BANK0_IO2_13 BANK_VOL_1V8_MSC
#define VOL_BANK1_IO14_25 BANK_VOL_1V8_MSC
#define VOL_BANK2_IO26_37 BANK_VOL_3V3_MSC
#define VOL_BANK3_IO38_49 BANK_VOL_1V8_MSC
#define VOL_BANK4_IO50_61 BANK_VOL_1V8_MSC
#define VOL_BANK5_IO62_63 BANK_VOL_1V8_MSC

/* clang-format off */

const board_pinmux_cfg_t board_pinmux_cfg[FPIOA_PIN_MAX_NUM] = {
    /* BOOT IO */
    [0] = { .func = BOOT0, .cfg = { .u.value = 0 } }, // BOOT0
    [1] = { .func = BOOT1, .cfg = { .u.value = 0 } }, // BOOT1

    /* BANK0 */
    [2] = { .func = JTAG_TCK, .cfg = { .u.value = 0 } }, // JTAG_TCK
    [3] = { .func = JTAG_TDI, .cfg = { .u.value = 0 } }, // JTAG_TDI
    [4] = { .func = JTAG_TDO, .cfg = { .u.value = 0 } }, // JTAG_TDO
    [5] = { .func = JTAG_TMS, .cfg = { .u.value = 0 } }, // JTAG_TMS
    [6] = { .func = JTAG_RST, .cfg = { .u.value = 0 } }, // JTAG_RST
    [7] = { .func = IIC4_SCL, .cfg = { .u.value = 0 } }, // IIC4_SCL
    [8] = { .func = IIC4_SDA, .cfg = { .u.value = 0 } }, // IIC4_SDA
    [9] = { .func = GPIO9, .cfg = { .u.value = 0 } }, // GPIO9
    [10] = { .func = CTRL_IN_3D, .cfg = { .u.value = 0 } }, // CTRL_IN_3D
    [11] = { .func = CTRL_O1_3D, .cfg = { .u.value = 0 } }, // CTRL_O1_3D
    [12] = { .func = CTRL_O2_3D, .cfg = { .u.value = 0 } }, // CTRL_O2_3D
    [13] = { .func = M_CLK1, .cfg = { .u.value = 0 } }, // M_CLK1

    /* BANK1 */
    [14] = { .func = OSPI_CS, .cfg = { .u.value = 0 } }, // OSPI_CS
    [15] = { .func = OSPI_CLK, .cfg = { .u.value = 0 } }, // OSPI_CLK
    [16] = { .func = OSPI_D0, .cfg = { .u.value = 0 } }, // OSPI_D0
    [17] = { .func = OSPI_D1, .cfg = { .u.value = 0 } }, // OSPI_D1
    [18] = { .func = OSPI_D2, .cfg = { .u.value = 0 } }, // OSPI_D2
    [19] = { .func = OSPI_D3, .cfg = { .u.value = 0 } }, // OSPI_D3
    [20] = { .func = OSPI_D4, .cfg = { .u.value = 0 } }, // OSPI_D4
    [21] = { .func = GPIO21, .cfg = { .u.value = 0 } }, // GPIO21
    [22] = { .func = OSPI_D6, .cfg = { .u.value = 0 } }, // OSPI_D6
    [23] = { .func = OSPI_D7, .cfg = { .u.value = 0 } }, // OSPI_D7
    [24] = { .func = OSPI_DQS, .cfg = { .u.value = 0 } }, // OSPI_DQS
    [25] = { .func = GPIO25, .cfg = { .u.value = 0 } }, // GPIO25

    /* BANK2 */
    [26] = { .func = PDM_CLK, .cfg = { .u.value = 0 } }, // PDM_CLK
    [27] = { .func = GPIO27, .cfg = { .u.value = 0 } }, // GPIO27
    [28] = { .func = GPIO28, .cfg = { .u.value = 0 } }, // GPIO28
    [29] = { .func = GPIO29, .cfg = { .u.value = 0 } }, // GPIO29
    [30] = { .func = GPIO30, .cfg = { .u.value = 0 } }, // GPIO30
    [31] = { .func = GPIO31, .cfg = { .u.value = 0 } }, // GPIO31
    [32] = { .func = GPIO32, .cfg = { .u.value = 0 } }, // GPIO32
    [33] = { .func = GPIO33, .cfg = { .u.value = 0 } }, // GPIO33
    [34] = { .func = IIS_D_IN0_PDM_IN3, .cfg = { .u.value = 0 } }, // IIS_D_IN0_PDM_IN3
    [35] = { .func = IIS_D_OUT0_PDM_IN1, .cfg = { .u.value = 0 } }, // IIS_D_OUT0_PDM_IN1
    [36] = { .func = IIS_D_IN1_PDM_IN2, .cfg = { .u.value = 0 } }, // IIS_D_IN1_PDM_IN2
    [37] = { .func = GPIO37, .cfg = { .u.value = 0 } }, // GPIO37

    /* BANK3 */
    [38] = { .func = UART0_TXD, .cfg = { .u.value = 0 } }, // UART0_TXD
    [39] = { .func = UART0_RXD, .cfg = { .u.value = 0 } }, // UART0_RXD
    [40] = { .func = IIC1_SCL, .cfg = { .u.value = 0 } }, // IIC1_SCL
    [41] = { .func = IIC1_SDA, .cfg = { .u.value = 0 } }, // IIC1_SDA
    [42] = { .func = GPIO42, .cfg = { .u.value = 0 } }, // GPIO42
    [43] = { .func = GPIO43, .cfg = { .u.value = 0 } }, // GPIO43
    [44] = { .func = IIC3_SCL, .cfg = { .u.value = 0 } }, // IIC3_SCL
    [45] = { .func = IIC3_SDA, .cfg = { .u.value = 0 } }, // IIC3_SDA
    [46] = { .func = GPIO46, .cfg = { .u.value = 0 } }, // GPIO46
    [47] = { .func = GPIO47, .cfg = { .u.value = 0 } }, // GPIO47
    [48] = { .func = GPIO48, .cfg = { .u.value = 0 } }, // GPIO48
    [49] = { .func = GPIO49, .cfg = { .u.value = 0 } }, // GPIO49

    /* BANK4 */
    [50] = { .func = UART3_TXD, .cfg = { .u.value = 0 } }, // UART3_TXD
    [51] = { .func = UART3_RXD, .cfg = { .u.value = 0 } }, // UART3_RXD
    [52] = { .func = GPIO52, .cfg = { .u.value = 0 } }, // GPIO52
    [53] = { .func = GPIO53, .cfg = { .u.value = 0 } }, // GPIO53
    [54] = { .func = MMC1_CMD, .cfg = { .u.value = 0 } }, // MMC1_CMD
    [55] = { .func = MMC1_CLK, .cfg = { .u.value = 0 } }, // MMC1_CLK
    [56] = { .func = MMC1_D0, .cfg = { .u.value = 0 } }, // MMC1_D0
    [57] = { .func = MMC1_D1, .cfg = { .u.value = 0 } }, // MMC1_D1
    [58] = { .func = MMC1_D2, .cfg = { .u.value = 0 } }, // MMC1_D2
    [59] = { .func = MMC1_D3, .cfg = { .u.value = 0 } }, // MMC1_D3
    [60] = { .func = IIC0_SCL, .cfg = { .u.value = 0 } }, // IIC0_SCL
    [61] = { .func = IIC0_SDA, .cfg = { .u.value = 0 } }, // IIC0_SDA

    /* BANK5 */
    [62] = { .func = M_CLK2, .cfg = { .u.value = 0 } }, // M_CLK2
    [63] = { .func = M_CLK3, .cfg = { .u.value = 0 } }, // M_CLK3
};
/* clang-format on */

static inline __attribute__((always_inline)) void board_specific_pin_init_sequence()
{
  kd_pin_mode(33, GPIO_DM_OUTPUT);
  kd_pin_write(33, GPIO_PV_LOW);
}
