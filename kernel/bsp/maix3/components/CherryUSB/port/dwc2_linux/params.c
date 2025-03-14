// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (C) 2004-2016 Synopsys, Inc.
 */

/*
 * This file is licensed under the BSD-3-Clause License.
 * You may obtain a copy of the License at:
 * https://opensource.org/licenses/BSD-3-Clause
 *
 */

#include "core.h"

/*
 * Gets host hardware parameters. Forces host mode if not currently in
 * host mode. Should be called immediately after a core soft reset in
 * order to get the reset values.
 */
static void dwc2_get_host_hwparams(struct dwc2_hsotg *hsotg)
{
    struct dwc2_hw_params *hw = &hsotg->hw_params;
    u32 gnptxfsiz;
    u32 hptxfsiz;

    dwc2_force_mode(hsotg, true);

    gnptxfsiz = dwc2_readl(hsotg, GNPTXFSIZ);
    hptxfsiz = dwc2_readl(hsotg, HPTXFSIZ);

    hw->host_nperio_tx_fifo_size = (gnptxfsiz & FIFOSIZE_DEPTH_MASK) >>
        FIFOSIZE_DEPTH_SHIFT;
    hw->host_perio_tx_fifo_size = (hptxfsiz & FIFOSIZE_DEPTH_MASK) >>
        FIFOSIZE_DEPTH_SHIFT;
}

/**
 * dwc2_get_hwparams() - During device initialization, read various hardware
 *                       configuration registers and interpret the contents.
 *
 * @hsotg: Programming view of the DWC_otg controller
 *
 */
int dwc2_get_hwparams(struct dwc2_hsotg *hsotg)
{
    struct dwc2_hw_params *hw = &hsotg->hw_params;
    unsigned int width;
    u32 hwcfg1, hwcfg2, hwcfg3, hwcfg4;
    u32 grxfsiz;

    dev_dbg(hsotg->dev, "%s()\n", __func__);

    hwcfg1 = dwc2_readl(hsotg, GHWCFG1);
    hwcfg2 = dwc2_readl(hsotg, GHWCFG2);
    hwcfg3 = dwc2_readl(hsotg, GHWCFG3);
    hwcfg4 = dwc2_readl(hsotg, GHWCFG4);
    grxfsiz = dwc2_readl(hsotg, GRXFSIZ);

    /* hwcfg1 */
    hw->dev_ep_dirs = hwcfg1;

    /* hwcfg2 */
    hw->op_mode = (hwcfg2 & GHWCFG2_OP_MODE_MASK) >>
        GHWCFG2_OP_MODE_SHIFT;
    hw->arch = (hwcfg2 & GHWCFG2_ARCHITECTURE_MASK) >>
        GHWCFG2_ARCHITECTURE_SHIFT;
    hw->enable_dynamic_fifo = !!(hwcfg2 & GHWCFG2_DYNAMIC_FIFO);
    hw->host_channels = 1 + ((hwcfg2 & GHWCFG2_NUM_HOST_CHAN_MASK) >>
                             GHWCFG2_NUM_HOST_CHAN_SHIFT);
    hw->hs_phy_type = (hwcfg2 & GHWCFG2_HS_PHY_TYPE_MASK) >>
        GHWCFG2_HS_PHY_TYPE_SHIFT;
    hw->fs_phy_type = (hwcfg2 & GHWCFG2_FS_PHY_TYPE_MASK) >>
        GHWCFG2_FS_PHY_TYPE_SHIFT;
    hw->num_dev_ep = (hwcfg2 & GHWCFG2_NUM_DEV_EP_MASK) >>
        GHWCFG2_NUM_DEV_EP_SHIFT;
    hw->nperio_tx_q_depth =
        (hwcfg2 & GHWCFG2_NONPERIO_TX_Q_DEPTH_MASK) >>
        GHWCFG2_NONPERIO_TX_Q_DEPTH_SHIFT << 1;
    hw->host_perio_tx_q_depth =
        (hwcfg2 & GHWCFG2_HOST_PERIO_TX_Q_DEPTH_MASK) >>
        GHWCFG2_HOST_PERIO_TX_Q_DEPTH_SHIFT << 1;
    hw->dev_token_q_depth =
        (hwcfg2 & GHWCFG2_DEV_TOKEN_Q_DEPTH_MASK) >>
        GHWCFG2_DEV_TOKEN_Q_DEPTH_SHIFT;

    /* hwcfg3 */
    width = (hwcfg3 & GHWCFG3_XFER_SIZE_CNTR_WIDTH_MASK) >>
        GHWCFG3_XFER_SIZE_CNTR_WIDTH_SHIFT;
    hw->max_transfer_size = (1 << (width + 11)) - 1;
    width = (hwcfg3 & GHWCFG3_PACKET_SIZE_CNTR_WIDTH_MASK) >>
        GHWCFG3_PACKET_SIZE_CNTR_WIDTH_SHIFT;
    hw->max_packet_count = (1 << (width + 4)) - 1;
    hw->i2c_enable = !!(hwcfg3 & GHWCFG3_I2C);
    hw->total_fifo_size = (hwcfg3 & GHWCFG3_DFIFO_DEPTH_MASK) >>
        GHWCFG3_DFIFO_DEPTH_SHIFT;
    hw->lpm_mode = !!(hwcfg3 & GHWCFG3_OTG_LPM_EN);

    /* hwcfg4 */
    hw->en_multiple_tx_fifo = !!(hwcfg4 & GHWCFG4_DED_FIFO_EN);
    hw->num_dev_perio_in_ep = (hwcfg4 & GHWCFG4_NUM_DEV_PERIO_IN_EP_MASK) >>
        GHWCFG4_NUM_DEV_PERIO_IN_EP_SHIFT;
    hw->num_dev_in_eps = (hwcfg4 & GHWCFG4_NUM_IN_EPS_MASK) >>
        GHWCFG4_NUM_IN_EPS_SHIFT;
    hw->dma_desc_enable = !!(hwcfg4 & GHWCFG4_DESC_DMA);
    hw->power_optimized = !!(hwcfg4 & GHWCFG4_POWER_OPTIMIZ);
    hw->hibernation = !!(hwcfg4 & GHWCFG4_HIBER);
    hw->utmi_phy_data_width = (hwcfg4 & GHWCFG4_UTMI_PHY_DATA_WIDTH_MASK) >>
        GHWCFG4_UTMI_PHY_DATA_WIDTH_SHIFT;
    hw->acg_enable = !!(hwcfg4 & GHWCFG4_ACG_SUPPORTED);
    hw->ipg_isoc_en = !!(hwcfg4 & GHWCFG4_IPG_ISOC_SUPPORTED);
    hw->service_interval_mode = !!(hwcfg4 &
                                   GHWCFG4_SERVICE_INTERVAL_SUPPORTED);

    /* fifo sizes */
    hw->rx_fifo_size = (grxfsiz & GRXFSIZ_DEPTH_MASK) >>
        GRXFSIZ_DEPTH_SHIFT;
    /*
     * Host specific hardware parameters. Reading these parameters
     * requires the controller to be in host mode. The mode will
     * be forced, if necessary, to read these values.
     */
    dwc2_get_host_hwparams(hsotg);

    dev_dbg(hsotg->dev, "Detected values from hardware:\n");
    dev_dbg(hsotg->dev, "  op_mode=%d\n",
            hw->op_mode);
    dev_dbg(hsotg->dev, "  arch=%d\n",
            hw->arch);
    dev_dbg(hsotg->dev, "  dma_desc_enable=%d\n",
            hw->dma_desc_enable);
    dev_dbg(hsotg->dev, "  power_optimized=%d\n",
            hw->power_optimized);
    dev_dbg(hsotg->dev, "  i2c_enable=%d\n",
            hw->i2c_enable);
    dev_dbg(hsotg->dev, "  hs_phy_type=%d\n",
            hw->hs_phy_type);
    dev_dbg(hsotg->dev, "  fs_phy_type=%d\n",
            hw->fs_phy_type);
    dev_dbg(hsotg->dev, "  utmi_phy_data_width=%d\n",
            hw->utmi_phy_data_width);
    dev_dbg(hsotg->dev, "  num_dev_ep=%d\n",
            hw->num_dev_ep);
    dev_dbg(hsotg->dev, "  num_dev_perio_in_ep=%d\n",
            hw->num_dev_perio_in_ep);
    dev_dbg(hsotg->dev, "  host_channels=%d\n",
            hw->host_channels);
    dev_dbg(hsotg->dev, "  max_transfer_size=%d\n",
            hw->max_transfer_size);
    dev_dbg(hsotg->dev, "  max_packet_count=%d\n",
            hw->max_packet_count);
    dev_dbg(hsotg->dev, "  nperio_tx_q_depth=0x%0x\n",
            hw->nperio_tx_q_depth);
    dev_dbg(hsotg->dev, "  host_perio_tx_q_depth=0x%0x\n",
            hw->host_perio_tx_q_depth);
    dev_dbg(hsotg->dev, "  dev_token_q_depth=0x%0x\n",
            hw->dev_token_q_depth);
    dev_dbg(hsotg->dev, "  enable_dynamic_fifo=%d\n",
            hw->enable_dynamic_fifo);
    dev_dbg(hsotg->dev, "  en_multiple_tx_fifo=%d\n",
            hw->en_multiple_tx_fifo);
    dev_dbg(hsotg->dev, "  total_fifo_size=%d\n",
            hw->total_fifo_size);
    dev_dbg(hsotg->dev, "  rx_fifo_size=%d\n",
            hw->rx_fifo_size);
    dev_dbg(hsotg->dev, "  host_nperio_tx_fifo_size=%d\n",
            hw->host_nperio_tx_fifo_size);
    dev_dbg(hsotg->dev, "  host_perio_tx_fifo_size=%d\n",
            hw->host_perio_tx_fifo_size);
    dev_dbg(hsotg->dev, "\n");
    return 0;
}


static void dwc2_set_param_otg_cap(struct dwc2_hsotg *hsotg)
{
    switch (hsotg->hw_params.op_mode) {
    case GHWCFG2_OP_MODE_HNP_SRP_CAPABLE:
        hsotg->params.otg_caps.hnp_support = true;
        hsotg->params.otg_caps.srp_support = true;
        break;
    case GHWCFG2_OP_MODE_SRP_ONLY_CAPABLE:
    case GHWCFG2_OP_MODE_SRP_CAPABLE_DEVICE:
    case GHWCFG2_OP_MODE_SRP_CAPABLE_HOST:
        hsotg->params.otg_caps.hnp_support = false;
        hsotg->params.otg_caps.srp_support = true;
        break;
    default:
        hsotg->params.otg_caps.hnp_support = false;
        hsotg->params.otg_caps.srp_support = false;
        break;
    }
}

static void dwc2_set_param_phy_type(struct dwc2_hsotg *hsotg)
{
    int val;
    u32 hs_phy_type = hsotg->hw_params.hs_phy_type;

    val = DWC2_PHY_TYPE_PARAM_FS;
    if (hs_phy_type != GHWCFG2_HS_PHY_TYPE_NOT_SUPPORTED) {
        if (hs_phy_type == GHWCFG2_HS_PHY_TYPE_UTMI ||
            hs_phy_type == GHWCFG2_HS_PHY_TYPE_UTMI_ULPI)
            val = DWC2_PHY_TYPE_PARAM_UTMI;
        else
            val = DWC2_PHY_TYPE_PARAM_ULPI;
    }

    if (dwc2_is_fs_iot(hsotg))
        hsotg->params.phy_type = DWC2_PHY_TYPE_PARAM_FS;

    hsotg->params.phy_type = val;
}

static void dwc2_set_param_speed(struct dwc2_hsotg *hsotg)
{
    int val;

    val = hsotg->params.phy_type == DWC2_PHY_TYPE_PARAM_FS ?
        DWC2_SPEED_PARAM_FULL : DWC2_SPEED_PARAM_HIGH;

    if (dwc2_is_fs_iot(hsotg))
        val = DWC2_SPEED_PARAM_FULL;

    if (dwc2_is_hs_iot(hsotg))
        val = DWC2_SPEED_PARAM_HIGH;

    hsotg->params.speed = val;
}

static void dwc2_set_param_phy_utmi_width(struct dwc2_hsotg *hsotg)
{
    int val;

    val = (hsotg->hw_params.utmi_phy_data_width ==
           GHWCFG4_UTMI_PHY_DATA_WIDTH_8) ? 8 : 16;

#if 0
    /* todo:  no generic phy */
    if (hsotg->phy) {
        /*
         * If using the generic PHY framework, check if the PHY bus
         * width is 8-bit and set the phyif appropriately.
         */
        if (phy_get_bus_width(hsotg->phy) == 8)
            val = 8;
    }
#endif

    hsotg->params.phy_utmi_width = val;
}

static void dwc2_set_param_power_down(struct dwc2_hsotg *hsotg)
{
    int val;

    if (hsotg->hw_params.hibernation)
        val = DWC2_POWER_DOWN_PARAM_HIBERNATION;
    else if (hsotg->hw_params.power_optimized)
        val = DWC2_POWER_DOWN_PARAM_PARTIAL;
    else
        val = DWC2_POWER_DOWN_PARAM_NONE;

    hsotg->params.power_down = val;
}

static void dwc2_set_param_lpm(struct dwc2_hsotg *hsotg)
{
    struct dwc2_core_params *p = &hsotg->params;

    p->lpm = hsotg->hw_params.lpm_mode;
    if (p->lpm) {
        p->lpm_clock_gating = true;
        p->besl = true;
        p->hird_threshold_en = true;
        p->hird_threshold = 4;
    } else {
        p->lpm_clock_gating = false;
        p->besl = false;
        p->hird_threshold_en = false;
    }
}

/**
 * dwc2_set_default_params() - Set all core parameters to their
 * auto-detected default values.
 *
 * @hsotg: Programming view of the DWC_otg controller
 *
 */
static void dwc2_set_default_params(struct dwc2_hsotg *hsotg)
{
    struct dwc2_hw_params *hw = &hsotg->hw_params;
    struct dwc2_core_params *p = &hsotg->params;
    bool dma_capable = !(hw->arch == GHWCFG2_SLAVE_ONLY_ARCH);

    dwc2_set_param_otg_cap(hsotg);
    dwc2_set_param_phy_type(hsotg);
    dwc2_set_param_speed(hsotg);
    dwc2_set_param_phy_utmi_width(hsotg);
    dwc2_set_param_power_down(hsotg);
    dwc2_set_param_lpm(hsotg);
    p->phy_ulpi_ddr = false;
    p->phy_ulpi_ext_vbus = false;
    p->eusb2_disc = false;

    p->enable_dynamic_fifo = hw->enable_dynamic_fifo;
    p->en_multiple_tx_fifo = hw->en_multiple_tx_fifo;
    p->i2c_enable = hw->i2c_enable;
    p->acg_enable = hw->acg_enable;
    p->ulpi_fs_ls = false;
    p->ts_dline = false;
    p->reload_ctl = (hw->snpsid >= DWC2_CORE_REV_2_92a);
    p->uframe_sched = true;
    p->external_id_pin_ctl = false;
    p->ipg_isoc_en = false;
    p->service_interval = false;
    p->max_packet_count = hw->max_packet_count;
    p->max_transfer_size = hw->max_transfer_size;
    p->ahbcfg = GAHBCFG_HBSTLEN_INCR << GAHBCFG_HBSTLEN_SHIFT;
    p->ref_clk_per = 33333;
    p->sof_cnt_wkup_alert = 100;

    {
        p->host_dma = dma_capable;
#ifdef DWC2_DRV_DMA_DESC_ENABLE
        p->dma_desc_enable = true;
#else
        p->dma_desc_enable = false;
#endif
        p->dma_desc_fs_enable = false;
        p->host_support_fs_ls_low_power = false;
        p->host_ls_low_power_phy_clk = false;
        p->host_channels = hw->host_channels;
        p->host_rx_fifo_size = hw->rx_fifo_size;
        p->host_nperio_tx_fifo_size = hw->host_nperio_tx_fifo_size;
        p->host_perio_tx_fifo_size = hw->host_perio_tx_fifo_size;
    }
}

static void dwc2_check_param_otg_cap(struct dwc2_hsotg *hsotg)
{
    int valid = 1;

    if (hsotg->params.otg_caps.hnp_support && hsotg->params.otg_caps.srp_support) {
        /* check HNP && SRP capable */
        if (hsotg->hw_params.op_mode != GHWCFG2_OP_MODE_HNP_SRP_CAPABLE)
            valid = 0;
    } else if (!hsotg->params.otg_caps.hnp_support) {
        /* check SRP only capable */
        if (hsotg->params.otg_caps.srp_support) {
            switch (hsotg->hw_params.op_mode) {
            case GHWCFG2_OP_MODE_HNP_SRP_CAPABLE:
            case GHWCFG2_OP_MODE_SRP_ONLY_CAPABLE:
            case GHWCFG2_OP_MODE_SRP_CAPABLE_DEVICE:
            case GHWCFG2_OP_MODE_SRP_CAPABLE_HOST:
                break;
            default:
                valid = 0;
                break;
            }
        }
        /* else: NO HNP && NO SRP capable: always valid */
    } else {
        valid = 0;
    }

    if (!valid)
        dwc2_set_param_otg_cap(hsotg);
}

static void dwc2_check_param_phy_type(struct dwc2_hsotg *hsotg)
{
    int valid = 0;
    u32 hs_phy_type;
    u32 fs_phy_type;

    hs_phy_type = hsotg->hw_params.hs_phy_type;
    fs_phy_type = hsotg->hw_params.fs_phy_type;

    switch (hsotg->params.phy_type) {
    case DWC2_PHY_TYPE_PARAM_FS:
        if (fs_phy_type == GHWCFG2_FS_PHY_TYPE_DEDICATED)
            valid = 1;
        break;
    case DWC2_PHY_TYPE_PARAM_UTMI:
        if ((hs_phy_type == GHWCFG2_HS_PHY_TYPE_UTMI) ||
            (hs_phy_type == GHWCFG2_HS_PHY_TYPE_UTMI_ULPI))
            valid = 1;
        break;
    case DWC2_PHY_TYPE_PARAM_ULPI:
        if ((hs_phy_type == GHWCFG2_HS_PHY_TYPE_UTMI) ||
            (hs_phy_type == GHWCFG2_HS_PHY_TYPE_UTMI_ULPI))
            valid = 1;
        break;
    default:
        break;
    }

    if (!valid)
        dwc2_set_param_phy_type(hsotg);
}

static void dwc2_check_param_speed(struct dwc2_hsotg *hsotg)
{
    int valid = 1;
    int phy_type = hsotg->params.phy_type;
    int speed = hsotg->params.speed;

    switch (speed) {
    case DWC2_SPEED_PARAM_HIGH:
        if ((hsotg->params.speed == DWC2_SPEED_PARAM_HIGH) &&
            (phy_type == DWC2_PHY_TYPE_PARAM_FS))
            valid = 0;
        break;
    case DWC2_SPEED_PARAM_FULL:
    case DWC2_SPEED_PARAM_LOW:
        break;
    default:
        valid = 0;
        break;
    }

    if (!valid)
        dwc2_set_param_speed(hsotg);
}

static void dwc2_check_param_phy_utmi_width(struct dwc2_hsotg *hsotg)
{
    int valid = 0;
    int param = hsotg->params.phy_utmi_width;
    int width = hsotg->hw_params.utmi_phy_data_width;

    switch (width) {
    case GHWCFG4_UTMI_PHY_DATA_WIDTH_8:
        valid = (param == 8);
        break;
    case GHWCFG4_UTMI_PHY_DATA_WIDTH_16:
        valid = (param == 16);
        break;
    case GHWCFG4_UTMI_PHY_DATA_WIDTH_8_OR_16:
        valid = (param == 8 || param == 16);
        break;
    }

    if (!valid)
        dwc2_set_param_phy_utmi_width(hsotg);
}

static void dwc2_check_param_power_down(struct dwc2_hsotg *hsotg)
{
    int param = hsotg->params.power_down;

    switch (param) {
    case DWC2_POWER_DOWN_PARAM_NONE:
        break;
    case DWC2_POWER_DOWN_PARAM_PARTIAL:
        if (hsotg->hw_params.power_optimized)
            break;
        dev_dbg(hsotg->dev,
                "Partial power down isn't supported by HW\n");
        param = DWC2_POWER_DOWN_PARAM_NONE;
        break;
    case DWC2_POWER_DOWN_PARAM_HIBERNATION:
        if (hsotg->hw_params.hibernation)
            break;
        dev_dbg(hsotg->dev,
                "Hibernation isn't supported by HW\n");
        param = DWC2_POWER_DOWN_PARAM_NONE;
        break;
    default:
        dev_err(hsotg->dev,
                "%s: Invalid parameter power_down=%d\n",
                __func__, param);
        param = DWC2_POWER_DOWN_PARAM_NONE;
        break;
    }

    hsotg->params.power_down = param;
}

static void dwc2_check_param_eusb2_disc(struct dwc2_hsotg *hsotg)
{
    u32 gsnpsid;

    if (!hsotg->params.eusb2_disc)
        return;
    gsnpsid = dwc2_readl(hsotg, GSNPSID);
    /*
     * eusb2_disc not supported by FS IOT devices.
     * For other cores, it supported starting from version 5.00a
     */
    if ((gsnpsid & ~DWC2_CORE_REV_MASK) == DWC2_FS_IOT_ID ||
        (gsnpsid & DWC2_CORE_REV_MASK) <
        (DWC2_CORE_REV_5_00a & DWC2_CORE_REV_MASK)) {
        hsotg->params.eusb2_disc = false;
        return;
    }
}

#define CHECK_RANGE(_param, _min, _max, _def) do {			\
    if ((int)(hsotg->params._param) < (_min) ||		\
        (hsotg->params._param) > (_max)) {			\
        dev_warn(hsotg->dev, "%s: Invalid parameter %s=%d\n", \
                 __func__, #_param, hsotg->params._param); \
        hsotg->params._param = (_def);			\
    }							\
} while (0)

#define CHECK_BOOL(_param, _check) do {					\
    if (hsotg->params._param && !(_check)) {		\
        dev_warn(hsotg->dev, "%s: Invalid parameter %s=%d\n", \
                 __func__, #_param, hsotg->params._param); \
        hsotg->params._param = false;			\
    }							\
} while (0)

static void dwc2_check_params(struct dwc2_hsotg *hsotg)
{
    struct dwc2_hw_params *hw = &hsotg->hw_params;
    struct dwc2_core_params *p = &hsotg->params;
    bool dma_capable = !(hw->arch == GHWCFG2_SLAVE_ONLY_ARCH);

    dwc2_check_param_otg_cap(hsotg);
    dwc2_check_param_phy_type(hsotg);
    dwc2_check_param_speed(hsotg);
    dwc2_check_param_phy_utmi_width(hsotg);
    dwc2_check_param_power_down(hsotg);
    dwc2_check_param_eusb2_disc(hsotg);

    CHECK_BOOL(enable_dynamic_fifo, hw->enable_dynamic_fifo);
    CHECK_BOOL(en_multiple_tx_fifo, hw->en_multiple_tx_fifo);
    CHECK_BOOL(i2c_enable, hw->i2c_enable);
    CHECK_BOOL(ipg_isoc_en, hw->ipg_isoc_en);
    CHECK_BOOL(acg_enable, hw->acg_enable);
    CHECK_BOOL(reload_ctl, (hsotg->hw_params.snpsid > DWC2_CORE_REV_2_92a));
    CHECK_BOOL(lpm, (hsotg->hw_params.snpsid >= DWC2_CORE_REV_2_80a));
    CHECK_BOOL(lpm, hw->lpm_mode);
    CHECK_BOOL(lpm_clock_gating, hsotg->params.lpm);
    CHECK_BOOL(besl, hsotg->params.lpm);
    CHECK_BOOL(besl, (hsotg->hw_params.snpsid >= DWC2_CORE_REV_3_00a));
    CHECK_BOOL(hird_threshold_en, hsotg->params.lpm);
    CHECK_RANGE(hird_threshold, 0, hsotg->params.besl ? 12 : 7, 0);
    CHECK_BOOL(service_interval, hw->service_interval_mode);
    CHECK_RANGE(max_packet_count,
                15, hw->max_packet_count,
                hw->max_packet_count);
    CHECK_RANGE(max_transfer_size,
                2047, hw->max_transfer_size,
                hw->max_transfer_size);

    {
        CHECK_BOOL(host_dma, dma_capable);
        CHECK_BOOL(dma_desc_enable, p->host_dma);
        CHECK_BOOL(dma_desc_fs_enable, p->dma_desc_enable);
        CHECK_BOOL(host_ls_low_power_phy_clk,
                   p->phy_type == DWC2_PHY_TYPE_PARAM_FS);
        CHECK_RANGE(host_channels,
                    1, hw->host_channels,
                    hw->host_channels);
        CHECK_RANGE(host_rx_fifo_size,
                    16, hw->rx_fifo_size,
                    hw->rx_fifo_size);
        CHECK_RANGE(host_nperio_tx_fifo_size,
                    16, hw->host_nperio_tx_fifo_size,
                    hw->host_nperio_tx_fifo_size);
        CHECK_RANGE(host_perio_tx_fifo_size,
                    16, hw->host_perio_tx_fifo_size,
                    hw->host_perio_tx_fifo_size);
    }
}

int dwc2_init_params(struct dwc2_hsotg *hsotg)
{

    dwc2_set_default_params(hsotg);

    struct dwc2_core_params *p = &hsotg->params;

    {
        p->lpm = false;
        p->lpm_clock_gating = false;
        p->besl = false;
        p->hird_threshold_en = false;

        p->host_rx_fifo_size = 1024;
        p->host_nperio_tx_fifo_size = 512;
        p->host_perio_tx_fifo_size = 1024;
        p->ahbcfg = GAHBCFG_HBSTLEN_INCR16 <<
            GAHBCFG_HBSTLEN_SHIFT;
    }

    dev_dbg(hsotg->dev, "sw set params:\n");
    dev_dbg(hsotg->dev, "  host_dma=%d\n",
            p->host_dma);
    dev_dbg(hsotg->dev, "  dma_desc_enable=%d\n",
            p->dma_desc_enable);
    dev_dbg(hsotg->dev, "  power_down=%d\n",
            p->power_down);
    dev_dbg(hsotg->dev, "  i2c_enable=%d\n",
            p->i2c_enable);
    dev_dbg(hsotg->dev, "  phy_type=%d\n",
            p->phy_type);
    dev_dbg(hsotg->dev, "  phy_utmi_width=%d\n",
            p->phy_utmi_width);
    dev_dbg(hsotg->dev, "  host_channels=%d\n",
            p->host_channels);
    dev_dbg(hsotg->dev, "  max_transfer_size=%d\n",
            p->max_transfer_size);
    dev_dbg(hsotg->dev, "  max_packet_count=%d\n",
            p->max_packet_count);
    dev_dbg(hsotg->dev, "  enable_dynamic_fifo=%d\n",
            p->enable_dynamic_fifo);
    dev_dbg(hsotg->dev, "  en_multiple_tx_fifo=%d\n",
            p->en_multiple_tx_fifo);
    dev_dbg(hsotg->dev, "  host_rx_fifo_size=%d\n",
            p->host_rx_fifo_size);
    dev_dbg(hsotg->dev, "  host_nperio_tx_fifo_size=%d\n",
            p->host_nperio_tx_fifo_size);
    dev_dbg(hsotg->dev, "  host_perio_tx_fifo_size=%d\n",
            p->host_perio_tx_fifo_size);
    dev_dbg(hsotg->dev, "  uframe_sched=%d\n",
            p->uframe_sched);
    dev_dbg(hsotg->dev, "\n");
    dwc2_check_params(hsotg);

    return 0;
}
