#include "display_hal.h"

#include "rtthread.h"

#include "k_dma_comm.h"
#include "mpi_connector_api.h"
#include "mpi_vb_api.h"
#include "mpi_vo_api.h"
#include "mpi_sys_api.h"
#include "mpi_dma_api.h"

#define DBG_TAG    "DISP_HAL"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static k_connector_info s_connector_info;
static k_vo_rotation s_rotation_flag = K_ROTATION_270;
static k_vo_mirror_mode s_mirror_mode_flag = K_VO_MIRROR_NONE;
static k_u32 s_display_width = 0;
static k_u32 s_display_height = 0;
static k_pixel_format s_pixel_format = PIXEL_FORMAT_RGB_888;
static uint32_t s_pixel_format_bpp = 24;
static size_t s_buffer_size = 0;
static k_s32 s_vb_pool_id = VB_INVALID_POOLID;
static k_u8 s_dma_dev_chn = -1;
static terminal_vb_blk_info_t s_vb_blk_info_disp;
static k_video_frame_info s_vf_info_disp;
static terminal_vb_blk_info_t s_vb_blk_info_rotation;
static k_video_frame_info s_vf_info_rotation;
static terminal_vb_blk_info_t s_vb_blk_info_dma;
static k_vo_osd s_vo_osd_layer = K_VO_OSD0;

static k_u32 width_to_stride(k_u32 width) {
    k_u32 stride = width * s_pixel_format_bpp;
    stride = (stride + 7) >> 3; /*Round up*/
    return stride;
}

static k_s32 display_connector_init() {
    k_s32 ret = 0;

    k_connector_type connector_type = ST7701_V1_MIPI_2LAN_480X800_30FPS;
    // k_connector_type connector_type = ST7701_V1_MIPI_2LAN_480X640_30FPS;
    ret = kd_mpi_get_connector_info(connector_type, &s_connector_info);
    if (ret) {
        LOG_E("connector type %d not found\n", connector_type);
        return ret;
    }

    k_s32 connector_fd = kd_mpi_connector_open(s_connector_info.connector_name);
    if (connector_fd < 0) {
        LOG_E("%s, connector open failed.\n", __func__);
        return ret;
    }
    // set connect power
    kd_mpi_connector_power_set(connector_fd, K_TRUE);
    // connector init
    kd_mpi_connector_init(connector_fd, s_connector_info);
    // close fd
    kd_mpi_connector_close(connector_fd);
    return ret;
}

static k_s32 display_connector_deinit() {
    k_s32 ret = 0;

    k_s32 connector_fd = kd_mpi_connector_open(s_connector_info.connector_name);
    if (connector_fd < 0) {
        LOG_E("%s, connector open failed.\n", __func__);
        return ret;
    }

    ret = kd_mpi_connector_power_set(connector_fd, K_FALSE);
    if (ret) {
        LOG_E("kd_mpi_connector_power_set off failed.\n");
        return ret;
    }

    // close fd
    kd_mpi_connector_close(connector_fd);

    return ret;
}

static rt_err_t get_vb_blk_info(struct terminal_vb_blk_info *info, k_u64 blk_size) {

    k_vb_blk_handle handle = kd_mpi_vb_get_block(s_vb_pool_id, blk_size, RT_NULL);
    if (handle == VB_INVALID_HANDLE) {
        LOG_E("kd_mpi_vb_get_block failed\n");
        return RT_ERROR;
    }

    // k_s32 pool_id = kd_mpi_vb_handle_to_pool_id(handle);
    // if (pool_id == VB_INVALID_POOLID) {
    //     LOG_E("kd_mpi_vb_handle_to_pool_id failed\n");
    //     kd_mpi_vb_release_block(handle);
    //     return RT_ERROR;
    // }

    k_u64 phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);
    if ((void *)phys_addr == RT_NULL) {
        LOG_E("kd_mpi_vb_handle_to_phyaddr faile\n");
        kd_mpi_vb_release_block(handle);
        return RT_ERROR;
    }

    void* virt_addr = kd_mpi_sys_mmap(phys_addr, blk_size);
    if (virt_addr == RT_NULL) {
        LOG_E("kd_mpi_sys_mmap faile\n");
        kd_mpi_vb_release_block(handle);
        return RT_ERROR;
    }

    // info->pool_id = pool_id;
    info->pool_id = s_vb_pool_id;
    info->blk_size = blk_size;
    info->phys_addr = phys_addr;
    info->virt_addr = (k_u64)virt_addr;

    return RT_EOK;
}

static k_s32 vb_init(k_u64 buffer_size) {
    k_s32 ret = K_SUCCESS;

    k_vb_config config;
    rt_memset(&config, 0, sizeof(config));
    config.max_pool_cnt = VB_MAX_COMM_POOLS;
    // config.comm_pool[0].blk_cnt = 4;
    // config.comm_pool[0].blk_size = buffer_size;
    // config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;

    // for (size_t i = 1; i < 9; i++) {
    //     config.comm_pool[i] = config.comm_pool[0];
    // }

    ret = kd_mpi_vb_set_config(&config);
    if (ret) {
        LOG_E("kd_mpi_vb_set_config failed: %d\n", ret);
        return ret;
    }

    ret = kd_mpi_vb_init();
    if (ret) {
        LOG_E("kd_mpi_vb_init failed: %d\n", ret);
        return ret;
    }

    k_vb_pool_config pool_config;
    rt_memset(&pool_config, 0, sizeof(pool_config));
    pool_config.blk_cnt = 5;
    pool_config.blk_size = buffer_size;
    pool_config.mode = VB_REMAP_MODE_NOCACHE;

    s_vb_pool_id = kd_mpi_vb_create_pool(&pool_config);

    if (s_vb_pool_id == VB_INVALID_POOLID) {
        LOG_E("kd_mpi_vb_init create pool failed\n");
        return K_FAILED;
    }

    get_vb_blk_info(&s_vb_blk_info_rotation, buffer_size);
    get_vb_blk_info(&s_vb_blk_info_disp, buffer_size);
    get_vb_blk_info(&s_vb_blk_info_dma, buffer_size);

    return ret;
}

static k_s32 vo_init(k_vo_osd vo_osd_layer, k_u32 x, k_u32 y, k_u32 width, k_u32 height) {

    k_u32 disp_width = s_connector_info.resolution.hdisplay;
    k_u32 rotated_x = x;
    k_u32 rotated_y = y;
    k_u32 rotated_width = width;
    k_u32 rotated_height = height;

    switch (s_rotation_flag)
    {                                                                                                                                                                                                                                                                                                            
        case K_ROTATION_0:
            break;
        case K_ROTATION_90:
            rotated_x = disp_width - height - y;
            rotated_y = x;
            rotated_width = height;
            rotated_height = width;
            break;
        case K_ROTATION_180:
            break;
        case K_ROTATION_270:
            rotated_x = disp_width - height - y;
            rotated_y = x;
            rotated_width = height;
            rotated_height = width;
            break;
        default:
            RT_ASSERT("invalid arg!");
            break;
    }

    s_vf_info_rotation.pool_id = s_vb_blk_info_rotation.pool_id;
    s_vf_info_rotation.mod_id = K_ID_DMA;
    s_vf_info_rotation.v_frame.width = width;
    s_vf_info_rotation.v_frame.height = height;
    s_vf_info_rotation.v_frame.pixel_format = s_pixel_format;
    s_vf_info_rotation.v_frame.stride[0] = width_to_stride(width);
    s_vf_info_rotation.v_frame.phys_addr[0] = s_vb_blk_info_rotation.phys_addr;
    s_vf_info_rotation.v_frame.virt_addr[0] = s_vb_blk_info_rotation.virt_addr;

    s_vf_info_disp.pool_id = s_vb_blk_info_disp.pool_id;
    s_vf_info_disp.mod_id = K_ID_VO;
    s_vf_info_disp.v_frame.width = rotated_width;
    s_vf_info_disp.v_frame.height = rotated_height;
    s_vf_info_disp.v_frame.pixel_format = s_pixel_format;
    k_u32 rotated_stride = width_to_stride(rotated_width);
    s_vf_info_disp.v_frame.stride[0] = rotated_stride;
    s_vf_info_disp.v_frame.phys_addr[0] = s_vb_blk_info_disp.phys_addr;
    s_vf_info_disp.v_frame.virt_addr[0] = s_vb_blk_info_disp.virt_addr;

    k_vo_video_osd_attr attr = {
        .display_rect = {rotated_x, rotated_y},
        .img_size = {rotated_width, rotated_height},
        .pixel_format = s_pixel_format,
        .stride = rotated_stride / 8,
        .global_alptha = 0xff,
    };

    k_s32 ret = kd_mpi_vo_set_video_osd_attr(vo_osd_layer, &attr);
    if (ret) {
        LOG_E("kd_mpi_vo_set_video_osd_attr failed: %d\n", ret);
        return ret;
    }
    ret = kd_mpi_vo_osd_enable(vo_osd_layer);
    if (ret) {
        LOG_E("kd_mpi_vo_osd_enable failed: %d\n", ret);
        return ret;
    }
    ret = kd_mpi_vo_enable();
    if (ret) {
        LOG_E("kd_mpi_vo_enable failed: %d\n", ret);
        return ret;
    }

    s_vo_osd_layer = vo_osd_layer;

    return ret;
}

static k_s32 dma_dev_init(void) {
    k_dma_dev_attr_t dev_attr;
    dev_attr.burst_len = 0;
    dev_attr.ckg_bypass = K_TRUE;
    dev_attr.outstanding = 7;

    k_s32 ret = kd_mpi_dma_set_dev_attr(&dev_attr);
    if (ret != K_SUCCESS) {
        LOG_E("set dev attr error\n");
        return ret;
    }

    ret = kd_mpi_dma_start_dev();
    if (ret != K_SUCCESS) {
        LOG_E("start dev error\n");
        return ret;
    }

    s_dma_dev_chn = kd_mpi_dma_request_chn(GDMA_TYPE);
    if(s_dma_dev_chn < 0) {
        LOG_E("request gdma chn failed.\n");
        return K_FAILED;
    }

    k_u32 flag = s_rotation_flag | s_mirror_mode_flag;

    k_dma_chn_attr_u chn_attr = {
        .gdma_attr.buffer_num = 1,
        .gdma_attr.rotation = flag & K_ROTATION_0 ? DEGREE_0 :
            flag & K_ROTATION_90 ? DEGREE_90 :
            flag & K_ROTATION_180 ? DEGREE_180 :
            flag & K_ROTATION_270 ? DEGREE_270 : DEGREE_0,
        .gdma_attr.x_mirror = flag & (K_VO_MIRROR_HOR || K_VO_MIRROR_BOTH) ? 1 : 0,
        .gdma_attr.y_mirror = flag & (K_VO_MIRROR_VER || K_VO_MIRROR_BOTH) ? 1 : 0,
        .gdma_attr.width = s_vf_info_rotation.v_frame.width,
        .gdma_attr.height = s_vf_info_rotation.v_frame.height,
        .gdma_attr.src_stride[0] = s_vf_info_rotation.v_frame.stride[0],
        .gdma_attr.dst_stride[0] = s_vf_info_disp.v_frame.stride[0],
        .gdma_attr.work_mode = DMA_UNBIND,
        .gdma_attr.pixel_format =
            (s_vf_info_rotation.v_frame.pixel_format == PIXEL_FORMAT_ARGB_8888 ||
            s_vf_info_rotation.v_frame.pixel_format == PIXEL_FORMAT_ABGR_8888 ||
            s_vf_info_rotation.v_frame.pixel_format == PIXEL_FORMAT_BGRA_8888) ? DMA_PIXEL_FORMAT_ARGB_8888 :
            (s_vf_info_rotation.v_frame.pixel_format == PIXEL_FORMAT_RGB_888 ||
            s_vf_info_rotation.v_frame.pixel_format == PIXEL_FORMAT_BGR_888) ? DMA_PIXEL_FORMAT_RGB_888 :
            (s_vf_info_rotation.v_frame.pixel_format == 300 || s_vf_info_rotation.v_frame.pixel_format == 301) ? DMA_PIXEL_FORMAT_RGB_565:
            (s_vf_info_rotation.v_frame.pixel_format == PIXEL_FORMAT_RGB_MONOCHROME_8BPP) ? DMA_PIXEL_FORMAT_YUV_400_8BIT : 0,
    };

    kd_mpi_dma_attach_vb_pool(s_dma_dev_chn, s_vb_blk_info_dma.pool_id);

    ret = kd_mpi_dma_set_chn_attr(s_dma_dev_chn, &chn_attr);
    if (ret != K_SUCCESS) {
        LOG_E("kd_mpi_dma_set_chn_attr failed");
        return ret;
    }

    ret = kd_mpi_dma_start_chn(s_dma_dev_chn);
    if (ret != K_SUCCESS) {
        LOG_E("kd_mpi_dma_start_chn failed");
        return ret;
    }

}

static void dma_dev_deinit(void) {
    kd_mpi_dma_stop_chn(s_dma_dev_chn);
    kd_mpi_dma_release_chn(s_dma_dev_chn);
    kd_mpi_dma_stop_dev();
    s_dma_dev_chn = -1;
}

static k_s32 rotate_frame(k_u8 *px_map) {
    rt_memcpy((void *)s_vf_info_rotation.v_frame.virt_addr[0], px_map, s_buffer_size);

    k_s32 ret = kd_mpi_dma_send_frame(s_dma_dev_chn, &s_vf_info_rotation, -1);

    if (ret != K_SUCCESS) {
        LOG_E("kd_mpi_dma_send_frame failed");
        return ret;
    }

    k_video_frame_info tmp;
    ret = kd_mpi_dma_get_frame(s_dma_dev_chn, &tmp, -1);
    if (ret != K_SUCCESS) {
        LOG_E("kd_mpi_dma_get_frame failed");
        return ret;
    }

    void *tmp_addr = kd_mpi_sys_mmap_cached(tmp.v_frame.phys_addr[0], s_buffer_size);
    ret = kd_mpi_sys_mmz_flush_cache(tmp.v_frame.phys_addr[0], tmp_addr, s_buffer_size);
    if (ret != K_SUCCESS) {
        LOG_E("kd_mpi_sys_mmz_flush_cache failed");
        kd_mpi_dma_release_frame(s_dma_dev_chn, &tmp);
        return ret;
    }

    rt_memcpy((void *)s_vf_info_disp.v_frame.virt_addr[0], tmp_addr, s_buffer_size);

    kd_mpi_sys_munmap(tmp_addr, s_buffer_size);
    return kd_mpi_dma_release_frame(s_dma_dev_chn, &tmp);
}

k_s32 display_hal_init() {
    k_s32 ret = 0;

    ret = display_connector_init();
    if (ret) {
        LOG_E("display_connector_init failed: %d\n", ret);
        return K_FAILED;
    }

    k_u32 resolution_hdisplay = s_connector_info.resolution.hdisplay;
    k_u32 resolution_vdisplay = s_connector_info.resolution.vdisplay;

    switch (s_rotation_flag) {
        case K_ROTATION_0:
        case K_ROTATION_180:
            s_display_width = resolution_hdisplay;
            s_display_height = resolution_vdisplay;
            break;
        case K_ROTATION_90:
        case K_ROTATION_270:
            s_display_width = resolution_vdisplay;
            s_display_height = resolution_hdisplay;
            break;
        default:
            RT_ASSERT("invalid arg!");
            break;
    }

    s_buffer_size = width_to_stride(s_display_width) * s_display_height;

    ret = vb_init(s_buffer_size);
    if (ret) {
        LOG_E("vb_init failed: %d\n", ret);
        return ret;
    }

    ret = vo_init(K_VO_OSD0, 0, 0, s_display_width, s_display_height);
    if (ret) {
        LOG_E("vo_init failed: %d\n", ret);
        return ret;
    }

    ret = dma_dev_init();
    if (ret) {
        LOG_E("dma_dev_init failed: %d\n", ret);
        return ret;
    }

    return K_SUCCESS;
}

k_s32 display_hal_flush_frame(k_u8 *px_map) {
    k_s32 ret = rotate_frame(px_map);
    if (ret != K_SUCCESS) {
        LOG_E("roate frame failed: %d", ret);
    }

    kd_mpi_vo_chn_insert_frame(s_vo_osd_layer + K_MAX_VO_LAYER_NUM, &s_vf_info_disp);
}

k_u32 display_hal_get_width() {
    return s_display_width;
}

k_u32 display_hal_get_height() {
    return s_display_height;
}

k_u8 display_hal_get_dma_channel() {
    return s_dma_dev_chn;
}

k_vo_osd display_hal_get_osd_layer() {
    return s_vo_osd_layer;
}

k_video_frame_info * display_hal_get_disp_frame_info() {
    return &s_vf_info_disp;
}

k_video_frame_info * display_hal_get_rotation_frame_info() {
    return &s_vf_info_rotation;
}