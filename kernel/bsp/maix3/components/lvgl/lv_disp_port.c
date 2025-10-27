#include <stdio.h>

#include "lvgl.h"

#include "k_dma_comm.h"
#include "mpi_connector_api.h"
#include "mpi_vb_api.h"
#include "mpi_vo_api.h"
#include "mpi_sys_api.h"
#include "mpi_dma_api.h"

typedef struct terminal_vb_blk_info {
    k_s32 pool_id;
    k_u64 blk_size;
    k_u64 phys_addr;
    k_u64 virt_addr;
} terminal_vb_blk_info_t;

static k_connector_info s_connector_info;
static k_vo_rotation s_rotation_flag;
static k_vo_mirror_mode s_mirror_mode_flag;
static k_u8 s_dma_dev_chn;
static terminal_vb_blk_info_t s_vb_blk_info_disp;
static k_video_frame_info s_vf_info_disp;
static terminal_vb_blk_info_t s_vb_blk_info_rotation;
static k_video_frame_info s_vf_info_rotation;
static k_vo_osd s_vo_osd_layer;


static void dma_dev_deinit(void)
{
    kd_mpi_dma_stop_chn(s_dma_dev_chn);
    kd_mpi_dma_release_chn(s_dma_dev_chn);
    kd_mpi_dma_stop_dev();
    s_dma_dev_chn = -1;
}

static k_s32 display_connector_init() {
    k_s32 ret = 0;

    k_connector_type connector_type = ST7701_V1_MIPI_2LAN_480X640_30FPS;
    ret = kd_mpi_get_connector_info(connector_type, &s_connector_info);
    if (ret) {
        printf("connector type %d not found\n", connector_type);
        return ret;
    }

    k_s32 connector_fd = kd_mpi_connector_open(s_connector_info.connector_name);
    if (connector_fd < 0) {
        printf("%s, connector open failed.\n", __func__);
        return ret;
    }
    // set connect power
    kd_mpi_connector_power_set(connector_fd, 1);
    // set connect get id
    k_u32 chip_id = 0;
    kd_mpi_connector_id_get(connector_fd, &chip_id);
    // connector init
    kd_mpi_connector_init(connector_fd, s_connector_info);

    return ret;
}

static k_s32 vo_osd_rotation(k_video_frame_info *vf_info_rotation, k_video_frame_info *vf_info_disp, k_u8 s_dma_dev_chn) {
    k_s32 ret = kd_mpi_dma_send_frame(s_dma_dev_chn, vf_info_rotation, -1);

    if (ret != K_SUCCESS) {
        printf("kd_mpi_dma_send_frame failed\n");
        return ret;
    }

    k_video_frame_info tmp;
    ret = kd_mpi_dma_get_frame(s_dma_dev_chn, &tmp, -1);
    if (ret != K_SUCCESS) {
        printf("kd_mpi_dma_get_frame failed\n");
        return ret;
    }

    uint32_t size = vf_info_disp->v_frame.stride[0] * vf_info_disp->v_frame.height;
    void *tmp_addr = kd_mpi_sys_mmap_cached(tmp.v_frame.phys_addr[0], size);
    ret = kd_mpi_sys_mmz_flush_cache(tmp.v_frame.phys_addr[0], tmp_addr, size);
    if (ret != K_SUCCESS) {
        printf("kd_mpi_sys_mmz_flush_cache failed\n");
        kd_mpi_dma_release_frame(s_dma_dev_chn, &tmp);
        return ret;
    }

    rt_memcpy((void *)vf_info_disp->v_frame.virt_addr[0], tmp_addr, size);

    kd_mpi_sys_munmap(tmp_addr, size);
    return kd_mpi_dma_release_frame(s_dma_dev_chn, &tmp);
}

static void flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map) {
    if(!lv_display_flush_is_last(disp)) return;

    lv_memcpy((void *)s_vb_blk_info_rotation.virt_addr, px_map, s_vb_blk_info_rotation.blk_size);
    vo_osd_rotation(&s_vf_info_rotation, &s_vf_info_disp, s_dma_dev_chn);

    kd_mpi_vo_chn_insert_frame(s_vo_osd_layer + K_MAX_VO_LAYER_NUM, &s_vf_info_disp);

    lv_display_flush_ready(disp);
}

static void flush_wait_cb(lv_display_t * disp) {

}

static k_pixel_format map_lv_color_format(lv_color_format_t color_format) {
    switch (color_format)
    {
        case LV_COLOR_FORMAT_RGB565:
            return PIXEL_FORMAT_RGB_565;
        case LV_COLOR_FORMAT_RGB888:
            return PIXEL_FORMAT_RGB_888;
        case LV_COLOR_FORMAT_ARGB8888:
            return PIXEL_FORMAT_ARGB_8888;
        default:
            break;
    }

    return LV_COLOR_FORMAT_UNKNOWN;
}

static rt_err_t get_vb_blk_info(struct terminal_vb_blk_info *info, k_u64 blk_size) {

    k_vb_blk_handle handle = kd_mpi_vb_get_block(VB_INVALID_POOLID, blk_size, NULL);
    if (handle == VB_INVALID_HANDLE) {
        printf("kd_mpi_vb_get_block failed\n");
        kd_mpi_vb_release_block(handle);
        return RT_ERROR;
    }

    k_s32 pool_id = kd_mpi_vb_handle_to_pool_id(handle);
    if (pool_id == VB_INVALID_POOLID) {
        printf("kd_mpi_vb_handle_to_pool_id failed\n");
        kd_mpi_vb_release_block(handle);
        return RT_ERROR;
    }

    k_u64 phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);
    if ((void *)phys_addr == NULL) {
        printf("kd_mpi_vb_handle_to_phyaddr faile\n");
        kd_mpi_vb_release_block(handle);
        return RT_ERROR;
    }

    void* virt_addr = kd_mpi_sys_mmap(phys_addr, blk_size);
    if (virt_addr == NULL) {
        printf("kd_mpi_sys_mmap faile\n");
        kd_mpi_vb_release_block(handle);
        return RT_ERROR;
    }

    info->pool_id = pool_id;
    info->blk_size = blk_size;
    info->phys_addr = phys_addr;
    info->virt_addr = (k_u64)virt_addr;

    return RT_EOK;
}

static k_s32 vb_init(k_u64 buffer_size) {
    k_s32 ret = 0;

    // vb pool
    kd_mpi_vb_exit();
    k_vb_config config;
    k_vb_pool_config pool_config;
    lv_memset(&config, 0, sizeof(config));
    config.max_pool_cnt = 1;
    config.comm_pool[0].blk_cnt = 3;
    config.comm_pool[0].blk_size = buffer_size;
    config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;

    ret = kd_mpi_vb_set_config(&config);
    if (ret) {
        printf("kd_mpi_vb_set_config failed: %d\n", ret);
        return ret;
    }
    ret = kd_mpi_vb_init();
    if (ret) {
        printf("kd_mpi_vb_init failed: %d\n", ret);
        return ret;
    }

    get_vb_blk_info(&s_vb_blk_info_rotation, buffer_size);
    get_vb_blk_info(&s_vb_blk_info_disp, buffer_size);

    return ret;
}

static k_s32 vo_init(k_vo_osd vo_osd_layer, k_u32 x, k_u32 y, k_u32 width, k_u32 height, lv_color_format_t color_format) {

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

    printf("rotated_x = %d, rotated_y = %d rotated_width = %d, rotated_height = %d\n", rotated_x, rotated_y, rotated_width, rotated_height);

    k_pixel_format pixel_format = map_lv_color_format(color_format);

    s_vf_info_rotation.pool_id = s_vb_blk_info_rotation.pool_id;
    s_vf_info_rotation.mod_id = K_ID_DMA;
    s_vf_info_rotation.v_frame.width = width;
    s_vf_info_rotation.v_frame.height = height;
    s_vf_info_rotation.v_frame.pixel_format = pixel_format;
    s_vf_info_rotation.v_frame.stride[0] = lv_draw_buf_width_to_stride(width, color_format);
    s_vf_info_rotation.v_frame.phys_addr[0] = s_vb_blk_info_rotation.phys_addr;
    s_vf_info_rotation.v_frame.virt_addr[0] = s_vb_blk_info_rotation.virt_addr;

    s_vf_info_disp.pool_id = s_vb_blk_info_disp.pool_id;
    s_vf_info_disp.mod_id = K_ID_VO;
    s_vf_info_disp.v_frame.width = rotated_width;
    s_vf_info_disp.v_frame.height = rotated_height;
    s_vf_info_disp.v_frame.pixel_format = pixel_format;
    k_u32 rotated_stride = lv_draw_buf_width_to_stride(rotated_width, color_format);
    s_vf_info_disp.v_frame.stride[0] = rotated_stride;
    s_vf_info_disp.v_frame.phys_addr[0] = s_vb_blk_info_disp.phys_addr;
    s_vf_info_disp.v_frame.virt_addr[0] = s_vb_blk_info_disp.virt_addr;

    k_vo_video_osd_attr attr = {
        .display_rect = {rotated_x, rotated_y},
        .img_size = {rotated_width, rotated_height},
        .pixel_format = pixel_format,
        .stride = rotated_stride / 8,
        .global_alptha = 0xff,
    };

    k_s32 ret = kd_mpi_vo_set_video_osd_attr(vo_osd_layer, &attr);
    if (ret) {
        printf("kd_mpi_vo_set_video_osd_attr failed: %d\n", ret);
        return ret;
    }
    ret = kd_mpi_vo_osd_enable(vo_osd_layer);
    if (ret) {
        printf("kd_mpi_vo_osd_enable failed: %d\n", ret);
        return ret;
    }
    ret = kd_mpi_vo_enable();
    if (ret) {
        printf("kd_mpi_vo_enable failed: %d\n", ret);
        return ret;
    }

    s_vo_osd_layer = vo_osd_layer;

    return ret;
}

static k_s32 dma_dev_init(void)
{
    k_dma_dev_attr_t dev_attr;
    dev_attr.burst_len = 0;
    dev_attr.ckg_bypass = K_TRUE;
    dev_attr.outstanding = 7;

    k_s32 ret = kd_mpi_dma_set_dev_attr(&dev_attr);
    if (ret != K_SUCCESS) {
        printf("set dev attr error\n");
        return ret;
    }

    ret = kd_mpi_dma_start_dev();
    if (ret != K_SUCCESS) {
        printf("start dev error\n");
        return ret;
    }

    s_dma_dev_chn = kd_mpi_dma_request_chn(GDMA_TYPE);
    if(s_dma_dev_chn < 0) {
        printf("request gdma chn failed.\n");
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

    printf("gdma_attr rotation = %d width = %d, height = %d, src_stride = %d, dst_stride = %d, pixel_format = %d\n", 
        chn_attr.gdma_attr.rotation, chn_attr.gdma_attr.width, chn_attr.gdma_attr.height, chn_attr.gdma_attr.src_stride[0], chn_attr.gdma_attr.dst_stride[0], chn_attr.gdma_attr.pixel_format);

    ret = kd_mpi_dma_set_chn_attr(s_dma_dev_chn, &chn_attr);
    if (ret != K_SUCCESS) {
        printf("kd_mpi_dma_set_chn_attr failed");
        return ret;
    }

    ret = kd_mpi_dma_start_chn(s_dma_dev_chn);
    if (ret != K_SUCCESS) {
        return ret;
    }

}

RT_WEAK void lv_port_disp_init() {
    k_s32 ret = 0;

    ret = display_connector_init();
    if (ret) {
        printf("display_connector_init failed: %d\n", ret);
        return;
    }

    k_u32 resolution_hdisplay = s_connector_info.resolution.hdisplay;
    k_u32 resolution_vdisplay = s_connector_info.resolution.vdisplay;

    s_rotation_flag = K_ROTATION_270;
    s_mirror_mode_flag = K_VO_MIRROR_NONE;

    k_u32 display_width;
    k_u32 display_height;
    switch (s_rotation_flag) {
        case K_ROTATION_0:
        case K_ROTATION_180:
            display_width = resolution_hdisplay;
            display_height = resolution_vdisplay;
            break;
        case K_ROTATION_90:
        case K_ROTATION_270:
            display_width = resolution_vdisplay;
            display_height = resolution_hdisplay;
            break;
        default:
            RT_ASSERT("invalid arg!");
            break;
    }

    printf("display_width: %d display_height: %d\n", display_width, display_height);
    lv_disp_t *disp = lv_display_create(display_width, display_height);
    lv_display_set_flush_cb(disp, flush);
    lv_display_set_flush_wait_cb(disp, flush_wait_cb);
    lv_display_set_dpi(disp, 400);
    lv_color_format_t color_format = lv_display_get_color_format(disp);
    uint32_t stride = lv_draw_buf_width_to_stride(display_width, color_format);
    printf("display stride: %d\n", stride);
    size_t buffer_size = stride * display_height;
    printf("buffer_size: %ld\n", buffer_size);
    void* draw_buffer = lv_malloc(buffer_size);
    lv_display_set_buffers(disp, draw_buffer, NULL, buffer_size, LV_DISPLAY_RENDER_MODE_DIRECT);

    ret = vb_init(buffer_size);
    if (ret) {
        printf("vb_init failed: %d\n", ret);
        return;
    }

    ret = vo_init(K_VO_OSD1, 0, 0, display_width, display_height, color_format);
    if (ret) {
        printf("vo_init failed: %d\n", ret);
        return;
    }

    ret = dma_dev_init();
    if (ret) {
        printf("dma_dev_init failed: %d\n", ret);
        return;
    }
}