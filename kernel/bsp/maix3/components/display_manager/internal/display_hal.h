#ifndef DISPLAY_HAL_H
#define DISPLAY_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "k_video_comm.h"
#include "k_vo_comm.h"

typedef struct terminal_vb_blk_info {
    k_s32 pool_id;
    k_u64 blk_size;
    k_u64 phys_addr;
    k_u64 virt_addr;
} terminal_vb_blk_info_t;

int display_hal_init();

k_s32 display_hal_flush_frame(k_u8 *px_map);

k_u32 display_hal_get_width();

k_u32 display_hal_get_height();

k_vo_osd display_hal_get_osd_layer();

k_u8 display_hal_get_dma_channel();

k_video_frame_info * display_hal_get_disp_frame_info();

k_video_frame_info * display_hal_get_rotation_frame_info();

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif // DISPLAY_HAL_H
