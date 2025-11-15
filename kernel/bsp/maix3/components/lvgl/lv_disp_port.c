#include <stdio.h>

#define DBG_TAG    "LVGL_PORT"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

#include "lvgl.h"

#include "display_manager.h"

static void flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map) {
    if(!lv_display_flush_is_last(disp)) return;

    display_manager_area_t dm_area = {
        .x1 = area->x1,
        .x2 = area->x2,
        .y1 = area->y1,
        .y2 = area->y2
    };

    display_manager_screen_flush(&dm_area, px_map);

    lv_display_flush_ready(disp);
}

static void flush_wait_cb(lv_display_t * disp) {

}

RT_WEAK void lv_port_disp_init() {
    uint32_t display_width = display_manager_get_screen_width();
    uint32_t display_height = display_manager_get_screen_height();

    lv_disp_t *disp = lv_display_create(display_width, display_height);
    lv_display_set_flush_cb(disp, flush);
    lv_display_set_flush_wait_cb(disp, flush_wait_cb);
    // lv_display_set_dpi(disp, 400);
    lv_display_set_dpi(disp, 300);
    lv_color_format_t color_format = lv_display_get_color_format(disp);
    uint32_t stride = lv_draw_buf_width_to_stride(display_width, color_format);
    size_t buffer_size = stride * display_height;
    void* draw_buffer = lv_malloc(buffer_size);
    lv_display_set_buffers(disp, draw_buffer, NULL, buffer_size, LV_DISPLAY_RENDER_MODE_DIRECT);
}