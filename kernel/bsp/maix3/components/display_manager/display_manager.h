#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rtdef.h"
#include "k_video_comm.h"
#include "k_vo_comm.h"

typedef struct display_manager_listener {
    void (*on_user_access)();
    void (*on_user_leave)();
} display_manager_listener_t;

/** Represents an area of the screen.*/
typedef struct {
    int32_t x1;
    int32_t y1;
    int32_t x2;
    int32_t y2;
} display_manager_area_t;

rt_err_t display_manager_register_listener(display_manager_listener_t *listener);

rt_err_t display_manager_unregister_listener(display_manager_listener_t *listener);

uint32_t display_manager_get_screen_width();

uint32_t display_manager_get_screen_height();

int display_manager_screen_flush(const display_manager_area_t *area, uint8_t *px_map);

int display_manager_screen_clean();

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif // DISPLAY_MANAGER_H
