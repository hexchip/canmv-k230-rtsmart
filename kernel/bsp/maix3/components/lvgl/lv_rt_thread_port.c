/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: MIT
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-18     Meco Man     the first version
 * 2022-05-10     Meco Man     improve rt-thread initialization process
 */

#ifdef __RTTHREAD__

#include <lvgl.h>
#include <rtthread.h>
#ifdef PKG_USING_CPU_USAGE
#include "cpu_usage.h"
#endif /* PKG_USING_CPU_USAGE */

#define DBG_TAG    "LVGL"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

#include "display_manager.h"

#ifndef PKG_LVGL_THREAD_STACK_SIZE
    #define PKG_LVGL_THREAD_STACK_SIZE 4096
#endif /* PKG_LVGL_THREAD_STACK_SIZE */

#ifndef PKG_LVGL_THREAD_PRIO
    #define PKG_LVGL_THREAD_PRIO (RT_THREAD_PRIORITY_MAX*2/3)
#endif /* PKG_LVGL_THREAD_PRIO */

extern void lv_port_disp_init(void);
extern void lv_port_indev_init(void);
extern void lv_user_gui_init(void);

static struct rt_thread lvgl_thread;
static rt_mutex_t s_display_mutex;

#ifdef rt_align
    rt_align(RT_ALIGN_SIZE)
#else
    ALIGN(RT_ALIGN_SIZE)
#endif
static rt_uint8_t lvgl_thread_stack[PKG_LVGL_THREAD_STACK_SIZE];

#if LV_USE_LOG
static void lv_rt_log(lv_log_level_t level, const char * buf)
{
    (void) level;
    LOG_I(buf);
}
#endif /* LV_USE_LOG */

#ifdef PKG_USING_CPU_USAGE
uint32_t lv_timer_os_get_idle(void)
{
    return (100 - (uint32_t)cpu_load_average());
}
#endif /* PKG_USING_CPU_USAGE */

static void lvgl_thread_entry(void *parameter)
{
#if LV_USE_LOG
    #if LV_LOG_PRINTF==0
        lv_log_register_print_cb(lv_rt_log);
    #endif
#endif /* LV_USE_LOG */
    lv_init();
    lv_tick_set_cb(rt_tick_get_millisecond);
    lv_port_disp_init();
    lv_port_indev_init();
    lv_user_gui_init();

#ifdef PKG_USING_CPU_USAGE
    cpu_usage_init();
#endif /* PKG_USING_CPU_USAGE */

    /* handle the tasks of LVGL */
    while(1)
    {
        rt_mutex_take(s_display_mutex, RT_WAITING_FOREVER);
        uint32_t time_until_next = lv_timer_handler();
        rt_mutex_release(s_display_mutex);
        if (time_until_next == LV_NO_TIMER_READY) {
            rt_thread_mdelay(33);
        }
        else {
            rt_thread_mdelay(time_until_next);
        }
    }
}

static void on_user_access_display() {
    rt_mutex_take(s_display_mutex, RT_WAITING_FOREVER);
}

static void on_user_leave_display() {
    rt_mutex_release(s_display_mutex);
}

display_manager_listener_t display_manager_listener = {
    .on_user_access = on_user_access_display,
    .on_user_leave = on_user_leave_display
};

static int lvgl_thread_init(void) {
    rt_err_t err;

    s_display_mutex = rt_mutex_create("display", RT_IPC_FLAG_PRIO);

    err = display_manager_register_listener(&display_manager_listener);
    if (err != RT_EOK) {
        LOG_E("Failed to register display_manager listener");
        return -1;
    }

    err = rt_thread_init(&lvgl_thread, "LVGL", lvgl_thread_entry, RT_NULL,
           &lvgl_thread_stack[0], sizeof(lvgl_thread_stack), PKG_LVGL_THREAD_PRIO, 10);
    if(err != RT_EOK) {
        LOG_E("Failed to create LVGL thread");
        return -1;
    }
    rt_thread_startup(&lvgl_thread);

    return 0;
}
INIT_APP_EXPORT(lvgl_thread_init);

RT_WEAK void lv_user_gui_init(void) {
    
}


#endif /*__RTTHREAD__*/
