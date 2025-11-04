#include "display_manager.h"

#include <stdbool.h>

/* 使用 IPC 服务，需要包含 lwp.h 头文件 */
#include <lwp.h>

/* 包含 rtthread.h，可以使用原来的 RT-Thread API */
#include <rtthread.h>

#define DBG_TAG    "DISP_MGR"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

#include "internal/display_hal.h"

typedef enum display_manager_cmd {
    DISPLAY_MANAGER_UNKNOWN,
    DISPLAY_MANAGER_GET_SCREEN_INFO,
    DISPLAY_MANAGER_ACCESS_DISPLAY,
    DISPLAY_MANAGER_LEAVE_DISPLAY,
    DISPLAY_MANAGER_FLUSH_SCREEN,
} display_manager_cmd_t;

typedef struct display_manager_screen_flush_cmd_arg {
    pid_t pid;
    display_manager_area_t area;
    int px_map_shmid;
} display_manager_screen_flush_cmd_arg_t;

typedef struct display_manager_screen_info {
    uint32_t width;
    uint32_t height;
    uint32_t dpi;
}display_manager_screen_info_t;

typedef struct display_manager_msg {
    display_manager_cmd_t cmd;
    union {
        pid_t pid;
        display_manager_screen_flush_cmd_arg_t screen_flush_arg;
    } arg;
    union {
        display_manager_screen_info_t screen_info;
    } ret;
    int error;
} display_manager_msg_t;


typedef struct display_manager_listener_entry {
    rt_list_t list;
    display_manager_listener_t *listener;
} display_manager_listener_entry_t;

static rt_thread_t ipc_channel_thread = NULL;

static rt_list_t listeners;

static rt_bool_t is_user_access = false;
int current_user_pid = 0;

static inline int handle_get_screen_info_cmd(display_manager_msg_t *msg) {
    LOG_I("handle_get_screen_info_cmd");
    display_manager_screen_info_t *info = &msg->ret.screen_info;
    info->width = display_hal_get_width();
    info->height = display_hal_get_height();
    info->dpi = 400;

    return 0;
}

static inline bool is_alive_process(pid_t pid) {
    struct rt_lwp *lwp = lwp_from_pid(pid);
    return lwp == NULL ? false : true;
}

static inline void access_display(pid_t pid) {
    is_user_access = true;
    current_user_pid = pid;

    display_manager_listener_entry_t *entry;
    rt_list_for_each_entry(entry, &listeners, list) {
        if (entry->listener->on_user_access) {
            entry->listener->on_user_access();
        }
    }
}

static inline int handle_access_dispaly_cmd(display_manager_msg_t *msg) {
    pid_t pid = msg->arg.pid;
    struct rt_lwp *lwp = lwp_from_pid(pid);

    if (lwp == NULL) {
        return -EINVAL;
    }

    if (is_user_access) {
        if (is_alive_process(current_user_pid)) {
            if (current_user_pid == lwp->pid) {
                return 0;
            }
            else {
                return -EBUSY;
            }
        }
    }

    access_display(pid);

    return 0;
}

static inline void leave_display() {
    is_user_access = false;
    current_user_pid = 0;

    display_manager_listener_entry_t *entry;
    rt_list_for_each_entry(entry, &listeners, list) {
        if (entry->listener->on_user_leave) {
            entry->listener->on_user_leave();
        }
    }
}

static inline int handle_leave_dispaly_cmd(display_manager_msg_t *msg) {
    if (!is_user_access) {
        return 0;
    }

    if (is_alive_process(current_user_pid)) {
        pid_t pid = msg->arg.pid;
        struct rt_lwp *lwp = lwp_from_pid(pid);

        if (lwp == NULL) {
            return -EINVAL;
        }

        if (current_user_pid != lwp->pid) {
            return -EBUSY;
        }
    }

    leave_display();

    return 0;
}

static inline int handle_flush_screen_cmd(display_manager_msg_t *msg) {
    if (!is_user_access) {
        return -EPERM;
    }

    display_manager_screen_flush_cmd_arg_t *arg = &msg->arg.screen_flush_arg;
    if (arg->pid != current_user_pid) {
        return -EPERM;
    }
    
    k_u8 *px_map = lwp_shminfo(arg->px_map_shmid);
    return display_hal_flush_frame(px_map);
}

static int handle_msg(display_manager_msg_t *msg) {
    int ret = -EINVAL;

    switch (msg->cmd) {
        case DISPLAY_MANAGER_GET_SCREEN_INFO:
            ret = handle_get_screen_info_cmd(msg);
            break;
        case DISPLAY_MANAGER_ACCESS_DISPLAY:
            ret = handle_access_dispaly_cmd(msg);
            break;
        case DISPLAY_MANAGER_LEAVE_DISPLAY:
            ret = handle_leave_dispaly_cmd(msg);
            break;
        case DISPLAY_MANAGER_FLUSH_SCREEN:
            ret = handle_flush_screen_cmd(msg);
            break;
        default:
            LOG_E("invalid cmd! cmd=%d", msg->cmd);
            break;
    }

    return ret;
}

static void display_manager_thread_entry(void *context) {
    int channel = rt_channel_open("display_manager", O_CREAT);
    if (channel == -1) {
        LOG_E("Fail to create the IPC channel for display_manager!");
        return;
    }

    struct rt_channel_msg channel_msg;

    while (1) {
        rt_err_t error = rt_channel_recv_timeout(channel, &channel_msg, rt_tick_from_millisecond(1000));
        if(error == RT_EOK) {
            int shmid = (int)(intptr_t)channel_msg.u.d;
            display_manager_msg_t *msg = lwp_shminfo(shmid);
            if (shmid < 0 || msg == NULL) {
                LOG_E("shmid < 0 || msg == NULL!");
                msg->error = -EINVAL;
            }
            else {
                msg->error = handle_msg(msg);
            }
            rt_channel_reply(channel, &channel_msg);
        }
        else if (error = -RT_ETIMEOUT) {
            // noop
        }
        else {
            LOG_E("display_manager rt_channel_recv failed!");
        }

        if (!is_alive_process(current_user_pid)) {
            leave_display();
        }
    }
    
    rt_channel_close(channel);
}

int display_manager_init() {
    rt_list_init(&listeners);

    int ret = display_hal_init();

    if (ret != 0) {
        LOG_E("Failed to init display hal");
        return -1;
    } 

    ipc_channel_thread = rt_thread_create("disp_mgr_ipc", display_manager_thread_entry, RT_NULL, 1024 * 8, 18, 10);
    if(ipc_channel_thread == RT_NULL) {
        LOG_E("Failed to create DISP_MGR thread");
        return -1;
    }

    rt_err_t rt_ret = rt_thread_startup(ipc_channel_thread);

    if (rt_ret != RT_EOK) {
        LOG_E("Failed to startup DISP_MGR thread");
        return -1;
    }

    return 0;
}
INIT_ENV_EXPORT(display_manager_init);

rt_err_t display_manager_register_listener(display_manager_listener_t *listener) {
    if (listener == RT_NULL) {
        return RT_EINVAL;
    }

    display_manager_listener_entry_t *entry = rt_malloc(sizeof(display_manager_listener_entry_t));

    if (entry == RT_NULL) {
        return RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->listener = listener;
    rt_list_insert_after(&listeners, &entry->list);

    return RT_EOK;
}

rt_err_t display_manager_unregister_listener(display_manager_listener_t *listener) {
    if (listener == RT_NULL) {
        return RT_EINVAL;
    }

    display_manager_listener_entry_t *entry;
    display_manager_listener_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &listeners, list) {
        if (entry->listener == listener) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}

uint32_t display_manager_get_screen_width() {
    return display_hal_get_width();
}

uint32_t display_manager_get_screen_height() {
    return display_hal_get_height();
}

int display_manager_screen_flush(const display_manager_area_t *area, uint8_t *px_map) {
    (void)area;
    
    return display_hal_flush_frame(px_map);
}