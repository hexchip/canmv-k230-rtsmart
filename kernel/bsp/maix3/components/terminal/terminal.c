#include "terminal.h"

#include <stdio.h>

#include "tmt.h"
#include "rt_device_wrap.h"
#include "console.h"

typedef struct terminal_context {
    TMT *vt;
    char line_buffer[256];
    size_t line_buffer_size;
    rt_device_t iodev;
    rt_tick_t last_output_time_ms;
    rt_device_wrap_t *iodev_wrap;
    rt_device_wrap_listener_t iodev_listener;
    rt_mq_t mq;
    rt_thread_t thread;
} terminal_context_t;

struct mq_msg {
    char short_str[TERMINAL_MQ_MSG_SHORT_STR_LEN * 2];
    char *str;
    rt_size_t size;
};

static rt_size_t io_out_memcpy(char *dest, const char *src, rt_size_t size) {
    rt_size_t residual_size = size;
    rt_size_t total_size = 0;
    while (residual_size)
    {
        if (*src == '\n')
        {
            *dest = '\r';
            ++ dest;
            ++ total_size;
        }

        *dest = *src;
        ++ dest;
        ++ src;
        -- residual_size;
        ++ total_size;
    }

    return total_size;
}

static void on_iodev_open(rt_device_t dev, rt_uint16_t oflag, void *user_data) {
    if(dev->ref_count == 0) {
        terminal_init((terminal_t *)user_data);
    }
}

static void on_iodev_close(rt_device_t dev, void *user_data) {
    if (dev->ref_count == 1) {
        terminal_deinit((terminal_t *)user_data);
    }
}

static void on_iodev_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size, void *user_data) {
    // Do not print!!! It will cause an infinite loop.
    
    struct mq_msg msg;
    rt_memset(&msg, 0, sizeof(struct mq_msg));
    if (size > TERMINAL_MQ_MSG_SHORT_STR_LEN) {
        msg.str = rt_malloc(size * 2);
        rt_memset(msg.str, 0, size * 2);
        RT_ASSERT(msg.str);
        rt_size_t real_size = io_out_memcpy(msg.str, buffer, size);
        msg.str = rt_realloc(msg.str, real_size);
        RT_ASSERT(msg.str);
        msg.size = real_size;
    }
    else {
        msg.size = io_out_memcpy(msg.short_str, buffer, size);
    }

    terminal_t *terminal = user_data;
    terminal->context->last_output_time_ms = rt_tick_get_millisecond();
    rt_err_t ret = rt_mq_send_wait(terminal->context->mq, &msg, sizeof(struct mq_msg), rt_tick_from_millisecond(1000));

    if (ret != RT_EOK) {
        RT_ASSERT("on_iodev_write: rt_mq_send error!")
    }
}

static void tmt_callback(tmt_msg_t m, TMT *vt, const void *a, void *context) {
    terminal_t *terminal = context;
    switch (m){
        case TMT_MSG_BELL:
            /* the terminal is requesting that we ring the bell/flash the
             * screen/do whatever ^G is supposed to do; a is NULL
             */
            // TODO At present, we have no sound output.
            break;

        case TMT_MSG_UPDATE:
            const TMTSCREEN *screen = a;
            char *buffer = terminal->context->line_buffer;
            size_t buffer_size = terminal->context->line_buffer_size;
            size_t dirty_line_index_array[256];
            size_t dirty_line_count = 0;
            for (size_t r = 0; r < screen->nline; r++) {
                if (screen->lines[r]->dirty) {
                    dirty_line_index_array[dirty_line_count] = r;
                    dirty_line_count++;
                    for (size_t col = 0; col < screen->ncol; col++) {
                          // Convert wide characters to multibyte characters.
                          wchar_t wc = screen->lines[r]->chars[col].c;
                          if (wc == L'\0' || wc == TMT_INVALID_CHAR) {
                              buffer[col] = ' ';
                          } else {
                              // Simplified processing: Assume it is ASCII characters.
                              buffer[col] = (char)wc;
                          }
                    }
                    buffer[screen->ncol] = '\0';
                    terminal->lifecycle->on_line_update(r, buffer, buffer_size);
                }
            }
            
            if (dirty_line_count > 0) {
                terminal->lifecycle->on_refresh_dirty_lines(dirty_line_index_array, dirty_line_count);
            }

            // Mark all dirty line as cleared.
            tmt_clean(vt);
            break;

        case TMT_MSG_ANSWER:
            /* the terminal has a response to give to the program; a is a
             * pointer to a string */
            // TODO what this is used for?
            break;

        case TMT_MSG_MOVED:
            /* the cursor moved; a is a pointer to the cursor's TMTPOINT */
            const TMTPOINT *point = a;
            terminal->lifecycle->on_cursor_move(point->r, point->c);
            break;
    }
}

static void terminal_thread_entry(void *parameter) {
    terminal_context_t *context = parameter;
    rt_mq_t mq = context->mq;
    TMT *vt = context->vt;

    struct mq_msg msg;
    while (1) {
        if (rt_mq_recv(mq, &msg, sizeof(struct mq_msg), RT_WAITING_FOREVER) == RT_EOK) {
            if (msg.str) {
                tmt_write(vt, msg.str, msg.size);
                rt_free(msg.str);
            }
            else {
                tmt_write(vt, msg.short_str, msg.size);
            }
        }
    }
}

terminal_t * terminal_create(char *name, 
                                size_t rows, size_t cols, 
                                terminal_lifecycle_t *lifecycle, 
                                size_t thread_stack_size, int thread_priority) {
    RT_ASSERT(name != NULL);
    RT_ASSERT(rows > 0 && cols > 0);
    RT_ASSERT(lifecycle != NULL);
    
    terminal_t *terminal = rt_malloc(sizeof(terminal_t));

    if (terminal == NULL) {
        return NULL;
    }

    terminal->context = NULL;

    rt_size_t name_buffer_size = sizeof(char) * (rt_strlen(name) + 1);
    char *terminal_name = rt_malloc(name_buffer_size);

    if (terminal_name == NULL) {
        rt_free(terminal);
        return NULL;
    }

    rt_memcpy(terminal_name, name, name_buffer_size);
    terminal->name = terminal_name;
    terminal->rows = rows;
    terminal->cols = cols;
    terminal->lifecycle = lifecycle;
    terminal->thread_stack_size = thread_stack_size;
    terminal->thread_priority = thread_priority;

    return terminal;
}

int terminal_init(terminal_t *terminal) {
    if (terminal->context != NULL) {
        return 0;
    }

    terminal->context = rt_malloc(sizeof(terminal_context_t));

    if (terminal->context == NULL) {
        return -ENOMEM;
    }

    rt_memset(terminal->context, 0, sizeof(struct terminal_context));

    terminal->context->line_buffer_size = sizeof(char) * (terminal->cols + 1);
    terminal->context->vt = tmt_open(terminal->rows, terminal->cols, tmt_callback, terminal, NULL);

    if (terminal->context->vt == NULL) {
        printf("Failed to initialize libtmt terminal.\n");
        terminal_deinit(terminal);
        return -ENOMEM;
    }

    rt_device_t iodev = console_get_iodev();
    terminal->context->iodev = iodev;

    rt_device_wrap_t *iodev_wrap = rt_device_wrap_create(iodev);
    if (iodev_wrap == NULL) {
        printf("Failed to create iodev wrap.\n");
        terminal_deinit(terminal);
        return -ENOMEM;
    }

    terminal->context->iodev_wrap = iodev_wrap;
    terminal->context->mq = rt_mq_create("iodev-out", sizeof(struct mq_msg), TERMINAL_MQ_MSG_MAX_NUM, RT_IPC_FLAG_PRIO);

    if (terminal->context->mq == NULL) {
        printf("Failed to create iodev-out mq.\n");
        terminal_deinit(terminal);
        return -ENOMEM;
    }

    rt_device_wrap_set_user_data(iodev_wrap, terminal);

    rt_err_t ret = rt_device_wrap_register(iodev_wrap, "iodev_wrap");
    if (ret != RT_EOK) {
        printf("device_wrap_register(%s) failed: %d\n", "iodev_wrap", ret);
        terminal_deinit(terminal);
    }
    console_set_iodev(rt_device_wrap_get_parent(iodev_wrap));

    terminal->context->iodev_listener.on_open = on_iodev_open;
    terminal->context->iodev_listener.on_write = on_iodev_write;
    terminal->context->iodev_listener.on_close = on_iodev_close;
    rt_device_wrap_register_listener(iodev_wrap, &terminal->context->iodev_listener);

    rt_thread_t terminal_thread = rt_thread_create(
        terminal->name, 
        terminal_thread_entry, 
        terminal->context, 
        terminal->thread_stack_size, 
        terminal->thread_priority, 10);

    if (terminal_thread == NULL) {
        printf("Failed to create terminal thread thread\n");
        terminal_deinit(terminal);
        return -ENOMEM;
    }

    terminal->context->thread = terminal_thread;

    rt_thread_startup(terminal_thread);

    return 0;
}

void terminal_deinit(terminal_t *terminal) {
    if (terminal->context == NULL) {
        return;
    }

    if (terminal->context->thread) {
        rt_thread_delete(terminal->context->thread);
    }

    if (terminal->context->iodev_wrap) {
        rt_device_wrap_unregister_listener(terminal->context->iodev_wrap, &terminal->context->iodev_listener);

        if (terminal->context->iodev) {
            console_set_iodev(terminal->context->iodev);
        }
        if (terminal->context->mq) {
            rt_mq_delete(terminal->context->mq);
        }
        
        rt_device_wrap_destroy(terminal->context->iodev_wrap);
    }

    if (terminal->context->vt) {
        tmt_close(terminal->context->vt);
    }

    free(terminal->context);

    terminal->context = NULL;
}

void terminal_destroy(terminal_t *terminal) {
    if (terminal == NULL) {
        return;
    }

    terminal_deinit(terminal);
    if (terminal->name != NULL) {
        rt_free(terminal->name);
    }
    rt_free(terminal);
}

void terminal_debug_printf(terminal_t *terminal, const char * format, ...) {
    rt_device_t dev = terminal->context->iodev;
    if (dev == NULL) {
        return;
    }

    static const rt_tick_t MIN_OUTPUT_INTERVAL_MS = 500;

    char msg[256];
    va_list args;
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args);
    va_end(args);
    if (rt_thread_self()) {
        rt_tick_t elapsed_time_ms = rt_tick_get_millisecond() - terminal->context->last_output_time_ms;
        if (elapsed_time_ms < MIN_OUTPUT_INTERVAL_MS) {
            rt_thread_mdelay(MIN_OUTPUT_INTERVAL_MS - elapsed_time_ms);
        }
    }
    rt_device_write(dev, 0, msg, strlen(msg));
}