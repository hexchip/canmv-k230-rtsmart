#include "rt_device_wrap.h"
#include "rt_device_wrap_internal.h"

#include "console.h"

void rt_device_wrap_printf(const char* format, ...){
    va_list args;
    rt_size_t length = 0;
    static char log_buf[RT_CONSOLEBUF_SIZE];

    va_start(args, format);
    length = rt_vsnprintf(log_buf, sizeof(log_buf) - 1, format, args);
    if (length > RT_CONSOLEBUF_SIZE - 1)
    {
        length = RT_CONSOLEBUF_SIZE - 1;
    }
    va_end(args);

    
    rt_device_wrap_t *io_device_wrap = RT_NULL;
    rt_device_wrap_list_node_t *node = RT_NULL;
    rt_device_t io_device = console_get_iodev();
    rt_list_for_each_entry(node, get_device_wrap_list(), list) {
        if (&node->device_wrap->parent == io_device) {
            io_device_wrap = node->device_wrap;
            break;
        }
    }
    rt_bool_t is_hacked = io_device->ops == get_device_hack_ops();

    if (is_hacked) {
        if (io_device_wrap == RT_NULL || io_device_wrap->device_ops == RT_NULL) {
            // impossible! otherwise, it would be a fatal error.
            return;
        }
        io_device_wrap->device_ops->write(io_device_wrap->device, 0, log_buf, length);
    }
    else {
        rt_device_write(io_device, 0, log_buf, length);
    }
}