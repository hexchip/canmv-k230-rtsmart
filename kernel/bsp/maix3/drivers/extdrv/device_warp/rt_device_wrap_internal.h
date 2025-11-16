#ifndef RT_DEVICE_WRAP_INTERNAL_H
#define RT_DEVICE_WRAP_INTERNAL_H

#include <rtthread.h>
#include "rt_device_wrap_log.h"

struct rt_device_wrap {
    struct rt_device parent;
    rt_device_t device;

#ifdef RT_USING_DEVICE_OPS
    const struct rt_device_ops *device_ops;
#else
    rt_err_t  (*device_init)   (rt_device_t dev);
    rt_err_t  (*device_open)   (rt_device_t dev, rt_uint16_t oflag);
    rt_err_t  (*device_close)  (rt_device_t dev);
    rt_size_t (*device_read)   (rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
    rt_size_t (*device_write)  (rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
    rt_err_t  (*device_control)(rt_device_t dev, int cmd, void *args);
#endif

    rt_bool_t is_registered_device;
    rt_list_t listeners;
    rt_list_t interceptors;
#ifdef RT_USING_POSIX
    const struct dfs_file_ops *device_fops;
    int device_fd;
    rt_list_t fops_interceptors;
#endif
};

typedef struct rt_device_wrap_list_node {
    rt_device_wrap_t *device_wrap;
    rt_list_t list;
} rt_device_wrap_list_node_t;

typedef struct rt_device_wrap_listener_entry {
    rt_list_t list;
    rt_device_wrap_listener_t *listener;
} rt_device_wrap_listener_entry_t;

typedef struct rt_device_wrap_interceptor_entry {
    rt_list_t list;
    rt_device_wrap_interceptor_t *interceptor;
} rt_device_wrap_interceptor_entry_t;

typedef struct rt_device_wrap_fops_interceptor_entry {
    rt_list_t list;
    rt_device_wrap_fops_interceptor_t *interceptor;
} rt_device_wrap_fops_interceptor_entry_t;


rt_list_t * get_device_wrap_list();

rt_device_wrap_t * get_device_wrap(rt_device_t dev);

const struct rt_device_ops * get_device_hack_ops();

const struct rt_device_ops * get_device_wrap_ops();

#ifdef RT_USING_POSIX
const struct dfs_file_ops * get_device_hack_fops();

const struct dfs_file_ops * get_device_wrap_fops();
#endif // RT_USING_POSIX

#endif // RT_DEVICE_WRAP_INTERNAL_H