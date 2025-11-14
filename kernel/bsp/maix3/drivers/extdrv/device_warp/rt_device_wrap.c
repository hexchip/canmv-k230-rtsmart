#include <rt_device_wrap.h>
#include <rtthread.h>

#ifdef RT_USING_POSIX
#include <dfs_posix.h>
#include <dfs_poll.h>
#endif // RT_USING_POSIX

#define DBG_SECTION_NAME    "DEV_WRAP"
#define DBG_LEVEL           DBG_ERROR
#include <rtdbg.h>

#ifdef RT_USING_DEVICE_OPS
#define device_init     (dev->ops->init)
#define device_open     (dev->ops->open)
#define device_close    (dev->ops->close)
#define device_read     (dev->ops->read)
#define device_write    (dev->ops->write)
#define device_control  (dev->ops->control)

#define device_hack_init     (device_warp->device_ops->init)
#define device_hack_open     (device_warp->device_ops->open)
#define device_hack_close    (device_warp->device_ops->close)
#define device_hack_read     (device_warp->device_ops->read)
#define device_hack_write    (device_warp->device_ops->write)
#define device_hack_control  (device_warp->device_ops->control)
#else
#define device_hack_init     (device_warp->device_init)
#define device_hack_open     (device_warp->device_open)
#define device_hack_close    (device_warp->device_close)
#define device_hack_read     (device_warp->device_read)
#define device_hack_write    (device_warp->device_write)
#define device_hack_control  (device_warp->device_control)

#define device_init     (dev->init)
#define device_open     (dev->open)
#define device_close    (dev->close)
#define device_read     (dev->read)
#define device_write    (dev->write)
#define device_control  (dev->control)
#endif

typedef struct rt_device_warp_listener_entry {
    rt_list_t list;
    rt_device_warp_listener_t *listener;
} rt_device_warp_listener_entry_t;

typedef struct rt_device_warp_interceptor_entry {
    rt_list_t list;
    rt_device_warp_interceptor_t *interceptor;
} rt_device_warp_interceptor_entry_t;

struct rt_device_warp {
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

#ifdef RT_USING_POSIX
    const struct dfs_file_ops *device_fops;
    int device_fd;
#endif
    rt_bool_t is_registered_device;
    rt_list_t listeners;
    rt_list_t interceptors;
};

typedef struct rt_device_warp_list_node {
    rt_device_warp_t *device_warp;
    rt_list_t list;
} rt_device_warp_list_node_t;

static rt_list_t s_device_warp_list = RT_LIST_OBJECT_INIT(s_device_warp_list);

static inline rt_device_warp_t * get_device_warp(rt_device_t dev) {
    rt_device_warp_t *device_warp = RT_NULL;
    rt_device_warp_list_node_t *node = RT_NULL;
    rt_list_for_each_entry(node, &s_device_warp_list, list) {
        if (node->device_warp->device == dev) {
            device_warp = node->device_warp;
            break;
        }
    }
    return device_warp;
}

static rt_err_t rt_device_hack_init(rt_device_t dev) {
    LOG_D("hack_init: enter");
    rt_device_warp_t *device_warp = get_device_warp(dev);

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_init) {
            is_intercepted = interceptor_entry->interceptor->on_init(dev, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_warp->listeners, list) {
        if (listener_entry->listener->on_init) {
            listener_entry->listener->on_init(dev, device_warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_D("hack_init: intercepted!");
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_hack_init(dev);
    }
    else {
        return device_init(dev);
    }
}

static rt_err_t rt_device_hack_open(rt_device_t dev, rt_uint16_t oflag) {
    LOG_D("hack_open: enter");
    rt_device_warp_t *device_warp = get_device_warp(dev);

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_open) {
            is_intercepted = interceptor_entry->interceptor->on_open(dev, &oflag, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_warp->listeners, list) {
        if (listener_entry->listener->on_open) {
            listener_entry->listener->on_open(dev, oflag, device_warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_D("hack_open: intercepted!");
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_hack_open(dev, oflag);
    }
    else {
        return device_open(dev, oflag);
    }
}

static rt_err_t rt_device_hack_close(rt_device_t dev) {
    LOG_D("hack_close: enter");
    rt_device_warp_t *device_warp = get_device_warp(dev);

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_close) {
            is_intercepted = interceptor_entry->interceptor->on_close(device_warp->device, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_warp->listeners, list) {
        if (listener_entry->listener->on_close) {
            listener_entry->listener->on_close(device_warp->device, device_warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_D("hack_close: intercepted!");
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_hack_close(dev);
    }
    else {
        return device_close(dev);
    }
}

static rt_size_t rt_device_hack_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size) {
    LOG_D("hack_read: enter");
    rt_device_warp_t *device_warp = get_device_warp(dev);

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_size_t read_size = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_read) {
            is_intercepted = interceptor_entry->interceptor->on_read(dev, &pos, &buffer, &size, &read_size, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_warp->listeners, list) {
        if (listener_entry->listener) {
            if (listener_entry->listener->on_read) {
                listener_entry->listener->on_read(dev, pos, buffer, size, device_warp->parent.user_data);
            }
        }
    }

    if (is_intercepted) {
        LOG_D("hack_read: intercepted!");
        return read_size;
    }

    if (device_warp->is_registered_device) {
        return device_hack_read(dev, pos, buffer, size);
    }
    else {
        return device_read(dev, pos, buffer, size);
    }
}

static rt_size_t rt_device_hack_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size) {
    LOG_D("hack_write: enter");
    rt_device_warp_t *device_warp = get_device_warp(dev);

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_size_t write_size = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_write) {
            is_intercepted = interceptor_entry->interceptor->on_write(dev, &pos, &buffer, &size, &write_size, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_warp->listeners, list) {
        if (listener_entry->listener) {
            if (listener_entry->listener->on_write) {
                listener_entry->listener->on_write(dev, pos, buffer, size, device_warp->parent.user_data);
            }
        }
    }

    if (is_intercepted) {
        LOG_D("hack_write: intercepted!");
        return write_size;
    }

    if (device_warp->is_registered_device) {
        return device_hack_write(dev, pos, buffer, size);
    }
    else {
        return device_write(dev, pos, buffer, size);
    }
}

static rt_err_t rt_device_hack_control(rt_device_t dev, int cmd, void *args) {
    LOG_D("hack_control: enter");
    rt_device_warp_t *device_warp = get_device_warp(dev);

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_control) {
            is_intercepted = interceptor_entry->interceptor->on_control(dev, &cmd, &args, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_warp->listeners, list) {
        if (listener_entry->listener->on_control) {
            listener_entry->listener->on_control(dev, cmd, args, device_warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_D("hack_control: intercepted!");
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_hack_control(dev, cmd, args);
    }
    else {
        return device_control(dev, cmd, args);
    }
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops device_hack_ops = {
    rt_device_hack_init,
    rt_device_hack_open,
    rt_device_hack_close,
    rt_device_hack_read,
    rt_device_hack_write,
    rt_device_hack_control
};
#endif // RT_USING_DEVICE_OPS

static rt_err_t device_warp_init(rt_device_t dev) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)dev;
    LOG_D("%s device_warp_init: enter", device_warp->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_warp->is_registered_device) {
        #ifdef RT_USING_DEVICE_OPS
        device_warp->device_ops = device_warp->device->ops;
        device_warp->device->ops = &device_hack_ops;
        ret = rt_device_init(device_warp->device);
        if(device_warp->device->ops == &device_hack_ops) {
            device_warp->device->ops = device_warp->device_ops;
        }
        #else
        device_warp->device_init = device_warp->device->init;
        device_warp->device->init = device_hack_init;
        ret = rt_device_init(device_warp->device);
        if (device_warp->device->init == device_hack_init) {
            device_warp->device->init = device_warp->device_init;
        }
        #endif // RT_USING_DEVICE_OPS
    }
    else {
        ret = rt_device_hack_init(device_warp->device);
    }

    return ret;
}

static rt_err_t device_warp_open(rt_device_t dev, rt_uint16_t oflag) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)dev;
    LOG_D("%s device_warp_open: enter", device_warp->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_warp->is_registered_device) {
        #ifdef RT_USING_DEVICE_OPS
        device_warp->device_ops = device_warp->device->ops;
        device_warp->device->ops = &device_hack_ops;
        ret = rt_device_open(device_warp->device, oflag);
        if(device_warp->device->ops == &device_hack_ops) {
            device_warp->device->ops = device_warp->device_ops;
        }
        #else
        device_warp->device_open = device_warp->device->open;
        device_warp->device->open = device_hack_open;
        ret = rt_device_open(device_warp->device, oflag);
        if (device_warp->device->open == device_hack_open) {
            device_warp->device->open = device_warp->device_open;
        }
        #endif // RT_USING_DEVICE_OPS
    }
    else {
        ret = rt_device_hack_open(device_warp->device, oflag);
    }

    return ret;
}

static rt_err_t device_warp_close(rt_device_t dev) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)dev;
    LOG_D("%s device_warp_close: enter", device_warp->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_warp->is_registered_device) {
        #ifdef RT_USING_DEVICE_OPS
        device_warp->device_ops = device_warp->device->ops;
        device_warp->device->ops = &device_hack_ops;
        ret = rt_device_close(device_warp->device);
        if(device_warp->device->ops == &device_hack_ops) {
            device_warp->device->ops = device_warp->device_ops;
        }
        #else
        device_warp->device_close = device_warp->device->close;
        device_warp->device->close = device_hack_close;
        ret = rt_device_close(device_warp->device);
        if (device_warp->device->close == device_hack_close) {
            device_warp->device->close = device_warp->device_close;
        }
        #endif // RT_USING_DEVICE_OPS
    }
    else {
        ret = rt_device_hack_close(device_warp->device);
    }

    return ret;
}

static rt_size_t device_warp_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)dev;
    LOG_D("%s device_warp_read: enter", device_warp->parent.parent.name);

    rt_size_t read_size = RT_EOK;

    if (device_warp->is_registered_device) {
        #ifdef RT_USING_DEVICE_OPS
        device_warp->device_ops = device_warp->device->ops;
        device_warp->device->ops = &device_hack_ops;
        read_size = rt_device_read(device_warp->device, pos, buffer, size);
        if(device_warp->device->ops == &device_hack_ops) {
            device_warp->device->ops = device_warp->device_ops;
        }
        #else
        device_warp->device_read = device_warp->device->read;
        device_warp->device->read = device_hack_read;
        read_size = rt_device_read(device_warp->device, pos, buffer, size);
        if (device_warp->device->read == device_hack_read) {
            device_warp->device->read = device_warp->device_read;
        }
        #endif // RT_USING_DEVICE_OPS
    }
    else {
        read_size = rt_device_hack_read(device_warp->device, pos, buffer, size);
    }

    return read_size;
}

static rt_size_t device_warp_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)dev;
    LOG_D("%s device_warp_write: enter", device_warp->parent.parent.name);

    rt_size_t write_size = RT_EOK;

    if (device_warp->is_registered_device) {
        #ifdef RT_USING_DEVICE_OPS
        const struct rt_device_ops *origin_device_ops = device_warp->device->ops;
        device_warp->device->ops = &device_hack_ops;
        write_size = rt_device_write(device_warp->device, pos, buffer, size);
        if(device_warp->device->ops == &device_hack_ops) {
            device_warp->device->ops = origin_device_ops;
        }
        #else
        rt_err_t (*origin_write_func)(rt_device_t dev) = device_warp->device->write;
        device_warp->device->write = device_hack_write;
        write_size = rt_device_write(device_warp->device, pos, buffer, size);
        if (device_warp->device->write == device_hack_write) {
            device_warp->device->write = origin_write_func;
        }
        #endif // RT_USING_DEVICE_OPS
    }
    else {
        write_size = rt_device_hack_write(device_warp->device, pos, buffer, size);
    }

    return write_size;
}

static rt_err_t device_warp_control(rt_device_t dev, int cmd, void *args) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)dev;
    LOG_D("%s device_warp_control: enter", device_warp->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_warp->is_registered_device) {
        #ifdef RT_USING_DEVICE_OPS
        const struct rt_device_ops *origin_device_ops = device_warp->device->ops;
        device_warp->device->ops = &device_hack_ops;
        ret = rt_device_control(device_warp->device, cmd, args);
        if(device_warp->device->ops == &device_hack_ops) {
            device_warp->device->ops = origin_device_ops;
        }
        #else
        rt_err_t (*origin_control_func)(rt_device_t dev) = device_warp->device->control;
        device_warp->device->control = device_hack_control;
        ret = rt_device_control(device_warp->device, cmd, args);
        if (device_warp->device->control == device_hack_control) {
            device_warp->device->control = origin_control_func;
        }
        #endif // RT_USING_DEVICE_OPS
    }
    else {
        ret = rt_device_hack_control(device_warp->device, cmd, args);
    }

    return ret;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops device_warp_ops =
{
    device_warp_init,
    device_warp_open,
    device_warp_close,
    device_warp_read,
    device_warp_write,
    device_warp_control
};
#endif // RT_USING_DEVICE_OPS

#ifdef RT_USING_POSIX

static int fops_hack_open(struct dfs_fd *fd) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_warp_t *device_warp = get_device_warp(dev);

    LOG_I("%s fops_hack_open: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_open) {
            is_intercepted = interceptor_entry->interceptor->on_fops_open(fd, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_D("%s fops_hack_open: intercepted", dev->parent.name);
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_warp->device_fops->open(fd);
    }
    else {
        return dev->fops->open(fd);
    }
}

static int fops_hack_close(struct dfs_fd *fd) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_warp_t *device_warp = get_device_warp(dev);

    LOG_I("%s fops_hack_close: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_close) {
            is_intercepted = interceptor_entry->interceptor->on_fops_close(fd, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_close: intercepted", dev->parent.name);
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_warp->device_fops->close(fd);
    }
    else {
        return dev->fops->close(fd);
    }
}

static int fops_hack_ioctl(struct dfs_fd *fd, int cmd, void *args) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_warp_t *device_warp = get_device_warp(dev);

    LOG_I("%s fops_hack_ioctl: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_ioctl) {
            is_intercepted = interceptor_entry->interceptor->on_fops_ioctl(fd, &cmd, &args, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_ioctl: intercepted", dev->parent.name);
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_warp->device_fops->ioctl(fd, cmd, args);
    }
    else {
        return dev->fops->ioctl(fd, cmd, args);
    }
}

static int fops_hack_read(struct dfs_fd *fd, void *buf, size_t count) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_warp_t *device_warp = get_device_warp(dev);

    LOG_I("%s fops_hack_read: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_read) {
            is_intercepted = interceptor_entry->interceptor->on_fops_read(fd, &buf, &count, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_read: intercepted", dev->parent.name);
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_warp->device_fops->read(fd, buf, count);
    }
    else {
        return dev->fops->read(fd, buf, count);
    }
}

static int fops_hack_write(struct dfs_fd *fd, const void *buf, size_t count) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_warp_t *device_warp = get_device_warp(dev);

    LOG_I("%s fops_hack_write: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_write) {
            is_intercepted = interceptor_entry->interceptor->on_fops_write(fd, &buf, &count, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_write: intercepted", dev->parent.name);
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_warp->device_fops->write(fd, buf, count);
    }
    else {
        return dev->fops->write(fd, buf, count);
    }
}

static int fops_hack_flush(struct dfs_fd *fd) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_warp_t *device_warp = get_device_warp(dev);

    LOG_I("%s fops_hack_flush: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_flush) {
            is_intercepted = interceptor_entry->interceptor->on_fops_flush(fd, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_flush: intercepted", dev->parent.name);
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_warp->device_fops->flush(fd);
    }
    else {
        return dev->fops->flush(fd);
    }
}

static int fops_hack_lseek(struct dfs_fd *fd, off_t offset) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_warp_t *device_warp = get_device_warp(dev);

    LOG_I("%s fops_hack_lseek: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_lseek) {
            is_intercepted = interceptor_entry->interceptor->on_fops_lseek(fd, &offset, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_lseek: intercepted", dev->parent.name);
        return ret;
    }

    if (device_warp->is_registered_device) {
        return device_warp->device_fops->lseek(fd, offset);
    }
    else {
        return dev->fops->lseek(fd, offset);
    }
}

static int fops_hack_poll(struct dfs_fd *fd, struct rt_pollreq *req) {
    rt_device_warp_t *device_warp = fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_hack_poll: enter", dev->parent.name);

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_poll) {
            is_intercepted = interceptor_entry->interceptor->on_fops_poll(fd, &req, &ret, device_warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_D("%s fops_hack_poll: intercepted ret = %d", dev->parent.name, ret);
        return ret;
    }
    
    return dev->fops->poll(fd, req);
}

static const struct dfs_file_ops hack_fops = {
    .open = fops_hack_open,
    .close = fops_hack_close,
    .ioctl = fops_hack_ioctl,
    .read = fops_hack_read,
    .write = fops_hack_write,
    .flush = fops_hack_flush,
    .lseek = fops_hack_lseek,
    .getdents = RT_NULL,
    .poll = fops_hack_poll
};

static int fops_open(struct dfs_fd *fd) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_open: enter", device_warp->parent.parent.name);

    int ret = 0;

    if (device_warp->is_registered_device) {
        device_warp->device_fops = dev->fops;
        dev->fops = &hack_fops;
        char device_fullpath[RT_NAME_MAX + 6];
        rt_snprintf(device_fullpath, sizeof(device_fullpath), "/dev/%s", dev->parent.name);
        device_warp->device_fd = open(device_fullpath, fd->flags);
        if (device_warp->device_fd == -1) {
            ret = errno;
        }
        if (dev->fops == &hack_fops) {
            dev->fops = device_warp->device_fops;
        }
        else {
            LOG_E("%s dev->fops != &hack_fops", device_warp->parent.parent.name);
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_open(fd);
        fd->data = device_warp;
    }

    return ret;
}

static int fops_close(struct dfs_fd *fd) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_close: enter", device_warp->parent.parent.name);

    int ret = 0;

    if (device_warp->is_registered_device) {
        device_warp->device_fops = dev->fops;
        dev->fops = &hack_fops;
        int err = close(device_warp->device_fd);
        if (err) {
            ret = errno;
        }
        if (dev->fops == &hack_fops) {
            dev->fops = device_warp->device_fops;
        }
        else {
            LOG_E("%s dev->fops != &hack_fops", device_warp->parent.parent.name);
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_close(fd);
        fd->data = device_warp;
    }

    return ret;
}

static int fops_ioctl(struct dfs_fd *fd, int cmd, void *args) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_ioctl: enter", device_warp->parent.parent.name);

    int ret = 0;

    if (device_warp->is_registered_device) {
        device_warp->device_fops = dev->fops;
        dev->fops = &hack_fops;
        int err = ioctl(device_warp->device_fd, cmd, args);
        if (err) {
            ret = errno;
        }
        if (dev->fops == &hack_fops) {
            dev->fops = device_warp->device_fops;
        }
        else {
            LOG_E("%s dev->fops != &hack_fops", device_warp->parent.parent.name);
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_ioctl(fd, cmd, args);
        fd->data = device_warp;
    }

    return ret;
}

static int fops_read(struct dfs_fd *fd, void *buf, size_t count) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_read: enter", device_warp->parent.parent.name);

    int ret = 0;

    if (device_warp->is_registered_device) {
        device_warp->device_fops = dev->fops;
        dev->fops = &hack_fops;
        int read_size = read(device_warp->device_fd, buf, count);
        if (read_size < 0) {
            ret = errno;
        }
        else {
            ret = read_size;
        }
        if (dev->fops == &hack_fops) {
            dev->fops = device_warp->device_fops;
        }
        else {
            LOG_E("%s dev->fops != &hack_fops", device_warp->parent.parent.name);
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_read(fd, buf, count);
        fd->data = device_warp;
    }

    return ret;
}

static int fops_write(struct dfs_fd *fd, const void *buf, size_t count) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_write: enter", device_warp->parent.parent.name);

    int ret = 0;

    if (device_warp->is_registered_device) {
        device_warp->device_fops = dev->fops;
        dev->fops = &hack_fops;
        int write_size = write(device_warp->device_fd, buf, count);
        if (write_size < 0) {
            ret = errno;
        }
        else {
            ret = write_size;
        }
        if (dev->fops == &hack_fops) {
            dev->fops = device_warp->device_fops;
        }
        else {
            LOG_E("%s dev->fops != &hack_fops", device_warp->parent.parent.name);
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_write(fd, buf, count);
        fd->data = device_warp;
    }

    return ret;
}

static int fops_flush(struct dfs_fd *fd) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_flush: enter", device_warp->parent.parent.name);

    int ret = 0;

    if (device_warp->is_registered_device) {
        device_warp->device_fops = dev->fops;
        dev->fops = &hack_fops;
        ret = fsync(device_warp->device_fd);
        if (dev->fops == &hack_fops) {
            dev->fops = device_warp->device_fops;
        }
        else {
            LOG_E("%s dev->fops != &hack_fops", device_warp->parent.parent.name);
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_flush(fd);
        fd->data = device_warp;
    }

    return ret;
}

static int fops_lseek(struct dfs_fd *fd, off_t offset) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;
    rt_device_t dev = device_warp->device;

    LOG_I("%s fops_lseek: enter", device_warp->parent.parent.name);

    int ret = offset;

    if (device_warp->is_registered_device) {
        device_warp->device_fops = dev->fops;
        dev->fops = &hack_fops;
        int lseek_ret = lseek(device_warp->device_fd, offset, SEEK_SET);
        if (lseek_ret == -1) {
            ret = errno;
        }
        else {
            ret = lseek_ret;
        }
        if (dev->fops == &hack_fops) {
            dev->fops = device_warp->device_fops;
        }
        else {
            LOG_E("%s dev->fops != &hack_fops", device_warp->parent.parent.name);
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_lseek(fd, offset);
        fd->data = device_warp;
    }

    return ret;
}

static int fops_poll(struct dfs_fd *fd, struct rt_pollreq *req) {
    rt_device_warp_t *device_warp = (rt_device_warp_t *)fd->fnode->data;

    LOG_I("%s fops_poll: enter", device_warp->parent.parent.name);

    return fops_hack_poll(fd, req);
}

const static struct dfs_file_ops fops = {
    fops_open,
    fops_close,
    fops_ioctl,
    fops_read,
    fops_write,
    fops_flush,
    fops_lseek,
    RT_NULL, /* getdents */
    fops_poll,
};
#endif // RT_USING_POSIX

rt_device_warp_t * rt_device_warp_create(rt_device_t dev) {
    return rt_device_warp_create_with_data(dev, 0);
}

rt_device_warp_t * rt_device_warp_create_with_data(rt_device_t dev, rt_size_t attach_size) {
    rt_device_warp_list_node_t *warp_list_node = rt_malloc(sizeof(rt_device_warp_list_node_t));
    if (warp_list_node == RT_NULL) {
        rt_set_errno(RT_ENOMEM);
        return RT_NULL;
    }
    rt_list_init(&warp_list_node->list);

    rt_size_t aligned_attach_size = 0;
    if (attach_size > 0) {
        aligned_attach_size = RT_ALIGN(attach_size, RT_ALIGN_SIZE);
    }
    rt_size_t size = RT_ALIGN(sizeof(rt_device_warp_t), RT_ALIGN_SIZE) + aligned_attach_size;
    rt_device_warp_t *device_warp = (rt_device_warp_t *)rt_malloc(size);
    if (device_warp == RT_NULL) {
        rt_set_errno(RT_ENOMEM);
        rt_free(warp_list_node);
        return RT_NULL;
    }

    rt_memset(device_warp, 0x0, sizeof(rt_device_warp_t));
    device_warp->parent.type = dev->type;
    device_warp->device = dev;
    if (rt_device_find(dev->parent.name) != RT_NULL) {
        device_warp->is_registered_device = RT_TRUE;
    }

    rt_list_init(&device_warp->listeners);
    rt_list_init(&device_warp->interceptors);

    if (attach_size > 0) {
        rt_device_warp_set_user_data(device_warp, device_warp + 1);
    }

    warp_list_node->device_warp = device_warp;
    rt_list_insert_after(&s_device_warp_list, &warp_list_node->list);

    return device_warp;
}

void rt_device_warp_destroy(rt_device_warp_t *device_warp) {
    if (device_warp == RT_NULL) {
        return;
    }

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_listener_entry_t *tmp_listener_entry;
 
    rt_list_for_each_entry_safe(listener_entry, tmp_listener_entry, &device_warp->listeners, list) {
        rt_list_remove(&listener_entry->list);
        rt_free(listener_entry);
    }

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_device_warp_interceptor_entry_t *tmp_interceptor_entry;
    rt_list_for_each_entry_safe(interceptor_entry, tmp_interceptor_entry, &device_warp->interceptors, list) {
        rt_list_remove(&listener_entry->list);
        rt_free(listener_entry);
    }

    rt_device_warp_list_node_t *warp_list_node;
    rt_list_for_each_entry(warp_list_node, &s_device_warp_list, list) {
        if (warp_list_node->device_warp == device_warp) {
            break;
        }
    }
    rt_list_remove(&warp_list_node->list);
    rt_free(warp_list_node);

    rt_device_warp_unregister(device_warp);

    free(device_warp);
}

rt_err_t rt_device_warp_register(rt_device_warp_t *device_warp, const char *name) {
    struct rt_device *device = &(device_warp->parent);

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &device_warp_ops;
#else
    device->init        = device_warp_init;
    device->open        = device_warp_open;
    device->close       = device_warp_close;
    device->read        = device_warp_read;
    device->write       = device_warp_write;
    device->control     = device_warp_control;
#endif

    rt_err_t ret = rt_device_register(device, name, device_warp->device->flag);

#if defined(RT_USING_POSIX)
    device->fops = &fops;
#endif

    if(!device_warp->is_registered_device) {
        char name_buffer[RT_NAME_MAX];
        rt_snprintf(name_buffer, sizeof(name_buffer), "_%s", name);
        rt_strncpy(device_warp->device->parent.name, name_buffer, RT_NAME_MAX);
    }

    return ret;
}

rt_err_t rt_device_warp_unregister(rt_device_warp_t *device_warp) {
    if(rt_object_get_type(&device_warp->parent.parent) != RT_Object_Class_Device) {
        return -RT_EINVAL;
    }

    if (!rt_object_is_systemobject(&device_warp->parent.parent)) {
        return -RT_EINVAL;
    }

    return rt_device_unregister(&device_warp->parent);
}

rt_err_t rt_device_warp_register_listener(rt_device_warp_t *device_warp, rt_device_warp_listener_t *listener) {
    if (device_warp == RT_NULL || listener == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_warp_listener_entry_t *entry = rt_malloc(sizeof(rt_device_warp_listener_entry_t));

    if (entry == RT_NULL) {
        return -RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->listener = listener;
    rt_list_insert_after(&device_warp->listeners, &entry->list);

    return RT_EOK;
}

rt_err_t rt_device_warp_unregister_listener(rt_device_warp_t *device_warp, rt_device_warp_listener_t *listener) {
    if (device_warp == RT_NULL || listener == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_warp_listener_entry_t *entry;
    rt_device_warp_listener_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &device_warp->listeners, list) {
        if (entry->listener == listener) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}

rt_err_t rt_device_warp_register_interceptor(rt_device_warp_t *device_warp, rt_device_warp_interceptor_t *interceptor) {
    if (device_warp == RT_NULL || interceptor == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_warp_interceptor_entry_t *entry = rt_malloc(sizeof(rt_device_warp_interceptor_entry_t));

    if (entry == RT_NULL) {
        return -RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->interceptor = interceptor;
    rt_list_insert_after(&device_warp->interceptors, &entry->list);

    return RT_EOK;
}

rt_err_t rt_device_warp_unregister_interceptor(rt_device_warp_t *device_warp, rt_device_warp_interceptor_t *interceptor) {
    if (device_warp == RT_NULL || interceptor == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_warp_interceptor_entry_t *entry;
    rt_device_warp_interceptor_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &device_warp->interceptors, list) {
        if (entry->interceptor == interceptor) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}

const char * rt_device_warp_get_name(rt_device_warp_t *device_warp) {
    return device_warp->parent.parent.name;
}

void * rt_device_warp_get_user_data(rt_device_warp_t *device_warp) {
    RT_ASSERT(device_warp != RT_NULL)
    
    return device_warp->parent.user_data;
}

void rt_device_warp_set_user_data(rt_device_warp_t *device_warp, void *user_data) {
    RT_ASSERT(device_warp != RT_NULL)

    device_warp->parent.user_data = user_data;
}

rt_device_t rt_device_warp_get_origin_device(rt_device_warp_t *device_warp) {
    RT_ASSERT(device_warp != RT_NULL)

    return device_warp->device;
}

rt_device_t rt_device_warp_get_parent(rt_device_warp_t *warp) {
    return &warp->parent;
}