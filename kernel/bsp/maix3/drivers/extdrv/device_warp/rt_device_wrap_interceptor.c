#include "rt_device_wrap.h"
#include "rt_device_wrap_internal.h"

#ifdef RT_USING_DEVICE_OPS
#define device_init     (dev->ops->init)
#define device_open     (dev->ops->open)
#define device_close    (dev->ops->close)
#define device_read     (dev->ops->read)
#define device_write    (dev->ops->write)
#define device_control  (dev->ops->control)

#define device_hack_init     (device_wrap->device_ops->init)
#define device_hack_open     (device_wrap->device_ops->open)
#define device_hack_close    (device_wrap->device_ops->close)
#define device_hack_read     (device_wrap->device_ops->read)
#define device_hack_write    (device_wrap->device_ops->write)
#define device_hack_control  (device_wrap->device_ops->control)
#else
#define device_hack_init     (device_wrap->device_init)
#define device_hack_open     (device_wrap->device_open)
#define device_hack_close    (device_wrap->device_close)
#define device_hack_read     (device_wrap->device_read)
#define device_hack_write    (device_wrap->device_write)
#define device_hack_control  (device_wrap->device_control)

#define device_init     (dev->init)
#define device_open     (dev->open)
#define device_close    (dev->close)
#define device_read     (dev->read)
#define device_write    (dev->write)
#define device_control  (dev->control)
#endif


static rt_err_t rt_device_hack_init(rt_device_t dev) {
    LOG_I("%s hack_init: enter", dev->parent.name);
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    rt_device_wrap_listener_entry_t *listener_entry;
    rt_device_wrap_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->interceptors, list) {
        if (interceptor_entry->interceptor->on_init) {
            is_intercepted = interceptor_entry->interceptor->on_init(dev, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_wrap->listeners, list) {
        if (listener_entry->listener->on_init) {
            listener_entry->listener->on_init(dev, device_wrap->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_I("%s hack_init: intercepted!", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_hack_init(dev);
    }
    else {
        return device_init(dev);
    }
}

static rt_err_t rt_device_hack_open(rt_device_t dev, rt_uint16_t oflag) {
    LOG_I("%s hack_open: enter", dev->parent.name);
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    rt_device_wrap_listener_entry_t *listener_entry;
    rt_device_wrap_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->interceptors, list) {
        if (interceptor_entry->interceptor->on_open) {
            is_intercepted = interceptor_entry->interceptor->on_open(dev, &oflag, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_wrap->listeners, list) {
        if (listener_entry->listener->on_open) {
            listener_entry->listener->on_open(dev, oflag, device_wrap->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_I("%s hack_open: intercepted!", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_hack_open(dev, oflag);
    }
    else {
        return device_open(dev, oflag);
    }
}

static rt_err_t rt_device_hack_close(rt_device_t dev) {
    LOG_I("%s hack_close: enter", dev->parent.name);
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    rt_device_wrap_listener_entry_t *listener_entry;
    rt_device_wrap_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->interceptors, list) {
        if (interceptor_entry->interceptor->on_close) {
            is_intercepted = interceptor_entry->interceptor->on_close(device_wrap->device, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_wrap->listeners, list) {
        if (listener_entry->listener->on_close) {
            listener_entry->listener->on_close(device_wrap->device, device_wrap->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_I("%s hack_close: intercepted!", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_hack_close(dev);
    }
    else {
        return device_close(dev);
    }
}

static rt_size_t rt_device_hack_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size) {
    LOG_D("%s hack_read: enter", dev->parent.name);
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    rt_device_wrap_listener_entry_t *listener_entry;
    rt_device_wrap_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_size_t read_size = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->interceptors, list) {
        if (interceptor_entry->interceptor->on_read) {
            is_intercepted = interceptor_entry->interceptor->on_read(dev, &pos, &buffer, &size, &read_size, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_wrap->listeners, list) {
        if (listener_entry->listener) {
            if (listener_entry->listener->on_read) {
                listener_entry->listener->on_read(dev, pos, buffer, size, device_wrap->parent.user_data);
            }
        }
    }

    if (is_intercepted) {
        LOG_D("%s hack_read: intercepted!", dev->parent.name);
        return read_size;
    }

    if (device_wrap->is_registered_device) {
        return device_hack_read(dev, pos, buffer, size);
    }
    else {
        return device_read(dev, pos, buffer, size);
    }
}

static rt_size_t rt_device_hack_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size) {
    LOG_D("%s hack_write: enter", dev->parent.name);
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    rt_device_wrap_listener_entry_t *listener_entry;
    rt_device_wrap_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_size_t write_size = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->interceptors, list) {
        if (interceptor_entry->interceptor->on_write) {
            is_intercepted = interceptor_entry->interceptor->on_write(dev, &pos, &buffer, &size, &write_size, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_wrap->listeners, list) {
        if (listener_entry->listener) {
            if (listener_entry->listener->on_write) {
                listener_entry->listener->on_write(dev, pos, buffer, size, device_wrap->parent.user_data);
            }
        }
    }

    if (is_intercepted) {
        LOG_D("%s hack_write: intercepted!", dev->parent.name);
        return write_size;
    }

    if (device_wrap->is_registered_device) {
        return device_hack_write(dev, pos, buffer, size);
    }
    else {
        return device_write(dev, pos, buffer, size);
    }
}

static rt_err_t rt_device_hack_control(rt_device_t dev, int cmd, void *args) {
    LOG_I("%s hack_control: enter", dev->parent.name);
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    rt_device_wrap_listener_entry_t *listener_entry;
    rt_device_wrap_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->interceptors, list) {
        if (interceptor_entry->interceptor->on_control) {
            is_intercepted = interceptor_entry->interceptor->on_control(dev, &cmd, &args, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &device_wrap->listeners, list) {
        if (listener_entry->listener->on_control) {
            listener_entry->listener->on_control(dev, cmd, args, device_wrap->parent.user_data);
        }
    }

    if (is_intercepted) {
        LOG_I("%s hack_control: intercepted!", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_hack_control(dev, cmd, args);
    }
    else {
        return device_control(dev, cmd, args);
    }
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops device_hack_ops = {
    .init = rt_device_hack_init,
    .open = rt_device_hack_open,
    .close = rt_device_hack_close,
    .read = rt_device_hack_read,
    .write = rt_device_hack_write,
    .control = rt_device_hack_control
};
#endif // RT_USING_DEVICE_OPS

static rt_err_t device_wrap_init(rt_device_t dev) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)dev;
    LOG_D("%s device_wrap_init: enter", device_wrap->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_wrap->is_registered_device) {
        ret = rt_device_init(device_wrap->device);
    }
    else {
        ret = rt_device_hack_init(device_wrap->device);
    }

    return ret;
}

static rt_err_t device_wrap_open(rt_device_t dev, rt_uint16_t oflag) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)dev;
    LOG_D("%s device_wrap_open: enter", device_wrap->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_wrap->is_registered_device) {
        ret = rt_device_open(device_wrap->device, oflag);
    }
    else {
        ret = rt_device_hack_open(device_wrap->device, oflag);
    }

    return ret;
}

static rt_err_t device_wrap_close(rt_device_t dev) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)dev;
    LOG_D("%s device_wrap_close: enter", device_wrap->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_wrap->is_registered_device) {
        ret = rt_device_close(device_wrap->device);
    }
    else {
        ret = rt_device_hack_close(device_wrap->device);
    }

    return ret;
}

static rt_size_t device_wrap_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)dev;
    LOG_D("%s device_wrap_read: enter", device_wrap->parent.parent.name);

    rt_size_t read_size = RT_EOK;

    if (device_wrap->is_registered_device) {
        read_size = rt_device_read(device_wrap->device, pos, buffer, size);
    }
    else {
        read_size = rt_device_hack_read(device_wrap->device, pos, buffer, size);
    }

    LOG_D("%s device_wrap_read: exit", device_wrap->parent.parent.name);

    return read_size;
}

static rt_size_t device_wrap_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)dev;
    LOG_D("%s device_wrap_write: enter", device_wrap->parent.parent.name);

    rt_size_t write_size = RT_EOK;

    if (device_wrap->is_registered_device) {
        write_size = rt_device_write(device_wrap->device, pos, buffer, size);
    }
    else {
        write_size = rt_device_hack_write(device_wrap->device, pos, buffer, size);
    }

    return write_size;
}

static rt_err_t device_wrap_control(rt_device_t dev, int cmd, void *args) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)dev;
    LOG_D("%s device_wrap_control: enter", device_wrap->parent.parent.name);

    rt_err_t ret = RT_EOK;

    if (device_wrap->is_registered_device) {
        ret = rt_device_control(device_wrap->device, cmd, args);
    }
    else {
        ret = rt_device_hack_control(device_wrap->device, cmd, args);
    }

    return ret;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops device_wrap_ops =
{
    device_wrap_init,
    device_wrap_open,
    device_wrap_close,
    device_wrap_read,
    device_wrap_write,
    device_wrap_control
};
#endif // RT_USING_DEVICE_OPS

const struct rt_device_ops * get_device_hack_ops() {
    return &device_hack_ops;
}

const struct rt_device_ops * get_device_wrap_ops() {
    return &device_wrap_ops;
}

rt_err_t rt_device_wrap_register_listener(rt_device_wrap_t *device_wrap, rt_device_wrap_listener_t *listener) {
    if (device_wrap == RT_NULL || listener == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_wrap_listener_entry_t *entry = rt_malloc(sizeof(rt_device_wrap_listener_entry_t));

    if (entry == RT_NULL) {
        return -RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->listener = listener;
    rt_list_insert_after(&device_wrap->listeners, &entry->list);

    return RT_EOK;
}

rt_err_t rt_device_wrap_unregister_listener(rt_device_wrap_t *device_wrap, rt_device_wrap_listener_t *listener) {
    if (device_wrap == RT_NULL || listener == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_wrap_listener_entry_t *entry;
    rt_device_wrap_listener_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &device_wrap->listeners, list) {
        if (entry->listener == listener) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}

rt_err_t rt_device_wrap_register_interceptor(rt_device_wrap_t *device_wrap, rt_device_wrap_interceptor_t *interceptor) {
    if (device_wrap == RT_NULL || interceptor == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_wrap_interceptor_entry_t *entry = rt_malloc(sizeof(rt_device_wrap_interceptor_entry_t));

    if (entry == RT_NULL) {
        return -RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->interceptor = interceptor;
    rt_list_insert_after(&device_wrap->interceptors, &entry->list);

    return RT_EOK;
}

rt_err_t rt_device_wrap_unregister_interceptor(rt_device_wrap_t *device_wrap, rt_device_wrap_interceptor_t *interceptor) {
    if (device_wrap == RT_NULL || interceptor == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_wrap_interceptor_entry_t *entry;
    rt_device_wrap_interceptor_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &device_wrap->interceptors, list) {
        if (entry->interceptor == interceptor) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}
