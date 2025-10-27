#include <rt_device_wrap.h>
#include <rtthread.h>

#ifdef RT_USING_POSIX
#include <dfs_posix.h>
#include <dfs_poll.h>
#endif // RT_USING_POSIX

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
    rt_list_t listeners;
    rt_list_t interceptors;
};

static rt_err_t device_warp_init(rt_device_t dev) {
    rt_device_warp_t *warp = (rt_device_warp_t *)dev;
    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_init) {
            is_intercepted = interceptor_entry->interceptor->on_init(warp->device, &ret, warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &warp->listeners, list) {
        if (listener_entry->listener->on_init) {
            listener_entry->listener->on_init(warp->device, warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        return ret;
    }

    return rt_device_init(warp->device);
}

static rt_err_t device_warp_open(rt_device_t dev, rt_uint16_t oflag) {
    rt_device_warp_t *warp = (rt_device_warp_t *)dev;
    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_open) {
            is_intercepted = interceptor_entry->interceptor->on_open(warp->device, &oflag, &ret, warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &warp->listeners, list) {
        if (listener_entry->listener->on_open) {
            listener_entry->listener->on_open(warp->device, oflag, warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        return ret;
    }

    return rt_device_open(warp->device, oflag);
}

static rt_err_t device_warp_close(rt_device_t dev) {
    rt_device_warp_t *warp = (rt_device_warp_t *)dev;
    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_close) {
            is_intercepted = interceptor_entry->interceptor->on_close(warp->device, &ret, warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &warp->listeners, list) {
        if (listener_entry->listener->on_close) {
            listener_entry->listener->on_close(warp->device, warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        return ret;
    }

    return rt_device_close(warp->device);
}

static rt_size_t device_warp_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size) {
    rt_device_warp_t *warp = (rt_device_warp_t *)dev;
    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_size_t read_size = 0;

    rt_list_for_each_entry(interceptor_entry, &warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_read) {
            is_intercepted = interceptor_entry->interceptor->on_read(warp->device, &pos, buffer, &size, &read_size, warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &warp->listeners, list) {
        if (listener_entry->listener) {
            if (listener_entry->listener->on_read) {
                listener_entry->listener->on_read(warp->device, pos, buffer, size, warp->parent.user_data);
            }
        }
    }

    if (is_intercepted) {
        return read_size;
    }

    return rt_device_read(warp->device, pos, buffer, size);
}

static rt_size_t device_warp_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size) {
    rt_device_warp_t *warp = (rt_device_warp_t *)dev;
    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_size_t write_size = 0;

    rt_list_for_each_entry(interceptor_entry, &warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_write) {
            is_intercepted = interceptor_entry->interceptor->on_write(warp->device, &pos, buffer, &size, &write_size, warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &warp->listeners, list) {
        if (listener_entry->listener) {
            if (listener_entry->listener->on_write) {
                listener_entry->listener->on_write(warp->device, pos, buffer, size, warp->parent.user_data);
            }
        }
    }

    if (is_intercepted) {
        return write_size;
    }

    return rt_device_write(warp->device, pos, buffer, size);
}

static rt_err_t device_warp_control(rt_device_t dev, int cmd, void *args) {
    rt_device_warp_t *warp = (rt_device_warp_t *)dev;
    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    rt_err_t ret = RT_EOK;

    rt_list_for_each_entry(interceptor_entry, &warp->interceptors, list) {
        if (interceptor_entry->interceptor->on_control) {
            is_intercepted = interceptor_entry->interceptor->on_control(warp->device, &cmd, args, &ret, warp->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    rt_list_for_each_entry(listener_entry, &warp->listeners, list) {
        if (listener_entry->listener->on_control) {
            listener_entry->listener->on_control(warp->device, cmd, args, warp->parent.user_data);
        }
    }

    if (is_intercepted) {
        return ret;
    }

    return rt_device_control(warp->device, cmd, args);
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
static int fops_open(struct dfs_fd *fd) {
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->open) {
            return warp->device->fops->open(fd);
        }
    }
    return -ENOSYS;
}

static int fops_close(struct dfs_fd *fd) {
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->close) {
            return warp->device->fops->close(fd);
        }
    }
    return -ENOSYS;
}

static int fops_ioctl(struct dfs_fd *fd, int cmd, void *args) {
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->ioctl) {
            return warp->device->fops->ioctl(fd, cmd, args);
        }
    }
    return -ENOSYS;
}

static int fops_read(struct dfs_fd *fd, void *buf, size_t count) {
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->read) {
            return warp->device->fops->read(fd, buf, count);
        }
    }
    return -ENOSYS;
}

static int fops_write(struct dfs_fd *fd, const void *buf, size_t count) {
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->write) {
            return warp->device->fops->write(fd, buf, count);
        }
    }
    return -ENOSYS;
}

static int fops_flush(struct dfs_fd *fd) {
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->flush) {
            return warp->device->fops->flush(fd);
        }
    }
    return -ENOSYS;
}

static int fops_lseek(struct dfs_fd *fd, off_t offset) {
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->lseek) {
            return warp->device->fops->lseek(fd, offset);
        }
    }
    return -ENOSYS;
}

static int fops_poll(struct dfs_fd *fd, struct rt_pollreq *req)
{
    rt_device_warp_t *warp = (rt_device_warp_t *)fd->fnode->data;
    if (warp->device->fops) {
        if (warp->device->fops->poll) {
            return warp->device->fops->poll(fd, req);
        }
    }
    return -ENOSYS;
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
    rt_size_t aligned_attach_size = 0;
    if (attach_size > 0) {
        aligned_attach_size = RT_ALIGN(attach_size, RT_ALIGN_SIZE);
    }
    rt_size_t size = RT_ALIGN(sizeof(rt_device_warp_t), RT_ALIGN_SIZE) + aligned_attach_size;
    rt_device_warp_t *warp = (rt_device_warp_t *)rt_malloc(size);
    if (warp)
    {
        rt_memset(warp, 0x0, sizeof(rt_device_warp_t));
        warp->parent.type = dev->type;
        warp->device = dev;

        rt_list_init(&warp->listeners);
        rt_list_init(&warp->interceptors);

        if (attach_size > 0) {
            rt_device_warp_set_user_data(warp, warp + 1);
        }
    }
    return warp;
}

void rt_device_warp_destroy(rt_device_warp_t *warp) {
    if (warp == RT_NULL) {
        return;
    }

    rt_device_warp_listener_entry_t *listener_entry;
    rt_device_warp_listener_entry_t *tmp_listener_entry;
 
    rt_list_for_each_entry_safe(listener_entry, tmp_listener_entry, &warp->listeners, list) {
        rt_list_remove(&listener_entry->list);
        rt_free(listener_entry);
    }

    rt_device_warp_interceptor_entry_t *interceptor_entry;
    rt_device_warp_interceptor_entry_t *tmp_interceptor_entry;
    rt_list_for_each_entry_safe(interceptor_entry, tmp_interceptor_entry, &warp->interceptors, list) {
        rt_list_remove(&listener_entry->list);
        rt_free(listener_entry);
    }

    rt_device_destroy((rt_device_t)warp);
}

rt_err_t rt_device_warp_register(rt_device_warp_t *warp, const char *name) {
    struct rt_device *device = &(warp->parent);

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

    rt_err_t ret = rt_device_register(device, name, warp->device->flag);

#if defined(RT_USING_POSIX)
    device->fops = &fops;
#endif

    return ret;
}

rt_err_t rt_device_warp_unregister(rt_device_warp_t *warp) {
    return rt_device_unregister(&warp->parent);
}

rt_err_t rt_device_warp_register_listener(rt_device_warp_t *warp, rt_device_warp_listener_t *listener) {
    if (warp == RT_NULL || listener == RT_NULL) {
        return RT_EINVAL;
    }

    rt_device_warp_listener_entry_t *entry = rt_malloc(sizeof(rt_device_warp_listener_entry_t));

    if (entry == RT_NULL) {
        return RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->listener = listener;
    rt_list_insert_after(&warp->listeners, &entry->list);

    return RT_EOK;
}

rt_err_t rt_device_warp_unregister_listener(rt_device_warp_t *warp, rt_device_warp_listener_t *listener) {
    if (warp == RT_NULL || listener == RT_NULL) {
        return RT_EINVAL;
    }

    rt_device_warp_listener_entry_t *entry;
    rt_device_warp_listener_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &warp->listeners, list) {
        if (entry->listener == listener) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}

rt_err_t rt_device_warp_register_interceptor(rt_device_warp_t *warp, rt_device_warp_interceptor_t *interceptor) {
    if (warp == RT_NULL || interceptor == RT_NULL) {
        return RT_EINVAL;
    }

    rt_device_warp_interceptor_entry_t *entry = rt_malloc(sizeof(rt_device_warp_interceptor_entry_t));

    if (entry == RT_NULL) {
        return RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->interceptor = interceptor;
    rt_list_insert_after(&warp->interceptors, &entry->list);

    return RT_EOK;
}

rt_err_t rt_device_warp_unregister_interceptor(rt_device_warp_t *warp, rt_device_warp_interceptor_t *interceptor) {
    if (warp == RT_NULL || interceptor == RT_NULL) {
        return RT_EINVAL;
    }

    rt_device_warp_interceptor_entry_t *entry;
    rt_device_warp_interceptor_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &warp->interceptors, list) {
        if (entry->interceptor == interceptor) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}

void * rt_device_warp_get_user_data(rt_device_warp_t *warp) {
    RT_ASSERT(warp != RT_NULL)
    
    return warp->parent.user_data;
}

void rt_device_warp_set_user_data(rt_device_warp_t *warp, void *user_data) {
    RT_ASSERT(warp != RT_NULL)

    warp->parent.user_data = user_data;
}

rt_device_t rt_device_warp_get_origin_device(rt_device_warp_t *warp) {
    RT_ASSERT(warp != RT_NULL)

    return warp->device;
}