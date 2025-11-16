#include "rt_device_wrap.h"
#include "rt_device_wrap_internal.h"

#ifdef RT_USING_POSIX

#include <dfs_posix.h>
#include <poll.h>

static int fops_hack_open(struct dfs_fd *fd) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    LOG_I("%s fops_hack_open: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_open) {
            is_intercepted = interceptor_entry->interceptor->on_fops_open(fd, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_open: intercepted", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_wrap->device_fops->open(fd);
    }
    else {
        return dev->fops->open(fd);
    }
}

static int fops_hack_close(struct dfs_fd *fd) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    LOG_I("%s fops_hack_close: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_close) {
            LOG_I("%s fops_hack_close: on_fops_close 1", dev->parent.name);
            is_intercepted = interceptor_entry->interceptor->on_fops_close(fd, &ret, device_wrap->parent.user_data);
            LOG_I("%s fops_hack_close: on_fops_close 2", dev->parent.name);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_close: intercepted", dev->parent.name);
        return ret;
    }

    LOG_I("%s fops_hack_close: on_fops_close 3", dev->parent.name);
    if (device_wrap->is_registered_device) {
        LOG_I("%s fops_hack_close: on_fops_close 4", dev->parent.name);
        return device_wrap->device_fops->close(fd);
        LOG_I("%s fops_hack_close: on_fops_close 5", dev->parent.name);
    }
    else {
        LOG_I("%s fops_hack_close: on_fops_close 6", dev->parent.name);
        return dev->fops->close(fd);
        LOG_I("%s fops_hack_close: on_fops_close 7", dev->parent.name);
    }
}

static int fops_hack_ioctl(struct dfs_fd *fd, int cmd, void *args) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    LOG_I("%s fops_hack_ioctl: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_ioctl) {
            is_intercepted = interceptor_entry->interceptor->on_fops_ioctl(fd, &cmd, &args, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_ioctl: intercepted", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_wrap->device_fops->ioctl(fd, cmd, args);
    }
    else {
        return dev->fops->ioctl(fd, cmd, args);
    }
}

static int fops_hack_read(struct dfs_fd *fd, void *buf, size_t count) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    LOG_I("%s fops_hack_read: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_read) {
            is_intercepted = interceptor_entry->interceptor->on_fops_read(fd, &buf, &count, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_read: intercepted", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_wrap->device_fops->read(fd, buf, count);
    }
    else {
        return dev->fops->read(fd, buf, count);
    }
}

static int fops_hack_write(struct dfs_fd *fd, const void *buf, size_t count) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    LOG_I("%s fops_hack_write: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_write) {
            is_intercepted = interceptor_entry->interceptor->on_fops_write(fd, &buf, &count, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_write: intercepted", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_wrap->device_fops->write(fd, buf, count);
    }
    else {
        return dev->fops->write(fd, buf, count);
    }
}

static int fops_hack_flush(struct dfs_fd *fd) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    LOG_I("%s fops_hack_flush: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_flush) {
            is_intercepted = interceptor_entry->interceptor->on_fops_flush(fd, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_flush: intercepted", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_wrap->device_fops->flush(fd);
    }
    else {
        return dev->fops->flush(fd);
    }
}

static int fops_hack_lseek(struct dfs_fd *fd, off_t offset) {
    rt_device_t dev = (rt_device_t)fd->fnode->data;
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    LOG_I("%s fops_hack_lseek: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_lseek) {
            is_intercepted = interceptor_entry->interceptor->on_fops_lseek(fd, &offset, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_lseek: intercepted", dev->parent.name);
        return ret;
    }

    if (device_wrap->is_registered_device) {
        return device_wrap->device_fops->lseek(fd, offset);
    }
    else {
        return dev->fops->lseek(fd, offset);
    }
}

static int fops_hack_poll(struct dfs_fd *fd, struct rt_pollreq *req) {
    rt_device_wrap_t *device_wrap = fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_hack_poll: enter", dev->parent.name);

    rt_device_wrap_fops_interceptor_entry_t *interceptor_entry;
    rt_bool_t is_intercepted = RT_FALSE;
    int ret = 0;

    rt_list_for_each_entry(interceptor_entry, &device_wrap->fops_interceptors, list) {
        if (interceptor_entry->interceptor->on_fops_poll) {
            is_intercepted = interceptor_entry->interceptor->on_fops_poll(fd, &req, &ret, device_wrap->parent.user_data);
            if (is_intercepted) {
                break;
            }
        }
    }

    if (is_intercepted) {
        LOG_I("%s fops_hack_poll: intercepted ret = %d", dev->parent.name, ret);
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


static int get_dfs_fd(struct dfs_fd *dfs_fd) {
    struct dfs_fdtable *fdt = dfs_fdtable_get();
    int fd = -1;

    dfs_fd_lock();

    for(int i = 0; i < fdt->maxfd; i++) {
        if(fdt->fds[i] == dfs_fd) {
            fd = i;
        }
    }

    dfs_fd_unlock();

    return fd;
}

static int fops_open(struct dfs_fd *fd) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_open: enter", device_wrap->parent.parent.name);

    int ret = 0;

    rt_base_t level = rt_hw_interrupt_disable();

    if (device_wrap->is_registered_device) {
        char device_fullpath[RT_NAME_MAX + 6];
        rt_snprintf(device_fullpath, sizeof(device_fullpath), "/dev/%s", dev->parent.name);

        if (device_wrap->device_fd < 0) {
            device_wrap->device_fd = open(device_fullpath, fd->flags);
            if (device_wrap->device_fd == -1) {
                ret = errno;
            }
        }

    }
    else {
        fd->data = dev;
        ret = fops_hack_open(fd);
        fd->data = device_wrap;
    }

    rt_hw_interrupt_enable(level);

    return ret;
}

static int fops_close(struct dfs_fd *fd) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_close: enter", device_wrap->parent.parent.name);

    int ret = 0;

    rt_base_t level = rt_hw_interrupt_disable();

    if (device_wrap->is_registered_device) {
        int err = close(device_wrap->device_fd);
        if (err) {
            ret = errno;
        }
        else {
            device_wrap->device_fd = -1;
        }
    }
    else {
        fd->data = dev;
        ret = fops_hack_close(fd);
        fd->data = device_wrap;
    }

    rt_hw_interrupt_enable(level);

    return ret;
}

static int fops_ioctl(struct dfs_fd *fd, int cmd, void *args) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_ioctl: enter", device_wrap->parent.parent.name);

    int ret = 0;

    if (device_wrap->is_registered_device) {
        int err = ioctl(device_wrap->device_fd, cmd, args);
        if (err) {
            ret = errno;
        }
    }
    else {
        rt_base_t level = rt_hw_interrupt_disable();
        fd->data = dev;
        ret = fops_hack_ioctl(fd, cmd, args);
        fd->data = device_wrap;
        rt_hw_interrupt_enable(level);
    }

    return ret;
}

static int fops_read(struct dfs_fd *fd, void *buf, size_t count) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_read: enter", device_wrap->parent.parent.name);

    int ret = 0;

    if (device_wrap->is_registered_device) {
        int read_size = read(device_wrap->device_fd, buf, count);
        if (read_size < 0) {
            ret = errno;
        }
        else {
            ret = read_size;
        }
    }
    else {
        rt_base_t level = rt_hw_interrupt_disable();
        fd->data = dev;
        ret = fops_hack_read(fd, buf, count);
        fd->data = device_wrap;
        rt_hw_interrupt_enable(level);
    }

    return ret;
}

static int fops_write(struct dfs_fd *fd, const void *buf, size_t count) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_write: enter", device_wrap->parent.parent.name);

    int ret = 0;

    if (device_wrap->is_registered_device) {
        int write_size = write(device_wrap->device_fd, buf, count);
        if (write_size < 0) {
            ret = errno;
        }
        else {
            ret = write_size;
        }
    }
    else {
        rt_base_t level = rt_hw_interrupt_disable();
        fd->data = dev;
        ret = fops_hack_write(fd, buf, count);
        fd->data = device_wrap;
        rt_hw_interrupt_enable(level);
    }

    return ret;
}

static int fops_flush(struct dfs_fd *fd) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_flush: enter", device_wrap->parent.parent.name);

    int ret = 0;

    if (device_wrap->is_registered_device) {
        ret = fsync(device_wrap->device_fd);
    }
    else {
        rt_base_t level = rt_hw_interrupt_disable();
        fd->data = dev;
        ret = fops_hack_flush(fd);
        fd->data = device_wrap;
        rt_hw_interrupt_enable(level);
    }

    return ret;
}

static int fops_lseek(struct dfs_fd *fd, off_t offset) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;
    rt_device_t dev = device_wrap->device;

    LOG_I("%s fops_lseek: enter", device_wrap->parent.parent.name);

    int ret = offset;

    if (device_wrap->is_registered_device) {
        int lseek_ret = lseek(device_wrap->device_fd, offset, SEEK_SET);
        if (lseek_ret == -1) {
            ret = errno;
        }
        else {
            ret = lseek_ret;
        }
    }
    else {
        rt_base_t level = rt_hw_interrupt_disable();
        fd->data = dev;
        ret = fops_hack_lseek(fd, offset);
        fd->data = device_wrap;
        rt_hw_interrupt_enable(level);
    }

    return ret;
}

static int fops_poll(struct dfs_fd *fd, struct rt_pollreq *req) {
    rt_device_wrap_t *device_wrap = (rt_device_wrap_t *)fd->fnode->data;

    LOG_I("%s fops_poll: enter", device_wrap->parent.parent.name);

    return fops_hack_poll(fd, req);
}

static const struct dfs_file_ops fops = {
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

const struct dfs_file_ops * get_device_hack_fops() {
    return &hack_fops;
}

const struct dfs_file_ops * get_device_wrap_fops() {
    return &fops;
}

rt_err_t rt_device_wrap_register_fops_interceptor(rt_device_wrap_t *device_wrap, rt_device_wrap_fops_interceptor_t *interceptor) {
    if (device_wrap == RT_NULL || interceptor == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_wrap_fops_interceptor_entry_t *entry = rt_malloc(sizeof(rt_device_wrap_fops_interceptor_entry_t));

    if (entry == RT_NULL) {
        return -RT_ENOMEM;
    }

    rt_list_init(&entry->list);
    entry->interceptor = interceptor;
    rt_list_insert_after(&device_wrap->fops_interceptors, &entry->list);

    return RT_EOK;
}

rt_err_t rt_device_wrap_unregister_fops_interceptor(rt_device_wrap_t *device_wrap, rt_device_wrap_fops_interceptor_t *interceptor) {
    if (device_wrap == RT_NULL || interceptor == RT_NULL) {
        return -RT_EINVAL;
    }

    rt_device_wrap_fops_interceptor_entry_t *entry;
    rt_device_wrap_fops_interceptor_entry_t *tmp;
 
    rt_list_for_each_entry_safe(entry, tmp, &device_wrap->fops_interceptors, list) {
        if (entry->interceptor == interceptor) {
            rt_list_remove(&entry->list);
            rt_free(entry);
        }
    }

    return RT_EOK;
}

#endif // RT_USING_POSIX
