#include "rt_device_wrap.h"
#include "rt_device_wrap_internal.h"

rt_device_wrap_t * rt_device_wrap_create(rt_device_t dev) {
    rt_device_wrap_t *device_wrap = get_device_wrap(dev);

    if (device_wrap != RT_NULL) {
        return device_wrap;
    }

    rt_device_wrap_list_node_t *wrap_list_node = rt_malloc(sizeof(rt_device_wrap_list_node_t));
    if (wrap_list_node == RT_NULL) {
        rt_set_errno(RT_ENOMEM);
        return RT_NULL;
    }
    rt_list_init(&wrap_list_node->list);

    device_wrap = (rt_device_wrap_t *)rt_malloc(sizeof(rt_device_wrap_t));
    if (device_wrap == RT_NULL) {
        rt_set_errno(RT_ENOMEM);
        rt_free(wrap_list_node);
        return RT_NULL;
    }

    rt_memset(device_wrap, 0x0, sizeof(rt_device_wrap_t));
    device_wrap->parent.type = dev->type;
    device_wrap->device = dev;
    if (rt_device_find(dev->parent.name) != RT_NULL) {
        device_wrap->is_registered_device = RT_TRUE;
    }
    device_wrap->device_fd = -1;

    rt_list_init(&device_wrap->listeners);
    rt_list_init(&device_wrap->interceptors);
    rt_list_init(&device_wrap->fops_interceptors);

    wrap_list_node->device_wrap = device_wrap;
    rt_list_insert_after(get_device_wrap_list(), &wrap_list_node->list);

    return device_wrap;
}

void rt_device_wrap_destroy(rt_device_wrap_t *device_wrap) {
    if (device_wrap == RT_NULL) {
        return;
    }

    rt_device_wrap_listener_entry_t *listener_entry;
    rt_device_wrap_listener_entry_t *tmp_listener_entry;
 
    rt_list_for_each_entry_safe(listener_entry, tmp_listener_entry, &device_wrap->listeners, list) {
        rt_list_remove(&listener_entry->list);
        rt_free(listener_entry);
    }

    rt_device_wrap_interceptor_entry_t *interceptor_entry;
    rt_device_wrap_interceptor_entry_t *tmp_interceptor_entry;
    rt_list_for_each_entry_safe(interceptor_entry, tmp_interceptor_entry, &device_wrap->interceptors, list) {
        rt_list_remove(&interceptor_entry->list);
        rt_free(interceptor_entry);
    }

    rt_device_wrap_fops_interceptor_entry_t *fops_interceptor_entry;
    rt_device_wrap_fops_interceptor_entry_t *tmp_fops_interceptor_entry;
    rt_list_for_each_entry_safe(fops_interceptor_entry, tmp_fops_interceptor_entry, &device_wrap->fops_interceptors, list) {
        rt_list_remove(&fops_interceptor_entry->list);
        rt_free(fops_interceptor_entry);
    }

    rt_device_wrap_list_node_t *wrap_list_node;
    rt_list_for_each_entry(wrap_list_node, get_device_wrap_list(), list) {
        if (wrap_list_node->device_wrap == device_wrap) {
            break;
        }
    }
    rt_list_remove(&wrap_list_node->list);
    rt_free(wrap_list_node);

    rt_device_wrap_unregister(device_wrap);

    free(device_wrap);
}

rt_err_t rt_device_wrap_register(rt_device_wrap_t *device_wrap, const char *name) {
    struct rt_device *device = &(device_wrap->parent);

#ifdef RT_USING_DEVICE_OPS
    device->ops         = get_device_wrap_ops();
#else
    device->init        = device_wrap_init;
    device->open        = device_wrap_open;
    device->close       = device_wrap_close;
    device->read        = device_wrap_read;
    device->write       = device_wrap_write;
    device->control     = device_wrap_control;
#endif

    rt_err_t ret = rt_device_register(device, name, device_wrap->device->flag);

#if defined(RT_USING_POSIX)
    device->fops = get_device_wrap_fops();
#endif

    if(!device_wrap->is_registered_device) {
        char name_buffer[RT_NAME_MAX];
        rt_snprintf(name_buffer, sizeof(name_buffer), "_%s", name);
        rt_strncpy(device_wrap->device->parent.name, name_buffer, RT_NAME_MAX);
    }

    const struct rt_device_ops *hack_ops = get_device_hack_ops();
    if (device_wrap->device->ops != hack_ops) {
        device_wrap->device_ops = device_wrap->device->ops;
        device_wrap->device->ops = hack_ops;
    }

    const struct dfs_file_ops *device_hack_fops = get_device_hack_fops();
    if (device_wrap->device->fops != device_hack_fops) {
        device_wrap->device_fops = device_wrap->device->fops;
        device_wrap->device->fops = device_hack_fops;
    }

    return ret;
}

rt_err_t rt_device_wrap_unregister(rt_device_wrap_t *device_wrap) {
    if(rt_object_get_type(&device_wrap->parent.parent) != RT_Object_Class_Device) {
        return -RT_EINVAL;
    }

    if (!rt_object_is_systemobject(&device_wrap->parent.parent)) {
        return -RT_EINVAL;
    }

    const struct rt_device_ops *hack_ops = get_device_hack_ops();
    if(device_wrap->device->ops == hack_ops) {
        device_wrap->device->ops = device_wrap->device_ops;
    }
    else {
        LOG_W("ops of the origin device was changed!");
    }


    const struct dfs_file_ops *device_hack_fops = get_device_hack_fops();
    if (device_wrap->device->fops == device_hack_fops) {
        device_wrap->device->fops = device_wrap->device_fops;
    }
    else {
        LOG_W("fops of the origin device was changed!");
    }

    return rt_device_unregister(&device_wrap->parent);
}

const char * rt_device_wrap_get_name(rt_device_wrap_t *device_wrap) {
    return device_wrap->parent.parent.name;
}

void * rt_device_wrap_get_user_data(rt_device_wrap_t *device_wrap) {
    RT_ASSERT(device_wrap != RT_NULL)
    
    return device_wrap->parent.user_data;
}

void rt_device_wrap_set_user_data(rt_device_wrap_t *device_wrap, void *user_data) {
    RT_ASSERT(device_wrap != RT_NULL)

    device_wrap->parent.user_data = user_data;
}

rt_device_t rt_device_wrap_get_origin_device(rt_device_wrap_t *device_wrap) {
    RT_ASSERT(device_wrap != RT_NULL)

    return device_wrap->device;
}

rt_device_t rt_device_wrap_get_parent(rt_device_wrap_t *wrap) {
    return &wrap->parent;
}
