#include "rt_device_wrap.h"
#include "rt_device_wrap_internal.h"

static rt_list_t s_device_wrap_list = RT_LIST_OBJECT_INIT(s_device_wrap_list);

rt_list_t * get_device_wrap_list() {
    return &s_device_wrap_list;
}

rt_device_wrap_t * get_device_wrap(rt_device_t dev) {
    rt_device_wrap_t *device_wrap = RT_NULL;
    rt_device_wrap_list_node_t *node = RT_NULL;
    rt_list_for_each_entry(node, &s_device_wrap_list, list) {
        if (node->device_wrap->device == dev) {
            device_wrap = node->device_wrap;
            break;
        }
    }
    return device_wrap;
}

