#ifndef RT_DEVICE_WARP_H
#define RT_DEVICE_WARP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rtdef.h>

typedef struct rt_device_warp_listener {
    void (*on_init)    (rt_device_t origin_dev, void *user_data);
    void (*on_open)    (rt_device_t origin_dev, rt_uint16_t oflag, void *user_data);
    void (*on_close)   (rt_device_t origin_dev, void *user_data);
    void (*on_read)    (rt_device_t origin_dev, rt_off_t pos, void *buffer, rt_size_t size, void *user_data);
    void (*on_write)   (rt_device_t origin_dev, rt_off_t pos, const void *buffer, rt_size_t size, void *user_data);
    void (*on_control) (rt_device_t origin_dev, int cmd, void *args, void *user_data);
} rt_device_warp_listener_t;

typedef struct rt_device_warp_interceptor {
    rt_bool_t (*on_init)    (rt_device_t origin_dev, rt_err_t *ret, void *user_data);
    rt_bool_t (*on_open)    (rt_device_t origin_dev, rt_uint16_t *oflag, rt_err_t *ret, void *user_data);
    rt_bool_t (*on_close)   (rt_device_t origin_dev, rt_err_t *ret, void *user_data);
    rt_bool_t (*on_read)    (rt_device_t origin_dev, rt_off_t *pos, void *buffer, rt_size_t *size, rt_size_t *read_size, void *user_data);
    rt_bool_t (*on_write)   (rt_device_t origin_dev, rt_off_t *pos, const void *buffer, rt_size_t *size, rt_size_t *write_size, void *user_data);
    rt_bool_t (*on_control) (rt_device_t origin_dev, int *cmd, void *args, rt_err_t *ret, void *user_data);
} rt_device_warp_interceptor_t;

typedef struct rt_device_warp rt_device_warp_t;

rt_device_warp_t * rt_device_warp_create(rt_device_t dev);

rt_device_warp_t * rt_device_warp_create_with_data(rt_device_t dev, rt_size_t attach_size);

void rt_device_warp_destroy(rt_device_warp_t *warp);

rt_err_t rt_device_warp_register(rt_device_warp_t *warp, const char *name);

rt_err_t rt_device_warp_unregister(rt_device_warp_t *warp);

rt_err_t rt_device_warp_register_listener(rt_device_warp_t *warp, rt_device_warp_listener_t *listener);

rt_err_t rt_device_warp_unregister_listener(rt_device_warp_t *warp, rt_device_warp_listener_t *listener);

rt_err_t rt_device_warp_register_interceptor(rt_device_warp_t *warp, rt_device_warp_interceptor_t *interceptor);

rt_err_t rt_device_warp_unregister_interceptor(rt_device_warp_t *warp, rt_device_warp_interceptor_t *interceptor);

void * rt_device_warp_get_user_data();

void rt_device_warp_set_user_data(rt_device_warp_t *warp, void *user_data);

rt_device_t rt_device_warp_get_origin_device(rt_device_warp_t *warp);


#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*RT_DEVICE_WARP_H*/