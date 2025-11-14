#ifndef RT_DEVICE_WARP_H
#define RT_DEVICE_WARP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rtdef.h>
#include <dfs_file.h>

typedef struct rt_device_warp_listener {
    void (*on_init)    (rt_device_t origin_dev, void *user_data);
    void (*on_open)    (rt_device_t origin_dev, rt_uint16_t oflag, void *user_data);
    void (*on_close)   (rt_device_t origin_dev, void *user_data);
    void (*on_read)    (rt_device_t origin_dev, rt_off_t pos, void *buffer, rt_size_t size, void *user_data);
    void (*on_write)   (rt_device_t origin_dev, rt_off_t pos, const void *buffer, rt_size_t size, void *user_data);
    void (*on_control) (rt_device_t origin_dev, int cmd, void *args, void *user_data);
} rt_device_warp_listener_t;

typedef struct rt_device_warp_interceptor {
    rt_bool_t (*on_init)    (rt_device_t origin_dev, rt_err_t *ret_ptr, void *user_data);
    rt_bool_t (*on_open)    (rt_device_t origin_dev, rt_uint16_t *oflag_ptr, rt_err_t *ret_ptr, void *user_data);
    rt_bool_t (*on_close)   (rt_device_t origin_dev, rt_err_t *ret_ptr, void *user_data);
    rt_bool_t (*on_read)    (rt_device_t origin_dev, rt_off_t *pos_ptr, void **buffer_ptr, rt_size_t *size_ptr, rt_size_t *read_size_ptr, void *user_data);
    rt_bool_t (*on_write)   (rt_device_t origin_dev, rt_off_t *pos_ptr, const void **buffer_ptr, rt_size_t *size_ptr, rt_size_t *write_size_ptr, void *user_data);
    rt_bool_t (*on_control) (rt_device_t origin_dev, int *cmd_ptr, void **args_ptr, rt_err_t *ret_ptr, void *user_data);

    rt_bool_t (*on_fops_open)(struct dfs_fd *fd, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_close)(struct dfs_fd *fd, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_ioctl)(struct dfs_fd *fd, int *cmd_ptr, void **args_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_read)(struct dfs_fd *fd, void **buf_ptr, size_t *count_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_write)(struct dfs_fd *fd, const void **buf_ptr, size_t *count_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_flush)(struct dfs_fd *fd, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_lseek)(struct dfs_fd *fd, off_t *offset_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_poll)(struct dfs_fd *warp_fd, struct rt_pollreq **req_ptr, int *ret_ptr, void *user_data);
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

const char * rt_device_warp_get_name(rt_device_warp_t *warp);

void * rt_device_warp_get_user_data();

void rt_device_warp_set_user_data(rt_device_warp_t *warp, void *user_data);

rt_device_t rt_device_warp_get_origin_device(rt_device_warp_t *warp);

rt_device_t rt_device_warp_get_parent(rt_device_warp_t *warp);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*RT_DEVICE_WARP_H*/