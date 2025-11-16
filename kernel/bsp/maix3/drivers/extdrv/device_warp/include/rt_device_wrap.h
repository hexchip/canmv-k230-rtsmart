#ifndef RT_DEVICE_WRAP_H
#define RT_DEVICE_WRAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rtdef.h>

typedef struct rt_device_wrap_listener {
    void (*on_init)    (rt_device_t origin_dev, void *user_data);
    void (*on_open)    (rt_device_t origin_dev, rt_uint16_t oflag, void *user_data);
    void (*on_close)   (rt_device_t origin_dev, void *user_data);
    void (*on_read)    (rt_device_t origin_dev, rt_off_t pos, void *buffer, rt_size_t size, void *user_data);
    void (*on_write)   (rt_device_t origin_dev, rt_off_t pos, const void *buffer, rt_size_t size, void *user_data);
    void (*on_control) (rt_device_t origin_dev, int cmd, void *args, void *user_data);
} rt_device_wrap_listener_t;

typedef struct rt_device_wrap_interceptor {
    rt_bool_t (*on_init)    (rt_device_t origin_dev, rt_err_t *ret_ptr, void *user_data);
    rt_bool_t (*on_open)    (rt_device_t origin_dev, rt_uint16_t *oflag_ptr, rt_err_t *ret_ptr, void *user_data);
    rt_bool_t (*on_close)   (rt_device_t origin_dev, rt_err_t *ret_ptr, void *user_data);
    rt_bool_t (*on_read)    (rt_device_t origin_dev, rt_off_t *pos_ptr, void **buffer_ptr, rt_size_t *size_ptr, rt_size_t *read_size_ptr, void *user_data);
    rt_bool_t (*on_write)   (rt_device_t origin_dev, rt_off_t *pos_ptr, const void **buffer_ptr, rt_size_t *size_ptr, rt_size_t *write_size_ptr, void *user_data);
    rt_bool_t (*on_control) (rt_device_t origin_dev, int *cmd_ptr, void **args_ptr, rt_err_t *ret_ptr, void *user_data);
} rt_device_wrap_interceptor_t;

#ifdef RT_USING_POSIX

#include <dfs_file.h>

typedef struct rt_device_wrap_fops_interceptor {
    rt_bool_t (*on_fops_open)(struct dfs_fd *fd, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_close)(struct dfs_fd *fd, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_ioctl)(struct dfs_fd *fd, int *cmd_ptr, void **args_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_read)(struct dfs_fd *fd, void **buf_ptr, size_t *count_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_write)(struct dfs_fd *fd, const void **buf_ptr, size_t *count_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_flush)(struct dfs_fd *fd, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_lseek)(struct dfs_fd *fd, off_t *offset_ptr, int *ret_ptr, void *user_data);
    rt_bool_t (*on_fops_poll)(struct dfs_fd *wrap_fd, struct rt_pollreq **req_ptr, int *ret_ptr, void *user_data);
} rt_device_wrap_fops_interceptor_t;

#endif // RT_USING_POSIX

typedef struct rt_device_wrap rt_device_wrap_t;

rt_device_wrap_t * rt_device_wrap_create(rt_device_t dev);

void rt_device_wrap_destroy(rt_device_wrap_t *wrap);

rt_err_t rt_device_wrap_register(rt_device_wrap_t *wrap, const char *name);

rt_err_t rt_device_wrap_unregister(rt_device_wrap_t *wrap);

rt_err_t rt_device_wrap_register_listener(rt_device_wrap_t *wrap, rt_device_wrap_listener_t *listener);

rt_err_t rt_device_wrap_unregister_listener(rt_device_wrap_t *wrap, rt_device_wrap_listener_t *listener);

rt_err_t rt_device_wrap_register_interceptor(rt_device_wrap_t *wrap, rt_device_wrap_interceptor_t *interceptor);

rt_err_t rt_device_wrap_unregister_interceptor(rt_device_wrap_t *wrap, rt_device_wrap_interceptor_t *interceptor);

#ifdef RT_USING_POSIX

rt_err_t rt_device_wrap_register_fops_interceptor(rt_device_wrap_t *wrap, rt_device_wrap_fops_interceptor_t *interceptor);

rt_err_t rt_device_wrap_unregister_fops_interceptor(rt_device_wrap_t *wrap, rt_device_wrap_fops_interceptor_t *interceptor);

#endif // RT_USING_POSIX

const char * rt_device_wrap_get_name(rt_device_wrap_t *wrap);

void * rt_device_wrap_get_user_data();

void rt_device_wrap_set_user_data(rt_device_wrap_t *wrap, void *user_data);

rt_device_t rt_device_wrap_get_origin_device(rt_device_wrap_t *wrap);

rt_device_t rt_device_wrap_get_parent(rt_device_wrap_t *wrap);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*RT_DEVICE_WRAP_H*/