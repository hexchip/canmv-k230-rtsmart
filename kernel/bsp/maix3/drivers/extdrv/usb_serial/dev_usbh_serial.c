#include "dev_usbh_serial.h"

#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>

#include "drivers/serial.h"
#include "lwp_user_mm.h"

#include "rt_device_wrap.h"

#define DBG_SECTION_NAME    "DEV_USBH_SERIAL"
#define DBG_LEVEL           DBG_INFO
#include <rtdbg.h>

#define USBH_SERIAL_ASYNC_TRANSMIT_MAX_RETRY 5
#define USBH_SERIAL_RT_SEM_MAX_VALUE 1

struct usbh_serial_dev {
    rt_serial_t parent;
    void *usbh_serial;
    rt_mutex_t mutex;
    rt_sem_t rx_sem;
    rt_sem_t tx_sem;
    rt_mq_t mq;
    rt_thread_t thread;
    dev_usbh_serial_ops_t *ops;
    uint8_t *rx_buffer;
    size_t rx_buffer_size;
    bool is_open;
    volatile bool is_thread_running;
    rt_mutex_t thread_mutex;
    rt_device_wrap_t *serial_device_wrap;
    volatile bool is_async_rx_failed;
    volatile bool is_async_tx_failed;
};

typedef struct async_transmit_context {
    usbh_serial_dev_t *dev;
    void *buffer; 
    size_t buffer_size;
    uint8_t retry_count;
} async_transmit_context_t;

typedef enum dev_usbh_serial_event {
    DEV_USBH_SERIAL_EVENT_UNKNOWN,
    DEV_USBH_SERIAL_EVENT_RX_COMPLETED,
    DEV_USBH_SERIAL_EVENT_RX_LOOP_COMPLETED,
    DEV_USBH_SERIAL_EVENT_RX_RETRY,
    DEV_USBH_SERIAL_EVENT_RX_CANCEL,
    DEV_USBH_SERIAL_EVENT_TX_COMPLETED,
    DEV_USBH_SERIAL_EVENT_TX_RETRY,
    DEV_USBH_SERIAL_EVENT_TX_CANCEL,
    DEV_USBH_SERIAL_EVENT_DISCONNECT,
} dev_usbh_serial_event_t;

typedef struct mq_msg {
    dev_usbh_serial_event_t event;
    async_transmit_context_t *transmit_context;
    uint32_t nbytes;
} mq_msg_t;

static inline rt_err_t usb_rx_lock(usbh_serial_dev_t *dev, rt_int32_t time) {
    return rt_sem_take(dev->rx_sem, time);
}

static inline rt_err_t usb_rx_unlock(usbh_serial_dev_t *dev) {
    if(dev->rx_sem->value > USBH_SERIAL_RT_SEM_MAX_VALUE) {
        LOG_W("usb_rx_unlock: repeated release!");
        return RT_EOK;
    }

    return rt_sem_release(dev->rx_sem);
}

static inline rt_err_t usb_tx_lock(usbh_serial_dev_t *dev, rt_int32_t time) {
    return rt_sem_take(dev->tx_sem, time);
}

static inline rt_err_t usb_tx_unlock(usbh_serial_dev_t *dev) {
    if(dev->tx_sem->value > USBH_SERIAL_RT_SEM_MAX_VALUE) {
        LOG_W("usb_tx_unlock: repeated release!");
        return RT_EOK;
    }

    return rt_sem_release(dev->tx_sem);
}

static void on_async_usb_read_error(async_transmit_context_t *context) {
    usbh_serial_dev_t *dev = context->dev;
    dev->is_async_rx_failed = true;
    rt_wqueue_wakeup_all(&(dev->parent.parent.wait_queue), (void*)POLLERR);
    usb_rx_unlock(dev);
    rt_free(context);
}

static void on_async_usb_write_error(async_transmit_context_t *context) {
    usbh_serial_dev_t *dev = context->dev;
    struct rt_serial_tx_dma* serial_tx_dma = (struct rt_serial_tx_dma*)(dev->parent.serial_tx);
    serial_tx_dma->activated = RT_FALSE;
    dev->is_async_tx_failed = true;
    rt_wqueue_wakeup_all(&(dev->parent.parent.wait_queue), (void*)POLLERR);
    usb_tx_unlock(dev);
    rt_free(context);
}

static void usb_read_complete_callback(void *arg, int nbytes) {
    LOG_D("usb_read_complete_callback: nbytes = %d", nbytes);

    async_transmit_context_t *context = arg;
    usbh_serial_dev_t *dev = context->dev;
    if (nbytes > 0) {
        dev_usbh_serial_event_t event;
        if (context->buffer == dev->rx_buffer) {
            event = DEV_USBH_SERIAL_EVENT_RX_LOOP_COMPLETED;
        }
        else {
            event = DEV_USBH_SERIAL_EVENT_RX_COMPLETED;
        }

        mq_msg_t msg = {
            .event = event,
            .transmit_context = context,
            .nbytes = nbytes
        };
        rt_err_t err = rt_mq_send(dev->mq, &msg, sizeof(mq_msg_t));
        if (err) {
            LOG_E("usb async read: mq send failed! err = %d", err);
            on_async_usb_read_error(context);
        }
    }
    else if (nbytes == 0 || nbytes == -USB_ERR_NAK) {
        mq_msg_t msg = {
            .event = DEV_USBH_SERIAL_EVENT_RX_RETRY,
            .nbytes = 0,
            .transmit_context = context
        };
        rt_err_t err = rt_mq_send(dev->mq, &msg, sizeof(mq_msg_t));

        if (err) {
            LOG_E("usb async read: mq send failed! err = %d", err);
            on_async_usb_read_error(context);
        }
    }
    else {
        on_async_usb_read_error(context);
        if (nbytes == -USB_ERR_SHUTDOWN) {
            LOG_I("usb async read: USB_ERR_SHUTDOWN");
        }
        else if (nbytes == -USB_ERR_IO) {
            LOG_E("usb async read: USB_ERR_IO");
        }
        else if (nbytes == -USB_ERR_STALL) {
            LOG_E("usb async read: USB_ERR_STALL");
        }
        else if (nbytes == -USB_ERR_BABBLE) {
            LOG_E("usb async read: USB_ERR_BABBLE");
        }
        else if (nbytes == -USB_ERR_DT) {
            LOG_E("usb async read: USB_ERR_DT");
        }
        else {
            LOG_E("usb async read: unknown error nbytes = %d", nbytes);
        }
    }
}

static void usb_write_complete_callback(void *arg, int nbytes) {
    async_transmit_context_t *context = arg;
    usbh_serial_dev_t *dev = context->dev;
    rt_err_t err;
    if (nbytes > 0) {
        mq_msg_t msg = {
            .event = DEV_USBH_SERIAL_EVENT_TX_COMPLETED,
            .nbytes = nbytes
        };
        rt_err_t err = rt_mq_send(dev->mq, &msg, sizeof(mq_msg_t));
        if (err) {
            LOG_E("usb async write: mq send failed! err = %d", err);
            on_async_usb_write_error(context);
        }
    }
    else if (nbytes == -USB_ERR_NAK) {
        mq_msg_t msg = {
            .event = DEV_USBH_SERIAL_EVENT_TX_RETRY,
            .transmit_context = context
        };
        rt_err_t err = rt_mq_send(dev->mq, &msg, sizeof(mq_msg_t));

        if (err) {
            LOG_E("usb async write: mq send failed! err = %d", err);
            on_async_usb_write_error(context);
        }
    }
    else {
        on_async_usb_write_error(context);
        if (nbytes == -USB_ERR_SHUTDOWN) {
            LOG_E("usb async write: USB_ERR_SHUTDOWN");
        }
        else if (nbytes == -USB_ERR_IO) {
            LOG_E("usb async write: USB_ERR_IO");
        }
        else if (nbytes == -USB_ERR_STALL) {
            LOG_E("usb async write: USB_ERR_STALL");
        }
        else if (nbytes == -USB_ERR_BABBLE) {
            LOG_E("usb async write: USB_ERR_BABBLE");
        }
        else if (nbytes == -USB_ERR_DT) {
            LOG_E("usb async write: USB_ERR_DT");
        }
        else {
            LOG_E("usb async write: unknown error");
        }
    }
}

static int usb_read(usbh_serial_dev_t *dev, uint8_t *buffer, uint32_t buflen, uint32_t timeout) {
    rt_err_t rt_err = usb_rx_lock(dev, rt_tick_from_millisecond(5000));
    if (rt_err != RT_EOK) {
        LOG_E("usb_read: usb_rx_lock failed! err = %d", rt_err);
        return -EBUSY;
    }

    dev->ops->set_bulk_out_transfer_buffer(dev->usbh_serial, buffer, buflen);
    int nbytes = dev->ops->bulk_in_transfer(dev->usbh_serial, timeout);

    usb_rx_unlock(dev);

    return nbytes;

}

static int usb_write(usbh_serial_dev_t *dev, uint8_t *buffer, uint32_t buflen, uint32_t timeout) {
    rt_err_t rt_err = usb_tx_lock(dev, rt_tick_from_millisecond(5000));
    if (rt_err != RT_EOK) {
        LOG_E("usb_write: usb_tx_lock failed! err = %d", rt_err);
        return -EBUSY;
    }

    dev->ops->set_bulk_out_transfer_buffer(dev->usbh_serial, buffer, buflen);
    int nbytes = dev->ops->bulk_out_transfer(dev->usbh_serial, timeout);

    usb_tx_unlock(dev);

    return nbytes;
}

static int async_usb_read(usbh_serial_dev_t *dev, uint8_t *buffer, uint32_t buflen) {
    rt_err_t rt_err = usb_rx_lock(dev, rt_tick_from_millisecond(5000));
    if (rt_err != RT_EOK) {
        LOG_E("async_usb_read: usb_rx_lock failed! err = %d", rt_err);
        return -EBUSY;
    }

    dev->is_async_rx_failed = false;

    dev->ops->set_bulk_in_transfer_buffer(dev->usbh_serial, buffer, buflen);
    async_transmit_context_t *context = rt_malloc(sizeof(async_transmit_context_t));
    if (context == RT_NULL) {
        usb_rx_unlock(dev);
        return -ENOMEM;
    }
    rt_memset(context, 0, sizeof(async_transmit_context_t));
    context->dev = dev;
    context->buffer = buffer;
    context->buffer_size = buflen;
    int err = dev->ops->async_bulk_in_transfer(dev->usbh_serial, usb_read_complete_callback, context);
    if (err) {
        LOG_E("async_usb_read: async_bulk_in_transfer failed! err = %d", err);
         usb_rx_unlock(dev);
    }

    return err;
}

static int async_usb_write(usbh_serial_dev_t *dev, uint8_t *buffer, uint32_t buflen) {
    rt_err_t rt_err = usb_tx_lock(dev, rt_tick_from_millisecond(5000));
    if (rt_err != RT_EOK) {
        LOG_E("async_usb_write: usb_tx_lock failed! err = %d", rt_err);
        return -EBUSY;
    }

    dev->is_async_tx_failed = false;

    dev->ops->set_bulk_out_transfer_buffer(dev->usbh_serial, buffer, buflen);
    async_transmit_context_t *context = rt_malloc(sizeof(async_transmit_context_t));
    if (context == RT_NULL) {
        usb_tx_unlock(dev);
        return -ENOMEM;
    }
    rt_memset(context, 0, sizeof(async_transmit_context_t));
    context->dev = dev;
    context->buffer = buffer;
    context->buffer_size = buflen;

    int err = dev->ops->async_bulk_out_transfer(dev->usbh_serial, usb_write_complete_callback, context);
    if (err) {
        usb_tx_unlock(dev);
    }

    return err;
}

static void handle_mq_msg(mq_msg_t *msg, usbh_serial_dev_t *dev) {
    async_transmit_context_t *transmit_context = msg->transmit_context;
    rt_serial_t *serial_dev = &dev->parent;

    switch (msg->event) {
        case DEV_USBH_SERIAL_EVENT_RX_COMPLETED: {
            rt_serial_put_rxfifo(serial_dev, dev->rx_buffer, msg->nbytes);
            rt_hw_serial_isr(serial_dev, RT_SERIAL_EVENT_RX_DMADONE);
            rt_free(transmit_context);
            usb_rx_unlock(dev);
        } break;
        case DEV_USBH_SERIAL_EVENT_RX_LOOP_COMPLETED: {
            LOG_D("EVENT_RX_LOOP_COMPLETED: nbytes = %d", msg->nbytes);
            rt_serial_put_rxfifo(serial_dev, dev->rx_buffer, msg->nbytes);
            rt_hw_serial_isr(serial_dev, RT_SERIAL_EVENT_RX_DMADONE);
            usb_rx_unlock(dev);
            rt_thread_yield();
            rt_err_t rt_err = usb_rx_lock(dev, rt_tick_from_millisecond(5000));
            if (rt_err != RT_EOK) {
                LOG_E("EVENT_RX_LOOP_COMPLETED: usb_rx_lock failed! err = %d", rt_err);
                on_async_usb_read_error(transmit_context);
                break;
            }

            int err = dev->ops->async_bulk_in_transfer(dev->usbh_serial, usb_read_complete_callback, transmit_context);
            if (err) {
                LOG_E("EVENT_RX_LOOP_COMPLETED: async_bulk_in_transfer failed! err = %d", err);
                on_async_usb_read_error(transmit_context);
            }
        } break;
        case DEV_USBH_SERIAL_EVENT_RX_RETRY: {
            uint8_t retry_count = transmit_context->retry_count;
            LOG_W("EVENT_RX_RETRY: retry count = %d", retry_count);
            rt_thread_mdelay(rt_tick_from_millisecond(100 * retry_count));
            retry_count++;
            if (retry_count > USBH_SERIAL_ASYNC_TRANSMIT_MAX_RETRY) {
                LOG_E("EVENT_RX_RETRY: exceeded the retry limit!");
                on_async_usb_read_error(transmit_context);
                break;
            }
            transmit_context->retry_count = retry_count;
            int err = dev->ops->async_bulk_in_transfer(dev->usbh_serial, usb_read_complete_callback, transmit_context);
            if (err) {
                LOG_E("EVENT_RX_RETRY: async_bulk_in_transfer failed! err = %d", err);
                on_async_usb_read_error(transmit_context);
            }
        } break;
        case DEV_USBH_SERIAL_EVENT_RX_CANCEL: {
            // TODO
            LOG_E("EVENT_RX_CANCEL");
            on_async_usb_read_error(transmit_context);
        } break;
        case DEV_USBH_SERIAL_EVENT_TX_COMPLETED: {
            rt_hw_serial_isr(serial_dev, RT_SERIAL_EVENT_TX_DMADONE);
            rt_free(transmit_context);
            usb_tx_unlock(dev);
        } break;
        case DEV_USBH_SERIAL_EVENT_TX_RETRY: {
            uint8_t retry_count = transmit_context->retry_count;
            LOG_W("EVENT_TX_RETRY: retry count = %d", retry_count);
            rt_thread_mdelay(rt_tick_from_millisecond(100 * retry_count));
            retry_count++;
            if (retry_count > USBH_SERIAL_ASYNC_TRANSMIT_MAX_RETRY) {
                LOG_E("EVENT_TX_RETRY: exceeded the retry limit!");
                on_async_usb_write_error(transmit_context);
                break;
            }
            transmit_context->retry_count = retry_count;
            int err = dev->ops->async_bulk_out_transfer(dev->usbh_serial, usb_write_complete_callback, transmit_context);
            if (err) {
                LOG_E("EVENT_TX_RETRY: async_bulk_in_transfer failed! err = %d", err);
                on_async_usb_write_error(transmit_context);
            }
        } break;
        case DEV_USBH_SERIAL_EVENT_TX_CANCEL: {
            struct rt_serial_tx_dma* serial_tx_dma = (struct rt_serial_tx_dma*)(dev->parent.serial_tx);
            serial_tx_dma->activated = RT_FALSE;
            rt_sem_release(&serial_tx_dma->tx_done);
            LOG_E("EVENT_TX_CANCEL");
            on_async_usb_write_error(transmit_context);
        } break;
        case DEV_USBH_SERIAL_EVENT_DISCONNECT: {
            LOG_I("EVENT_DISCONNECT");
            mq_msg_t remaining_msg;
            while(rt_mq_recv(dev->mq, &remaining_msg, sizeof(mq_msg_t), 0) == RT_EOK) {
                LOG_D("EVENT_DISCONNECT: rt_mq_recv ok");
                if (msg->event == DEV_USBH_SERIAL_EVENT_DISCONNECT) {
                    continue;
                }
                handle_mq_msg(&remaining_msg, dev);
            }
            rt_wqueue_wakeup_all(&(serial_dev->parent.wait_queue), (void*)POLLHUP);
        } break;
        default:
            LOG_E("DEV_USBH_SERIAL_EVENT_UNKNOWN");
            if (transmit_context) {
                rt_free(transmit_context);
            }
            break;
    }
}

static void thread_entry(void *arg) {
    usbh_serial_dev_t *dev = arg;

    rt_mutex_take(dev->thread_mutex, RT_WAITING_FOREVER);
    dev->is_thread_running = true;

    mq_msg_t msg;
    while (dev->is_thread_running) {
        rt_err_t err = rt_mq_recv(dev->mq, &msg, sizeof(mq_msg_t), RT_WAITING_FOREVER);
        if (err == RT_EOK) {
            handle_mq_msg(&msg, dev);
        }
        else {
            LOG_E("rt_mq_recv failed! err = %d", err);
        }
    }

    rt_mutex_release(dev->thread_mutex);
}

static rt_bool_t on_fops_open(struct dfs_fd *fd, int *ret_ptr, void *user_data) {
    usbh_serial_dev_t *dev = user_data;
    if (!dev->is_thread_running) {
        LOG_I("on_fops_open: !dev->is_thread_running");
        *ret_ptr = -ENODEV;
        return true;
    }

    return false;
}

static rt_bool_t on_fops_close(struct dfs_fd *fd, int *ret_ptr, void *user_data) {
    usbh_serial_dev_t *dev = user_data;
    if (!dev->is_thread_running) {
        LOG_I("on_fops_close: !dev->is_thread_running");
        *ret_ptr = 0;
        return true;
    }

    return false;
}

static rt_bool_t on_fops_ioctl(struct dfs_fd *fd, int *cmd_ptr, void **args_ptr, int *ret_ptr, void *user_data) {
    usbh_serial_dev_t *dev = user_data;
    if (!dev->is_thread_running) {
        LOG_I("on_fops_ioctl: !dev->is_thread_running");
        *ret_ptr = -ENODEV;
        return true;
    }

    return false;
}

static rt_bool_t on_fops_read(struct dfs_fd *fd, void **buf_ptr, size_t *count_ptr, int *ret_ptr, void *user_data) {
    usbh_serial_dev_t *dev = user_data;
    if (!dev->is_thread_running) {
        LOG_I("on_fops_read: !dev->is_thread_running");
        *ret_ptr = -ENODEV;
        return true;
    }

    return false;
}

static rt_bool_t on_fops_write(struct dfs_fd *fd, const void **buf_ptr, size_t *count_ptr, int *ret_ptr, void *user_data) {
    usbh_serial_dev_t *dev = user_data;
    if (!dev->is_thread_running) {
        *ret_ptr = -ENODEV;
        return true;
    }

    return false;
}


static rt_size_t _serial_fifo_calc_recved_len(struct rt_serial_device *serial)
{
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *) serial->serial_rx;

    RT_ASSERT(rx_fifo != RT_NULL);

    if (rx_fifo->put_index == rx_fifo->get_index)
    {
        return (rx_fifo->is_full == RT_FALSE ? 0 : serial->config.bufsz);
    }
    else
    {
        if (rx_fifo->put_index > rx_fifo->get_index)
        {
            return rx_fifo->put_index - rx_fifo->get_index;
        }
        else
        {
            return serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index);
        }
    }
}

static rt_bool_t on_fops_poll(struct dfs_fd *wrap_fd, struct rt_pollreq **req_ptr, int *ret_ptr, void *user_data) {
    struct rt_pollreq *req = *req_ptr;
    usbh_serial_dev_t *dev = user_data;
    rt_device_wrap_t *device_wrap = wrap_fd->fnode->data;
    rt_device_t device = rt_device_wrap_get_origin_device(device_wrap);

    rt_poll_add(&(device->wait_queue), req);

    int mask = 0;

    if(!dev->is_thread_running) {
        LOG_D("on_fops_poll: !dev->is_thread_running");
        mask |= POLLHUP;
    }
    else {
        if (dev->is_async_rx_failed) {
            LOG_D("on_fops_poll: dev->is_async_rx_failed");
            mask |= POLLERR;
        }

        if (dev->is_async_tx_failed) {
            LOG_D("on_fops_poll: dev->is_async_tx_failed");
            mask |= POLLERR;
        }

        uint32_t flags = wrap_fd->flags & O_ACCMODE;
        if (flags == O_RDONLY || flags == O_RDWR) {
            rt_serial_t *serial = (rt_serial_t *)device;
            struct rt_serial_rx_fifo *rx_fifo = serial->serial_rx;
            if ((rx_fifo->get_index != rx_fifo->put_index) 
                || (rx_fifo->get_index == rx_fifo->put_index && rx_fifo->is_full == RT_TRUE)) {
                mask |= POLLIN | POLLRDNORM;
                LOG_D("on_fops_poll: POLLIN!");
            }
        }
    }

    *ret_ptr = mask;

    return true;
}

static rt_device_wrap_fops_interceptor_t serial_dev_fops_interceptor = {
    .on_fops_open = on_fops_open,
    .on_fops_close = NULL,
    .on_fops_ioctl = on_fops_ioctl,
    .on_fops_read = on_fops_read,
    .on_fops_write = on_fops_write,
    .on_fops_flush = NULL,
    .on_fops_lseek = NULL,
    .on_fops_poll = on_fops_poll,
};

static rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg) {
    LOG_D("uart_configure: enter");
    usbh_serial_dev_t *dev = serial->parent.user_data;

    struct cdc_line_coding cdc_line_coding = {
        .dwDTERate = cfg->baud_rate,
        .bDataBits = cfg->data_bits,
        .bCharFormat = cfg->stop_bits,
        .bParityType = cfg->parity
    };

    rt_err_t rt_err = rt_mutex_take(dev->mutex, rt_tick_from_millisecond(5000));
    if (rt_err != RT_EOK) {
        LOG_E("uart_configure: rt_mutex_take failed! err = %d", rt_err);
        return rt_err;
    }

    int op_ret = dev->ops->set_line_coding(dev->usbh_serial, &cdc_line_coding);
    if (op_ret < 0) {
        LOG_E("uart_configure: set_line_coding failed! err = %d", op_ret);
        rt_err = RT_EIO;
    }
    else {
        rt_err = RT_EOK;
    }
    rt_mutex_release(dev->mutex);

    return rt_err;
}

static rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg) {
    int ret = RT_EOK;

    usbh_serial_dev_t *dev = serial->parent.user_data;

    switch (cmd) {
        case RT_DEVICE_CTRL_CONFIG: 
        case RT_DEVICE_CTRL_SET_INT: {
            dev->is_open = true;
            rt_err_t rt_err = rt_mutex_take(dev->mutex, rt_tick_from_millisecond(5000));
            if (rt_err != RT_EOK) {
                LOG_E("uart_control: rt_mutex_take failed! err = %d", rt_err);
                ret = RT_EBUSY;
                break;
            }
            int ops_ret = dev->ops->set_line_state(dev->usbh_serial, true, true);
            rt_mutex_release(dev->mutex);
            if (ops_ret < 0) {
                LOG_E("uart_control: set_line_state failed! err = %d", ops_ret);
                ret = RT_EIO;
                break;
            }
            if (arg == (void *)RT_DEVICE_FLAG_DMA_RX) {
                int err = async_usb_read(dev, dev->rx_buffer, dev->rx_buffer_size);
                if (err) {
                    LOG_E("uart_control: async_usb_read failed! err = %d", err);
                }
            }
        } break;
        case RT_DEVICE_CTRL_CLR_INT: {
            if (arg == (void *)RT_DEVICE_FLAG_INT_RX) {
                // TODO
            }
            else if (arg == (void *)RT_DEVICE_FLAG_DMA_RX) {
                dev->ops->cancel_bulk_in_transfer(dev->usbh_serial);
            }
            if (arg == (void *)RT_DEVICE_FLAG_INT_TX) {
                // TODO
            }
            else if (arg == (void *)RT_DEVICE_FLAG_DMA_TX) {
               dev->ops->cancel_bulk_out_transfer(dev->usbh_serial);
            }
            else {
                // TODO
            }
        } break;
        case RT_DEVICE_CTRL_CLOSE: {
            dev->is_open = false;
            if (dev->is_thread_running) {
                rt_err_t rt_err = rt_mutex_take(dev->mutex, rt_tick_from_millisecond(5000));
                if (rt_err != RT_EOK) {
                    LOG_E("uart_control: rt_mutex_take failed! err = %d", rt_err);
                    ret = RT_EBUSY;
                    break;
                }
                int op_ret = dev->ops->set_line_state(dev->usbh_serial, false, false);
                rt_mutex_release(dev->mutex);
                if (op_ret < 0) {
                    LOG_W("uart_control: set_line_state failed! err = %d", op_ret);
                }
            }
            else {
                LOG_I("destroy");
                rt_device_wrap_unregister_fops_interceptor(dev->serial_device_wrap, &serial_dev_fops_interceptor);
                rt_device_wrap_destroy(dev->serial_device_wrap);
                rt_free(dev);
            }
        } break;
        case UART_IOCTL_SET_CONFIG: {
            LOG_D("uart_control: UART_IOCTL_SET_CONFIG");
            if (arg) {
                struct serial_configure _config;

                if (lwp_in_user_space(arg)) {
                    if (sizeof(_config) != lwp_get_from_user(&_config, arg, sizeof(_config))) {
                        USB_LOG_ERR("lwp get error size\n");
                        return -RT_EINVAL;
                    }
                } else {
                    rt_memcpy(&_config, arg, sizeof(_config));
                }

                if (0x00 == _config.bufsz) {
                    // Use default buffer size if not specified
                    _config.bufsz = serial->config.bufsz;
                } else {
                    if (_config.bufsz != serial->config.bufsz && serial->parent.ref_count) {
                        /*can not change buffer size*/
                        return RT_EBUSY;
                    }
                }

                /* set serial configure */
                rt_memcpy(&serial->config, &_config, sizeof(serial->config));

                if (serial->parent.ref_count) {
                    /* serial device has been opened, to configure it */
                    uart_configure(serial, &serial->config);
                }
            }
        } break;
        case UART_IOCTL_GET_CONFIG: {
            LOG_D("uart_control: UART_IOCTL_GET_CONFIG");
            if (!arg) {
                USB_LOG_ERR("arg is NULL\n");
                return -RT_EINVAL;
            }

            if (lwp_in_user_space(arg)) {
                if (sizeof(serial->config) != lwp_put_to_user(arg, &serial->config, sizeof(serial->config))) {
                    USB_LOG_ERR("lwp put error size\n");
                    return -RT_EINVAL;
                }
            } else {
                rt_memcpy(arg, &serial->config, sizeof(serial->config));
            }
        } break;
        case UART_IOCTL_SEND_BREAK: {
            return RT_EOK;
        } break;
        case UART_IOCTL_GET_DTR: {
            if(!arg) {
                USB_LOG_ERR("arg is NULL\n");
                return -RT_EINVAL;
            }

            if(lwp_in_user_space(arg)) {
                if (sizeof(bool) != lwp_put_to_user(arg, &dev->is_open, sizeof(bool))) {
                    USB_LOG_ERR("lwp put error size\n");
                    ret = -RT_EINVAL;
                }
            } else {
                *((bool *)arg) = dev->is_open;
            }
        } break;
        default:
            USB_LOG_ERR("%s: unsupport cmd %d\n", __func__, cmd);
            ret = RT_EINVAL;
            break;
        }

    return ret;
}

static rt_size_t uart_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction) {
    usbh_serial_dev_t *dev = serial->parent.user_data;

    if (direction == RT_SERIAL_DMA_TX) {
        int err = async_usb_write(dev, buf, size);
        return size;
    }
    else if (direction == RT_SERIAL_DMA_RX) {
        int err = async_usb_read(dev, buf, size);
        return size;
    }
    else {
        LOG_W("No way, not happening right now. direction = %d", direction);
    }

    return 0;
}

static const struct rt_uart_ops uart_ops = {
    .configure = uart_configure,
    .control = uart_control,
    .dma_transmit = uart_dma_transmit
};


usbh_serial_dev_t* dev_usbh_serial_create(void *usbh_serial, dev_usbh_serial_ops_t *ops, size_t ring_buffer_size) {
    const char *dev_path = ops->get_dev_path(usbh_serial);
    const char *dev_name = dev_path + rt_strlen("/dev/");

    if(rt_device_find(dev_name)) {
        LOG_E("%s already exists", dev_name);
        rt_set_errno(EINVAL);
        return RT_NULL;
    }

    usbh_serial_dev_t *dev = rt_malloc(sizeof(usbh_serial_dev_t));
    if (dev == RT_NULL) {
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    rt_memset(dev, 0, sizeof(usbh_serial_dev_t));

    size_t aligned_ring_buffer_size = RT_ALIGN(ring_buffer_size, RT_ALIGN_SIZE);

    dev->usbh_serial = usbh_serial;

    dev->mutex = rt_mutex_create(dev_name, RT_IPC_FLAG_PRIO);
    if (dev->mutex == RT_NULL) {
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    char name_buffer[RT_NAME_MAX];

    rt_snprintf(name_buffer, sizeof(name_buffer), "%s_rx", dev_name);
    dev->rx_sem = rt_sem_create(name_buffer, 1, RT_IPC_FLAG_PRIO);
    if (dev->rx_sem == RT_NULL) {
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    rt_snprintf(name_buffer, sizeof(name_buffer), "%s_tx", dev_name);
    dev->tx_sem = rt_sem_create(name_buffer, 1, RT_IPC_FLAG_PRIO);
    if (dev->tx_sem == RT_NULL) {
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    dev->mq = rt_mq_create(dev_name, sizeof(mq_msg_t), 20, RT_IPC_FLAG_PRIO);
    if (dev->mq == RT_NULL) {
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }


    dev->thread_mutex = rt_mutex_create(dev_name, RT_IPC_FLAG_PRIO);
    if (dev->thread_mutex == RT_NULL) {
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    rt_snprintf(name_buffer, sizeof(name_buffer), "%s_thread", dev_name);
    dev->thread = rt_thread_create(name_buffer, thread_entry, dev, 3 * 1024, 15, 10);
    if (dev->thread == RT_NULL) {
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    dev->ops = ops;
    dev->rx_buffer_size = aligned_ring_buffer_size / 4;
    dev->rx_buffer = rt_malloc(dev->rx_buffer_size);
    if (dev->rx_buffer == RT_NULL) {
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    rt_serial_t *serial_device = &dev->parent;
    serial_device->ops = &uart_ops;
    struct serial_configure def_cfg = RT_SERIAL_CONFIG_DEFAULT;
    serial_device->config = def_cfg;
    serial_device->config.bufsz = aligned_ring_buffer_size;

    rt_snprintf(name_buffer, sizeof(name_buffer), "_%s", dev_name);

    rt_err_t err = rt_hw_serial_register(serial_device
        , name_buffer 
        , RT_DEVICE_FLAG_RDWR 
            | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX
            // TODO serial.c needs to be improved.
            // | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX
        , dev);

    if (err != RT_EOK) {
        LOG_E("usbh_serial_create: register %s failed! err = %d", name_buffer, err);
        rt_set_errno(EPERM);
        dev_usbh_serial_destroy(dev);
        return RT_NULL;
    }

    dev->serial_device_wrap = rt_device_wrap_create(&serial_device->parent);

    if (dev->serial_device_wrap == RT_NULL) {
        LOG_E("usbh_serial_create: create serial device wrap failed! err = %d", errno);
        dev_usbh_serial_destroy(dev);
        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    rt_device_wrap_set_user_data(dev->serial_device_wrap, dev);

    err = rt_device_wrap_register_fops_interceptor(dev->serial_device_wrap, &serial_dev_fops_interceptor);
    if (err != RT_EOK) {
        LOG_E("usbh_serial_create: register interceptor for serial device failed! err = %d", err);
        dev_usbh_serial_destroy(dev);

        rt_set_errno(ENOMEM);
        return RT_NULL;
    }

    err = rt_device_wrap_register(dev->serial_device_wrap, dev_name);
    if (err != RT_EOK) {
        LOG_E("usbh_serial_create: register %s failed! err = %d", dev_name, err);
        rt_set_errno(EPERM);
        dev_usbh_serial_destroy(dev);
        return RT_NULL;
    }

    LOG_I("register %s", dev_name);

    rt_thread_startup(dev->thread);

    return dev;
}

void dev_usbh_serial_destroy(usbh_serial_dev_t* dev) {
    if (dev == RT_NULL) {
        return;
    }

    if (dev->thread) {
        dev->is_thread_running = false;
        dev->ops->cancel_bulk_in_transfer(dev->usbh_serial);
        dev->ops->cancel_bulk_out_transfer(dev->usbh_serial);
        mq_msg_t msg = {
            .event = DEV_USBH_SERIAL_EVENT_DISCONNECT,
            .transmit_context = NULL,
            .nbytes = 0
        };
        rt_err_t err = rt_mq_send(dev->mq, &msg, sizeof(mq_msg_t));
        if (err) {
            // TOOD
            LOG_E("dev_usbh_serial_destroy: mq send failed! err = %d", err);
        }
        else {
            err = rt_mutex_take(dev->thread_mutex, rt_tick_from_millisecond(5000));
        }
        if (err) {
            LOG_E("elegantly stop thread failed! err = %d", err);
            rt_thread_delete(dev->thread);
        }
        rt_mutex_release(dev->thread_mutex);
    }

    if (dev->thread_mutex) {
        rt_mutex_delete(dev->thread_mutex);
    }

    if (dev->mq) {
        rt_mq_delete(dev->mq);
    }

    if (dev->rx_buffer) {
        rt_free(dev->rx_buffer);
    }

    if (dev->mutex) {
        rt_mutex_delete(dev->mutex);
    }

    if (dev->rx_sem) {
        rt_sem_delete(dev->rx_sem);
    }

    if (dev->tx_sem) {
        rt_sem_delete(dev->tx_sem);
    }

    rt_serial_t *serial_device = &dev->parent;

    // for(rt_uint8_t i = serial_device->parent.ref_count; i > 0; i++) {
    //     rt_device_close(&serial_device->parent);
    // }

    if(rt_object_get_type(&serial_device->parent.parent) == RT_Object_Class_Device) {
        if (rt_object_is_systemobject(&serial_device->parent.parent)) {
            rt_device_unregister(&serial_device->parent);
            // patch: When unregistering a device, maybe we shouldn't block it from being used.
            serial_device->parent.parent.type = RT_Object_Class_Device | RT_Object_Class_Static;
        }
    }

    if (dev->serial_device_wrap) {
        rt_err_t err = rt_device_wrap_unregister(dev->serial_device_wrap);
        if (err == RT_EOK) {
            LOG_I("unregister %s\n", rt_device_wrap_get_name(dev->serial_device_wrap));
            // patch: When unregistering a device, maybe we shouldn't block it from being used.
            rt_device_wrap_get_parent(dev->serial_device_wrap)->parent.type = RT_Object_Class_Device | RT_Object_Class_Static;
        }
        // rt_device_wrap_destroy(dev->serial_device_wrap);
    }

    // rt_free(dev);
}
