#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "usbh_core.h"
#include "usbh_hid.h"

#ifdef RT_USING_POSIX
#include <dfs_file.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#endif

#define HID_RX_BUFSIZE 512

#define EV_SYN      0x00  /* Synchronization event */
#define EV_KEY      0x01  /* Key press/release event */

/* Synchronization event codes */
#define SYN_REPORT  0     /* Synchronize: report event frame boundary */

/* Key event values */
#define KEY_RELEASED    0  /* Key released */
#define KEY_PRESSED     1  /* Key pressed */

/* HID Keyboard Event - Exposed to userspace */
struct hid_keyboard_event
{
    uint16_t type;      /* Event type (EV_SYN, EV_KEY) */
    uint16_t code;      /* Key code (Linux keycode) */
    uint32_t value;     /* KEY_PRESSED or KEY_RELEASED */
};

static const unsigned char usb_kbd_keycode[256] = {
      0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
     50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
      4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
     27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
     65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
    105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
     72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
    191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
    115,114,  0,  0,  0,121,  0, 89, 93,124, 92, 94, 95,  0,  0,  0,
    122,123, 90, 91, 85,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
    150,158,159,128,136,177,178,176,142,152,173,140
};

/* HID Device Receive FIFO */
struct rt_hid_rx_fifo
{
    struct rt_ringbuffer rb;
    /* software fifo */
    rt_uint8_t buffer[];
};

/* HID Device Structure */
struct rt_hid_device
{
    struct rt_device parent;
    struct usbh_hid *hid_class;
    struct rt_hid_rx_fifo *rx_fifo;
    rt_size_t rx_bufsz;

    /* Last state for key press/release detection */
    rt_uint8_t last_report[8];
};

/* USB HID buffer */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t hid_buffer[128];

/* Global HID device instance */
static struct rt_hid_device hid_dev;

/* Forward declarations */
static void usbh_hid_callback(void *arg, int nbytes);

#ifdef RT_USING_POSIX
/* fops for HID */
static rt_err_t hid_fops_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_wqueue_wakeup(&(dev->wait_queue), (void*)POLLIN);
    return RT_EOK;
}

static int hid_fops_open(struct dfs_fd *fd)
{
    rt_err_t ret = 0;
    rt_uint16_t flags = 0;
    rt_device_t device;
    struct rt_hid_device *hid;

    device = (rt_device_t)fd->fnode->data;
    RT_ASSERT(device != RT_NULL);

    hid = (struct rt_hid_device *)device;

    if (NULL == hid->hid_class) {
        return -ENODEV;
    }

    if ((fd->flags & O_ACCMODE) != O_WRONLY)
        rt_device_set_rx_indicate(device, hid_fops_rx_ind);

    ret = rt_device_open(device, flags);
    if (ret == RT_EOK) return 0;

    return ret;
}

static int hid_fops_close(struct dfs_fd *fd)
{
    rt_device_t device;
    struct rt_hid_device *hid;

    device = (rt_device_t)fd->fnode->data;

    hid = (struct rt_hid_device *)device;

    if (NULL == hid->hid_class) {
        return 0;
    }

    rt_device_set_rx_indicate(device, RT_NULL);
    rt_device_close(device);

    return 0;
}

static int hid_fops_ioctl(struct dfs_fd *fd, int cmd, void *args)
{
    rt_device_t device;
    struct rt_hid_device *hid;

    device = (rt_device_t)fd->fnode->data;
    hid = device;

    if (NULL == hid->hid_class) {
        return -ENODEV;
    }

    return rt_device_control(device, cmd, args);
}

static int hid_fops_read(struct dfs_fd *fd, void *buf, size_t count)
{
    int size = 0;
    rt_device_t device;
    struct rt_hid_device *hid;

    device = (rt_device_t)fd->fnode->data;
    hid = device;

    if (NULL == hid->hid_class) {
        return -ENODEV;
    }

    do {
        size = rt_device_read(device, -1, buf, count);
        if (size <= 0) {
            if (fd->flags & O_NONBLOCK) {
                size = -EAGAIN;
                break;
            }

            rt_wqueue_wait(&(device->wait_queue), 0, RT_WAITING_FOREVER);
        }
    } while (size <= 0);

    return size;
}

static int hid_fops_write(struct dfs_fd *fd, const void *buf, size_t count)
{
    /* HID keyboard is read-only */
    return 0;
}

static int hid_fops_poll(struct dfs_fd *fd, struct rt_pollreq *req)
{
    int mask = 0;
    int flags = 0;
    rt_device_t device;
    struct rt_hid_device *hid;

    device = (rt_device_t)fd->fnode->data;
    RT_ASSERT(device != RT_NULL);

    hid = (struct rt_hid_device *)device;

    if (NULL == hid->hid_class) {
        return -ENODEV;
    }

    flags = fd->flags & O_ACCMODE;
    if (flags == O_RDONLY || flags == O_RDWR) {
        rt_base_t level;
        struct rt_hid_rx_fifo *rx_fifo;

        rt_poll_add(&(device->wait_queue), req);

        rx_fifo = hid->rx_fifo;
        if (rx_fifo) {
            level = rt_hw_interrupt_disable();
            if (rt_ringbuffer_data_len(&rx_fifo->rb))
                mask |= POLLIN;
            rt_hw_interrupt_enable(level);
        }
    }

    return mask;
}

const static struct dfs_file_ops _hid_fops =
{
    hid_fops_open,
    hid_fops_close,
    hid_fops_ioctl,
    hid_fops_read,
    hid_fops_write,
    RT_NULL, /* flush */
    RT_NULL, /* lseek */
    RT_NULL, /* getdents */
    hid_fops_poll,
};
#endif /* RT_USING_POSIX */

/* Helper function to put event into ringbuffer */
static void hid_put_event(struct rt_hid_device *hid, uint16_t code, uint32_t value)
{
    struct hid_keyboard_event event;
    struct rt_hid_rx_fifo *rx_fifo = hid->rx_fifo;
    rt_base_t level;
    rt_size_t put_len;

    if (rx_fifo == RT_NULL)
        return;

    /* Determine event type based on code */
    if (code == SYN_REPORT) {
        event.type = EV_SYN;
        event.code = SYN_REPORT;
        event.value = 0;
    } else {
        event.type = EV_KEY;
        event.code = code;
        event.value = value;
    }

    level = rt_hw_interrupt_disable();
    put_len = rt_ringbuffer_put(&(rx_fifo->rb), (uint8_t*)&event, sizeof(event));
    rt_hw_interrupt_enable(level);

    /* Notify readers when SYN_REPORT is sent (end of event frame) */
    if (put_len > 0 && code == SYN_REPORT) {
        if (hid->parent.rx_indicate != RT_NULL) {
            hid->parent.rx_indicate(&hid->parent, put_len);
        }
    }
}

/* Process HID keyboard data - Generate key events (Linux-style) */
static void hid_process_keyboard(struct rt_hid_device *hid, const rt_uint8_t *data, int nbytes)
{
    struct usb_hid_kbd_report *report = (struct usb_hid_kbd_report *)data;
    int i;

    if (nbytes < sizeof(struct usb_hid_kbd_report))
        return;

    /* Process modifier keys (bits 0-7 in modifier byte)
     * HID Usage 224-231 maps to usb_kbd_keycode[224-231]
     * This handles Ctrl, Shift, Alt, Meta keys
     */
    for (i = 0; i < 8; i++) {
        unsigned char keycode = usb_kbd_keycode[i + 224];
        if (keycode) {
            /* Check if this modifier bit changed */
            int old_state = (hid->last_report[0] >> i) & 1;
            int new_state = (report->modifier >> i) & 1;

            if (old_state != new_state) {
                hid_put_event(hid, keycode, new_state);
            }
        }
    }

    /* Process released keys (in old[2-7] but not in new[2-7])
     * old[i] contains HID Usage ID, need to check if it's still in new report
     */
    for (i = 2; i < 8; i++) {
        if (hid->last_report[i] > 3) {
            /* Check if this key is still pressed (exists in new report) */
            int j;
            rt_bool_t found = RT_FALSE;

            for (j = 0; j < 6; j++) {
                if (report->key[j] == hid->last_report[i]) {
                    found = RT_TRUE;
                    break;
                }
            }

            if (!found) {
                /* Key was released */
                unsigned char keycode = usb_kbd_keycode[hid->last_report[i]];
                if (keycode) {
                    hid_put_event(hid, keycode, KEY_RELEASED);
                }
            }
        }
    }

    /* Process pressed keys (in new[2-7] but not in old[2-7]) */
    for (i = 0; i < 6; i++) {
        if (report->key[i] > 3) {
            /* Check if this is a new key press (not in old report) */
            int j;
            rt_bool_t found = RT_FALSE;

            for (j = 2; j < 8; j++) {
                if (hid->last_report[j] == report->key[i]) {
                    found = RT_TRUE;
                    break;
                }
            }

            if (!found) {
                /* New key pressed */
                unsigned char keycode = usb_kbd_keycode[report->key[i]];
                if (keycode) {
                    hid_put_event(hid, keycode, KEY_PRESSED);
                }
            }
        }
    }

    /* Send SYN_REPORT to indicate end of event frame */
    hid_put_event(hid, SYN_REPORT, 0);

    /* Save current state for next comparison */
    rt_memcpy(hid->last_report, data, 8);
}

/* USB HID callback */
void usbh_hid_callback(void *arg, int nbytes)
{
    struct usbh_hid *hid_class = (struct usbh_hid *)arg;

    if (nbytes > 0) {
        /* Process HID data */
        hid_process_keyboard(&hid_dev, hid_buffer, nbytes);

        /* Re-submit URB for next data */
        usbh_submit_urb(&hid_class->intin_urb);
    } else if (nbytes == -USB_ERR_NAK) {
        /* for dwc2 */
        usbh_submit_urb(&hid_class->intin_urb);
    }
}

/* RT-Thread Device Interface */

/* HID device init */
static rt_err_t rt_hid_init(struct rt_device *dev)
{
    struct rt_hid_device *hid;

    RT_ASSERT(dev != RT_NULL);
    hid = (struct rt_hid_device *)dev;

    /* Initialize rx FIFO */
    hid->rx_fifo = RT_NULL;
    rt_memset(hid->last_report, 0, sizeof(hid->last_report));

    return RT_EOK;
}

/* HID device open */
static rt_err_t rt_hid_open(struct rt_device *dev, rt_uint16_t oflag)
{
    struct rt_hid_device *hid;
    struct rt_hid_rx_fifo *rx_fifo;

    RT_ASSERT(dev != RT_NULL);
    hid = (struct rt_hid_device *)dev;

    /* Allocate RX FIFO if not allocated */
    if (hid->rx_fifo == RT_NULL) {
        rx_fifo = (struct rt_hid_rx_fifo *)rt_malloc(sizeof(struct rt_hid_rx_fifo) + hid->rx_bufsz);
        if (rx_fifo == RT_NULL) {
            USB_LOG_ERR("Failed to allocate HID RX FIFO\n");
            return -RT_ENOMEM;
        }

        rt_ringbuffer_init(&(rx_fifo->rb), rx_fifo->buffer, hid->rx_bufsz);
        hid->rx_fifo = rx_fifo;
    }

    return RT_EOK;
}

/* HID device close */
static rt_err_t rt_hid_close(struct rt_device *dev)
{
    struct rt_hid_device *hid;

    RT_ASSERT(dev != RT_NULL);
    hid = (struct rt_hid_device *)dev;

    /* Free RX FIFO if this is the last user */
    if (dev->ref_count <= 1 && hid->rx_fifo != RT_NULL) {
        rt_free(hid->rx_fifo);
        hid->rx_fifo = RT_NULL;
    }

    return RT_EOK;
}

/* HID device read */
static rt_size_t rt_hid_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct rt_hid_device *hid;
    struct rt_hid_rx_fifo *rx_fifo;
    rt_size_t recv_len;
    rt_base_t level;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    hid = (struct rt_hid_device *)dev;
    rx_fifo = hid->rx_fifo;

    if (rx_fifo == RT_NULL) return 0;

    level = rt_hw_interrupt_disable();
    recv_len = rt_ringbuffer_get(&(rx_fifo->rb), buffer, size);
    rt_hw_interrupt_enable(level);

    return recv_len;
}

/* HID device write (not supported) */
static rt_size_t rt_hid_write(struct rt_device *dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    /* HID keyboard is read-only */
    return 0;
}

/* HID device control */
static rt_err_t rt_hid_control(struct rt_device *dev, int cmd, void *args)
{
    /* Add control commands if needed */
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops hid_ops =
{
    rt_hid_init,
    rt_hid_open,
    rt_hid_close,
    rt_hid_read,
    rt_hid_write,
    rt_hid_control
};
#endif

/* Register HID device */
rt_err_t rt_hw_hid_register(struct rt_hid_device *hid, const char *name, rt_uint32_t flag, void *data)
{
    rt_err_t ret;
    struct rt_device *device;

    RT_ASSERT(hid != RT_NULL);

    device = &(hid->parent);

    device->type = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops = &hid_ops;
#else
    device->init = rt_hid_init;
    device->open = rt_hid_open;
    device->close = rt_hid_close;
    device->read = rt_hid_read;
    device->write = rt_hid_write;
    device->control = rt_hid_control;
#endif
    device->user_data = data;

    ret = rt_device_register(device, name, flag);

#ifdef RT_USING_POSIX
    device->fops = &_hid_fops;
#endif

    return ret;
}

/* Start HID device */
void usbh_hid_run(struct usbh_hid *hid_class)
{
    int ret;
    struct usb_interface_descriptor *intf_desc;

    /* Get interface descriptor to check protocol */
    intf_desc = &hid_class->hport->config.intf[hid_class->intf].altsetting[0].intf_desc;

    /* Only handle keyboard devices (protocol = 0x01)
     * HID_PROTOCOL_KEYBOARD = 0x01 (from usb_hid.h)
     * HID_PROTOCOL_MOUSE    = 0x02
     * HID_PROTOCOL_NONE     = 0x00 (need to parse report descriptor)
     */
    if ((HID_PROTOCOL_KEYBOARD != intf_desc->bInterfaceProtocol) ||
        (HID_SUBCLASS_BOOTIF != intf_desc->bInterfaceSubClass) ||
        (0x1 != intf_desc->bNumEndpoints) || (NULL == hid_class->intin)) {
        USB_LOG_INFO("HID device protocol 0x%02x subclass 0x%02x have %d ep num is not keyboard, skipping\n",
                   intf_desc->bInterfaceProtocol, intf_desc->bInterfaceSubClass, intf_desc->bNumEndpoints);
        return;
    }

    rt_kprintf("HID Keyboard detected (protocol=0x%02x, subclass=0x%02x)\n",
               intf_desc->bInterfaceProtocol, intf_desc->bInterfaceSubClass);

    /* Initialize HID device structure */
    rt_memset(&hid_dev, 0, sizeof(hid_dev));
    hid_dev.hid_class = hid_class;
    hid_dev.rx_bufsz = HID_RX_BUFSIZE;

    /* Register HID device */
    ret = rt_hw_hid_register(&hid_dev, "hidk0", RT_DEVICE_FLAG_RDONLY, hid_class);
    if (ret != RT_EOK) {
        USB_LOG_ERR("Failed to register HID device: %d\n", ret);
        return;
    }

    /* Setup and submit URB */
    usbh_int_urb_fill(&hid_class->intin_urb, hid_class->hport, hid_class->intin,
                      hid_buffer, hid_class->intin->wMaxPacketSize, 0,
                      usbh_hid_callback, hid_class);

    ret = usbh_submit_urb(&hid_class->intin_urb);
    if (ret < 0) {
        USB_LOG_ERR("URB submit failed: %d\n", ret);
        rt_device_unregister(&hid_dev.parent);
        return;
    }

    rt_kprintf("HID keyboard started successfully\n");
}

/* Stop HID device */
void usbh_hid_stop(struct usbh_hid *hid_class)
{
    if (!hid_dev.hid_class) {
        return;
    }

    hid_dev.hid_class = NULL;

    /* Unregister device */
    rt_device_unregister(&hid_dev.parent);

    /* Free RX FIFO if allocated */
    if (hid_dev.rx_fifo != RT_NULL) {
        rt_free(hid_dev.rx_fifo);
        hid_dev.rx_fifo = RT_NULL;
    }
}
