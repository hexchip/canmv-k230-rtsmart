#include "dev_usbh_serial.h"
#include "usbh_ch34x.h"
#include "rtthread.h"

#define DBG_SECTION_NAME    "USBH_SERIAL_CH34x"
#define DBG_LEVEL           DBG_INFO
#include <rtdbg.h>

typedef struct usbh_ch34x_serial {
    struct usbh_ch34x *ch34x;
    usbh_serial_dev_t *dev;
    rt_list_t list;
} usbh_ch34x_serial_t;

static rt_list_t s_serial_list = RT_LIST_OBJECT_INIT(s_serial_list);

static char* get_dev_path(void *usbh_serial) {
    struct usbh_ch34x *ch34x = usbh_serial;
    return ch34x->hport->config.intf[ch34x->intf].devname;
}

static int set_line_coding(void *usbh_serial, struct cdc_line_coding *line_coding) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s set_line_coding", get_dev_path(usbh_serial));
    return usbh_ch34x_set_line_coding(ch34x, line_coding);
}

static int get_line_coding(void *usbh_serial, struct cdc_line_coding *line_coding) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s get_line_coding", get_dev_path(usbh_serial));
    return usbh_ch34x_get_line_coding(ch34x, line_coding);
}

static int set_line_state(void *usbh_serial, bool dtr, bool rts) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s set_line_state dtr=%d rts=%s", get_dev_path(usbh_serial), dtr, rts);
    return usbh_ch34x_set_line_state(ch34x, dtr, rts);
}

static void set_bulk_in_transfer_buffer(void *usbh_serial, uint8_t *buffer, uint32_t buflen) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s set_bulk_in_transfer_buffer, buflen = %d", get_dev_path(usbh_serial), buflen);
    usbh_bulk_urb_fill(&ch34x->bulkin_urb, ch34x->hport, ch34x->bulkin, buffer, buflen, 0, NULL, NULL);
}

static void set_bulk_out_transfer_buffer(void *usbh_serial, uint8_t *buffer, uint32_t buflen) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s set_bulk_out_transfer_buffer, buflen = %d", get_dev_path(usbh_serial), buflen);
    usbh_bulk_urb_fill(&ch34x->bulkout_urb, ch34x->hport, ch34x->bulkout, buffer, buflen, 0, NULL, NULL);
}

static int bulk_in_transfer(void *usbh_serial, uint32_t timeout) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s bulk_in_transfer, timeout = %d", get_dev_path(usbh_serial), timeout);
    ch34x->bulkin_urb.timeout = timeout;
    return usbh_submit_urb(&ch34x->bulkin_urb);
}

static int bulk_out_transfer(void *usbh_serial, uint32_t timeout) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s bulk_out_transfer, timeout = %d", get_dev_path(usbh_serial), timeout);
    ch34x->bulkout_urb.timeout = timeout;
    return usbh_submit_urb(&ch34x->bulkout_urb);
}

static int async_bulk_in_transfer(void *usbh_serial, usbh_complete_callback_t complete, void *arg) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s async_bulk_in_transfer", get_dev_path(usbh_serial));
    ch34x->bulkin_urb.timeout = 0;
    ch34x->bulkin_urb.complete = complete;
    ch34x->bulkin_urb.arg = arg;
    return usbh_submit_urb(&ch34x->bulkin_urb);
}

static int async_bulk_out_transfer(void *usbh_serial, usbh_complete_callback_t complete, void *arg) {
    struct usbh_ch34x *ch34x = usbh_serial;
    LOG_D("%s async_bulk_out_transfer", get_dev_path(usbh_serial));
    ch34x->bulkout_urb.timeout = 0;
    ch34x->bulkout_urb.complete = complete;
    ch34x->bulkout_urb.arg = arg;
    return usbh_submit_urb(&ch34x->bulkout_urb);
}

static int cancel_bulk_in_transfer(void *usbh_serial) {
    struct usbh_ch34x *ch34x = usbh_serial;
    return usbh_kill_urb(&ch34x->bulkin_urb);
}

static int cancel_bulk_out_transfer(void *usbh_serial) {
    struct usbh_ch34x *ch34x = usbh_serial;
    return usbh_kill_urb(&ch34x->bulkout_urb);
}

static dev_usbh_serial_ops_t serial_ops = {
    .get_dev_path = get_dev_path,
    .set_line_coding = set_line_coding,
    .get_line_coding = get_line_coding,
    .set_line_state = set_line_state,
    .set_bulk_in_transfer_buffer = set_bulk_in_transfer_buffer,
    .set_bulk_out_transfer_buffer = set_bulk_out_transfer_buffer,
    .bulk_in_transfer = bulk_in_transfer,
    .bulk_out_transfer = bulk_out_transfer,
    .async_bulk_in_transfer = async_bulk_in_transfer,
    .async_bulk_out_transfer = async_bulk_out_transfer,
    .cancel_bulk_in_transfer = cancel_bulk_in_transfer,
    .cancel_bulk_out_transfer = cancel_bulk_out_transfer
};

void usbh_ch34x_run(struct usbh_ch34x *ch34x_class) {
    usbh_ch34x_serial_t *ch34x_serial = rt_malloc(sizeof(usbh_ch34x_serial_t));
    if (ch34x_serial == RT_NULL) {
        LOG_E("malloc usbh_ch34x_serial_t failed!");
        return;
    }

    usbh_serial_dev_t *dev = dev_usbh_serial_create(ch34x_class, &serial_ops, 1024 * 8);
    if (dev == RT_NULL) {
        LOG_E("create usbh serial dev failed! name = %s", get_dev_path(ch34x_class));
        return;
    }

    ch34x_serial->ch34x = ch34x_class;
    ch34x_serial->dev = dev;
    rt_list_init(&ch34x_serial->list);
    rt_list_insert_after(&s_serial_list, &ch34x_serial->list);

    LOG_I("connected");
}

void usbh_ch34x_stop(struct usbh_ch34x *ch34x_class) {
    usbh_ch34x_serial_t *ch34x_serial;
    rt_list_for_each_entry(ch34x_serial, &s_serial_list, list) {
        if (ch34x_serial->ch34x = ch34x_class) {
            break;
        }
    }

    rt_list_remove(&ch34x_serial->list);

    dev_usbh_serial_destroy(ch34x_serial->dev);

    rt_free(ch34x_serial);

    LOG_I("disconnected");
}