#include "dev_usbh_serial.h"
#include "usbh_cdc_acm.h"
#include "rtthread.h"

#define DBG_SECTION_NAME    "USBH_CDC_ACM"
#define DBG_LEVEL           DBG_INFO
#include <rtdbg.h>

typedef struct usbh_cdc_acm_serial {
    struct usbh_cdc_acm *usbh_cdc_acm;
    usbh_serial_dev_t *dev;
    rt_list_t list;
} usbh_cdc_acm_serial_t;

static rt_list_t s_serial_list = RT_LIST_OBJECT_INIT(s_serial_list);

static char* get_dev_path(void *usbh_serial) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    return usbh_cdc_acm->hport->config.intf[usbh_cdc_acm->intf].devname;
}

static int set_line_coding(void *usbh_serial, struct cdc_line_coding *line_coding) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s set_line_coding", get_dev_path(usbh_serial));
    return usbh_cdc_acm_set_line_coding(usbh_cdc_acm, line_coding);
}

static int get_line_coding(void *usbh_serial, struct cdc_line_coding *line_coding) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s get_line_coding", get_dev_path(usbh_serial));
    return usbh_cdc_acm_get_line_coding(usbh_cdc_acm, line_coding);
}

static int set_line_state(void *usbh_serial, bool dtr, bool rts) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s set_line_state dtr=%d rts=%s", get_dev_path(usbh_serial), dtr, rts);
    return usbh_cdc_acm_set_line_state(usbh_cdc_acm, dtr, rts);
}

static void set_bulk_in_transfer_buffer(void *usbh_serial, uint8_t *buffer, uint32_t buflen) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s set_bulk_in_transfer_buffer, buflen = %d", get_dev_path(usbh_serial), buflen);
    usbh_bulk_urb_fill(&usbh_cdc_acm->bulkin_urb, usbh_cdc_acm->hport, usbh_cdc_acm->bulkin, buffer, buflen, 0, NULL, NULL);
}

static void set_bulk_out_transfer_buffer(void *usbh_serial, uint8_t *buffer, uint32_t buflen) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s set_bulk_out_transfer_buffer, buflen = %d", get_dev_path(usbh_serial), buflen);
    usbh_bulk_urb_fill(&usbh_cdc_acm->bulkout_urb, usbh_cdc_acm->hport, usbh_cdc_acm->bulkout, buffer, buflen, 0, NULL, NULL);
}

static int bulk_in_transfer(void *usbh_serial, uint32_t timeout) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s bulk_in_transfer, timeout = %d", get_dev_path(usbh_serial), timeout);
    usbh_cdc_acm->bulkin_urb.timeout = timeout;
    return usbh_submit_urb(&usbh_cdc_acm->bulkin_urb);
}

static int bulk_out_transfer(void *usbh_serial, uint32_t timeout) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s bulk_out_transfer, timeout = %d", get_dev_path(usbh_serial), timeout);
    usbh_cdc_acm->bulkout_urb.timeout = timeout;
    return usbh_submit_urb(&usbh_cdc_acm->bulkout_urb);
}

static int async_bulk_in_transfer(void *usbh_serial, usbh_complete_callback_t complete, void *arg) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s async_bulk_in_transfer", get_dev_path(usbh_serial));
    usbh_cdc_acm->bulkin_urb.timeout = 0;
    usbh_cdc_acm->bulkin_urb.complete = complete;
    usbh_cdc_acm->bulkin_urb.arg = arg;
    return usbh_submit_urb(&usbh_cdc_acm->bulkin_urb);
}

static int async_bulk_out_transfer(void *usbh_serial, usbh_complete_callback_t complete, void *arg) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    LOG_D("%s async_bulk_out_transfer", get_dev_path(usbh_serial));
    usbh_cdc_acm->bulkout_urb.timeout = 0;
    usbh_cdc_acm->bulkout_urb.complete = complete;
    usbh_cdc_acm->bulkout_urb.arg = arg;
    return usbh_submit_urb(&usbh_cdc_acm->bulkout_urb);
}

static int cancel_bulk_in_transfer(void *usbh_serial) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    return usbh_kill_urb(&usbh_cdc_acm->bulkin_urb);
}

static int cancel_bulk_out_transfer(void *usbh_serial) {
    struct usbh_cdc_acm *usbh_cdc_acm = usbh_serial;
    return usbh_kill_urb(&usbh_cdc_acm->bulkout_urb);
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

void usbh_cdc_acm_run(struct usbh_cdc_acm *cdc_acm_class) {
    usbh_cdc_acm_serial_t *cdc_acm_serial = rt_malloc(sizeof(usbh_cdc_acm_serial_t));
    if (cdc_acm_serial == RT_NULL) {
        LOG_E("malloc usbh_cdc_acm_serial_t failed!");
        return;
    }

    usbh_serial_dev_t *dev = dev_usbh_serial_create(cdc_acm_class, &serial_ops, 1024 * 8);
    if (dev == RT_NULL) {
        LOG_E("create usbh serial dev failed! name = %s", get_dev_path(cdc_acm_class));
        return;
    }

    cdc_acm_serial->usbh_cdc_acm = cdc_acm_class;
    cdc_acm_serial->dev = dev;
    rt_list_init(&cdc_acm_serial->list);
    rt_list_insert_after(&s_serial_list, &cdc_acm_serial->list);
}

void usbh_cdc_acm_stop(struct usbh_cdc_acm *cdc_acm_class) {
    usbh_cdc_acm_serial_t *cdc_acm_serial;
    rt_list_for_each_entry(cdc_acm_serial, &s_serial_list, list) {
        if (cdc_acm_serial->usbh_cdc_acm = cdc_acm_class) {
            break;
        }
    }

    rt_list_remove(&cdc_acm_serial->list);

    dev_usbh_serial_destroy(cdc_acm_serial->dev);

    rt_free(cdc_acm_serial);
}